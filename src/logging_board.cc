#include <hal.hh>
#include <pb_encode.h>
#include <vehicle_data.pb.h>

#include <array>
#include <cstdint>
#include <span>
#include <string_view>

// gpio for status led and usart2 pins (and optional rts/cts)
hal::Gpio s_led(hal::GpioPort::B, 12);
hal::Gpio s_cts(hal::GpioPort::A, 0);
hal::Gpio s_rts(hal::GpioPort::A, 1);
hal::Gpio tx(hal::GpioPort::A, 2);
hal::Gpio rx(hal::GpioPort::A, 3);

// crc-16-ccitt (poly 0x1021, init 0xffff, no reflect, no final xor)
static uint16_t crc16_ccitt(std::span<const uint8_t> data) {
    uint16_t crc = 0xFFFF; // init
    for (uint8_t b : data) {
        crc ^= static_cast<uint16_t>(b) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc; // big-endian customary when appending
}

// send a text string over usart2, blocking
void UART_send_string(std::string_view text) {
    for (char ch : text) {
        USART2->DR = ch;
        while (!(USART2->SR & USART_SR_TC))
            ;
    }
}

// send raw bytes over usart2, blocking
void UART_send_bytes(std::span<const uint8_t> bytes) {
    for (uint8_t byte : bytes) {
        USART2->DR = byte;
        while (!(USART2->SR & USART_SR_TC))
            ;
    }
}

// cobs-like framing, with crc16 appended before stuffing
void queue_message(std::span<std::uint8_t> input_data) {
    // build payload = data || crc_hi || crc_lo
    std::array<uint8_t, 258> payload{}; // Test_size <= 255 typical; +2 crc
    size_t plen = 0;

    // copy data
    const size_t n = input_data.size();
    if (n > 255) {
        // optional: clamp or split; here we clamp to first 255 bytes
    }
    for (size_t i = 0; i < n; ++i) {
        payload[plen++] = input_data[i];
    }

    // compute and append crc
    const uint16_t crc = crc16_ccitt(std::span<const uint8_t>(payload.data(), plen));
    payload[plen++] = static_cast<uint8_t>((crc >> 8) & 0xFF); // crc hi
    payload[plen++] = static_cast<uint8_t>(crc & 0xFF);        // crc lo

    // cobs-like stuff into fixed buffer and send
    std::array<std::uint8_t, 256 + 4> stuffed{}; // allow a bit extra
    std::uint8_t code_index = 0;
    std::uint8_t write_index = 0;

    stuffed[write_index++] = 1; // first code byte
    for (size_t i = 0; i < plen; ++i) {
        const uint8_t byte = payload[i];
        if (byte != 0) {
            stuffed[write_index++] = byte;
            stuffed[code_index]++;
            if (stuffed[code_index] == 0xFF) { // block full (254 non-zeros)
                code_index = write_index;
                stuffed[write_index++] = 1;
            }
        } else {
            code_index = write_index;
            stuffed[write_index++] = 1;
        }
    }

    stuffed[write_index++] = 0; // terminator
    UART_send_bytes(std::span<const uint8_t>(stuffed.data(), write_index));
}

// main app: init gpio/usart, encode nanopb message, blink and transmit
void app_main() {
    // gpio config
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    rx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    // note: rts/cts declared but not used

    hal::delay_us(100000); // small delay

    // usart2 basic setup
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->CR1 |= USART_CR1_UE;
    USART2->BRR = 484;
    USART2->CR1 |= USART_CR1_RE;
    USART2->CR1 |= USART_CR1_TE;

    // build a test protobuf message
    Test example = {42};
    std::array<uint8_t, Test_size> buffer{}; // encoded payload

    // encode with nanopb
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());
    bool encoded = pb_encode(&stream, Test_fields, &example);
    // stream.bytes_written holds valid length when encoded == true

    if (encoded) {
        hal::gpio_set(s_led); // indicate successful encode
    }

    while (true) {
        hal::gpio_set(s_led);
        hal::delay_us(500000);
        hal::gpio_reset(s_led);
        hal::delay_us(100000);

        // send only the bytes actually written by nanopb
        std::span<uint8_t> payload_span(buffer.data(), encoded ? stream.bytes_written : 0);
        queue_message(payload_span);
    }
}
