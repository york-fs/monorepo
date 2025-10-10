#include <hal.hh>
#include <string_view>
#include <pb_encode.h>
#include "bms.hh"
#include "vehicle_data.pb.h"

// gpio for status led and usart2 pins (and optional rts/cts)
hal::Gpio s_led(hal::GpioPort::B, 12);
hal::Gpio s_cts(hal::GpioPort::A, 0);
hal::Gpio s_rts(hal::GpioPort::A, 1);
hal::Gpio tx(hal::GpioPort::A, 2);
hal::Gpio rx(hal::GpioPort::A, 3);

// send a null-free text string over usart2, blocking per char
void UART_send_string(std::string_view text) {
    for (char ch : text) {
        USART2->DR = ch;                    // write data register
        while (!(USART2->SR & USART_SR_TC)) // wait for tx complete
            ;
    }
}

// send raw bytes over usart2, blocking; no flow control
void UART_send_bytes(std::span<uint8_t> bytes) {
    for (uint8_t byte : bytes) {
        USART2->DR = byte;                  // write data register
        while (!(USART2->SR & USART_SR_TC)) // wait for tx complete
            ;
    }
}

// cobs-like framing with zero as delimiter; outputs only used bytes
void queue_message(std::span<std::uint8_t> input_data) {
    std::array<std::uint8_t, 256> stuffed{}; // fixed output buffer

    std::uint8_t code_index = 0;  // current code byte position
    std::uint8_t write_index = 0; // next write position

    stuffed[write_index++] = 1;   // init first code byte
    for (const std::uint8_t byte : input_data) {
        if (byte != 0) {
            // non-zero: copy and increment run length
            stuffed[write_index++] = byte;
            stuffed[code_index]++; // count of non-zeros
        } else {
            // zero: start new block, reserve code byte
            code_index = write_index;
            stuffed[write_index++] = 1; // placeholder count
        }
    }

    stuffed[write_index] = 0; // frame terminator

    // send only the filled portion (including terminator)
    UART_send_bytes(std::span(stuffed).subspan(0, write_index + 1));
}

// main app: init gpio/usart, encode nanopb message, blink and transmit
void app_main() {
    // gpio config
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    rx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    // note: rts/cts lines declared but not configured/used here

    hal::delay_us(100000); // small settle delay before peripheral setup

    // usart2 enable and basic setup (assumes 8n1, no hw flow control)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // enable peripheral clock
    USART2->CR1 |= USART_CR1_UE;          // usart enable
    USART2->BRR = 484;                    // baud rate divisor (board specific)
    USART2->CR1 |= USART_CR1_RE;          // rx enable
    USART2->CR1 |= USART_CR1_TE;          // tx enable

    // build a test protobuf message
    Test example = {42};
    std::array<uint8_t, Test_size> buffer{}; // compile-time sized buffer

    // encode message into buffer using nanopb
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());
    bool encoded = pb_encode(&stream, Test_fields, &example);

    if (encoded) {
        hal::gpio_set(s_led); // indicate successful encode
    }

    // main loop: blink led and send framed payload
    while (true) {
        hal::gpio_set(s_led);
        hal::delay_us(500000);
        hal::gpio_reset(s_led);
        hal::delay_us(100000);

        // UART_send_string("hello world\r\n"); // optional plain text send
        queue_message(buffer); // send encoded payload with framing
    }
}
