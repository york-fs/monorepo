#include <hal.hh>
#include <string_view>
#include <pb_encode.h>
#include "bms.hh"
#include "vehicle_data.pb.h"




hal:: Gpio s_led(hal::GpioPort::B, 12);
hal::Gpio s_cts(hal::GpioPort::A, 0);
hal::Gpio s_rts(hal::GpioPort::A, 1);
hal:: Gpio tx(hal::GpioPort::A, 2);
hal:: Gpio rx(hal::GpioPort::A, 3);


void UART_send_string(std:: string_view text) {
    for (char ch : text) {
        USART2 -> DR = ch ;
        while(!(USART2 -> SR & USART_SR_TC));
    }

}

void UART_send_bytes(std::span <uint8_t>bytes) {
    for (uint8_t byte : bytes) {
        USART2 -> DR = byte;
        while(!(USART2 -> SR & USART_SR_TC));
    }

}

void queue_message(std::span<std::uint8_t> input_data)  {
    std::array<std::uint8_t, 256> stuffed{}; // Creates a fixed 256-byte output buffer

    std::uint8_t code_index = 0;  // Position of current byte
    std::uint8_t write_index = 0; // Next write position

    stuffed[write_index++] = 1; // Initialise first code byte
    for (const std::uint8_t byte : input_data) {
        if (byte != 0) {
            // Non-zero data byte - write it directly and increase the non-zero count.

            stuffed[write_index++] = byte; // Copy non-zero byte
            stuffed[code_index]++;  // Increment count at code position
        } else {
            code_index = write_index;
            stuffed[write_index++] = 1;
        }
        }

        stuffed[write_index] = 0; // Final delimiter

        UART_send_bytes(std::span(stuffed).subspan(0, write_index + 1));
}

void app_main() {
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    rx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);

    hal::delay_us(100000);
    RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2 -> CR1 |= USART_CR1_UE;
    USART2 -> BRR = 484;
    USART2 -> CR1 |= USART_CR1_RE;
    USART2 -> CR1 |= USART_CR1_TE;

    Test example = {42};
    std::array <uint8_t, Test_size> buffer{}; // angle brackets mean its done duing compile time

    // Creating an output stream with the byte buffer
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());
    bool encoded = pb_encode(&stream, Test_fields, &example);

    std::array<std::uint8_t, 4> test_data{0x11, 0x22, 0x00, 0x44};

    if (encoded) {
        hal::gpio_set(s_led);
    }



    while (true) {
       hal::gpio_set(s_led);
       hal::delay_us(500000);
       hal::gpio_reset(s_led);
       hal::delay_us(100000);

        // UART_send_string("Hello World\r\n");
        queue_message(test_data);
    }

}
