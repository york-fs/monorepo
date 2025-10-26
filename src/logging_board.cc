#include <hal.hh>
#include <pb_encode.h>
#include <vehicle_data.pb.h>

#include <array>
#include <cstdint>
#include <span>
#include <string_view>

namespace {

auto *const s_sd_spi = SPI1;

hal::Gpio s_sd_detect(hal::GpioPort::A, 4);
hal::Gpio s_radio_led(hal::GpioPort::B, 12);
hal::Gpio s_sd_led(hal::GpioPort::B, 13);

// SPI pins.
hal::Gpio s_sd_cs(hal::GpioPort::B, 0);
hal::Gpio s_sck(hal::GpioPort::A, 5);
hal::Gpio s_miso(hal::GpioPort::A, 6);
hal::Gpio s_mosi(hal::GpioPort::A, 7);

// UART pins.
hal::Gpio s_cts(hal::GpioPort::A, 0);
hal::Gpio s_rts(hal::GpioPort::A, 1);
hal::Gpio s_tx(hal::GpioPort::A, 2);
hal::Gpio s_rx(hal::GpioPort::A, 3);

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

std::uint8_t sd_crc7(std::span<const std::uint8_t> data) {
    std::uint8_t crc = 0;
    for (std::uint8_t byte : data) {
        crc ^= byte;
        for (std::uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x80) != 0) {
                crc ^= 0x89;
            }
            crc <<= 1;
        }
    }
    return crc >> 1;
}

void sd_send_command(std::uint8_t command_index, std::uint32_t argument) {
    std::array<std::uint8_t, 32> bytes;
    std::fill(bytes.begin(), bytes.end(), 0xff);

    // First byte is the start bit (0), transmission bit (1), and 6 bits of command index.
    bytes[0] = (1u << 6) | command_index;

    // Copy argument MSB first.
    bytes[1] = (argument >> 24) & 0xffu;
    bytes[2] = (argument >> 16) & 0xffu;
    bytes[3] = (argument >> 8) & 0xffu;
    bytes[4] = (argument >> 0) & 0xffu;

    // CRC7 is in the top 7 bits followed by the stop bit.
    bytes[5] = (sd_crc7(std::span(bytes).subspan(0, 5)) << 1) | 1u;

    // TODO: Check for errors.
    hal::spi_transfer(s_sd_spi, s_sd_cs, bytes, 10);
}

void sd_init() {
    // Deactivate CS driving whilst doing the wake-up routine.
    s_sd_cs.configure(hal::GpioInputMode::Floating);

    // Wake-up the SD card. Spec mandates 74 clock cycles, so round up to 80 (10 bytes * 8 bits).
    std::array<std::uint8_t, 10> wake_up_bytes;
    std::fill(wake_up_bytes.begin(), wake_up_bytes.end(), 0xff);
    hal::spi_transfer(s_sd_spi, s_sd_cs, wake_up_bytes, 10);

    // Re-enable CS.
    s_sd_cs.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);

    // TODO: Check for responses for commands below.

    // Send GO_IDLE_STATE.
    sd_send_command(0, 0);

    // Send SEND_IF_COND with 3V3 supply voltage.
    sd_send_command(8, 0x1aa);

    while (true) {
        std::array<std::uint8_t, 32> app_cmd{0x77, 0x00, 0x00, 0x00, 0x00, 0xff};
        std::fill(app_cmd.begin() + 6, app_cmd.end(), 0xff);
        hal::spi_transfer(SPI1, s_sd_cs, app_cmd, 100);
        for (std::uint8_t byte : app_cmd) {
            hal::swd_printf("received app_cmd byte: 0x%x\n", byte);
        }

        std::array<std::uint8_t, 32> send_op_cond{0x69, 0x40, 0x00, 0x00, 0x00, 0xff};
        std::fill(send_op_cond.begin() + 6, send_op_cond.end(), 0xff);
        hal::spi_transfer(SPI1, s_sd_cs, send_op_cond, 100);
        for (std::uint8_t byte : send_op_cond) {
            hal::swd_printf("received send_op_cond byte: 0x%x\n", byte);
        }

        std::uint8_t response = 0xff;
        for (int i = 6; i < 14; i++) {
            if (send_op_cond[i] != 0xff) {
                response = send_op_cond[i];
                break;
            }
        }

        hal::swd_printf("response: 0x%x\n", response);

        if (response == 0) {
            break;
        }

        hal::delay_us(100000);
    }

    std::array<std::uint8_t, 36> cid_info{0x4a, 0x00, 0x00, 0x00, 0x00, 0xff};
    std::fill(cid_info.begin() + 6, cid_info.end(), 0xff);
    hal::spi_transfer(SPI1, s_sd_cs, cid_info, 10);

    for (std::uint8_t byte : cid_info) {
        hal::swd_printf("received CID info byte: 0x%x %c\n", byte, byte);
    }

    auto cid_data = std::span(cid_info).subspan(11);

    const std::uint8_t manufacturer_id = cid_data[0];
    std::array<char, 3> oem_id{
        cid_data[1],
        cid_data[2],
        '\0',
    };
    std::array<char, 6> product_name{
        cid_data[3], cid_data[4], cid_data[5], cid_data[6], cid_data[7], '\0',
    };
    std::uint8_t major_version = (cid_data[8] >> 4) & 0xfu;
    std::uint8_t minor_version = cid_data[8] & 0xfu;
    std::uint32_t serial_number =
        (static_cast<std::uint32_t>(cid_data[9]) << 24) | (static_cast<std::uint32_t>(cid_data[10]) << 16) |
        (static_cast<std::uint32_t>(cid_data[11]) << 8) | static_cast<std::uint32_t>(cid_data[12]);
    std::uint16_t manufacture_date = (static_cast<std::uint16_t>(cid_data[13] & 0xfu) << 8) | cid_data[14];
    std::uint8_t crc = (cid_data[15] >> 1) & 0x7fu;

    std::uint16_t year = 2000 + ((manufacture_date >> 4) & 0xffu);
    std::uint8_t month = manufacture_date & 0xfu;

    hal::swd_printf("Manufacturer ID: 0x%02x\n", manufacturer_id);
    hal::swd_printf("OEM: %s\n", oem_id.data());
    hal::swd_printf("Product name: %s\n", product_name.data());
    hal::swd_printf("Product version: %u.%u\n", major_version, minor_version);
    hal::swd_printf("Serial number: 0x%08x\n", serial_number);
    hal::swd_printf("Manufacture date: %02u/%04u\n", month, year);
    hal::swd_printf("Given CRC: 0x%02x\n", crc);
    hal::swd_printf("Calculated CRC: 0x%02x\n", sd_crc7(cid_data.subspan(0, 15)));
}

} // namespace

// main app: init gpio/usart, encode nanopb message, blink and transmit
void app_main() {
    // Drive RTS high. This works around a missing hardware pull-up.
    s_rts.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    hal::gpio_set(s_rts);

    // GPIO config.
    s_sd_detect.configure(hal::GpioInputMode::PullUp);
    s_radio_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_sd_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_tx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);
    s_rx.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max2);

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

    // Configure SPI peripheral.
    s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);
    s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max50);
    s_miso.configure(hal::GpioInputMode::Floating);
    hal::spi_init_master(SPI1, SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);

    while (true) {
        hal::swd_printf("Waiting for SD card\n");
        while (s_sd_detect.read()) {
            hal::gpio_set(s_sd_led);
            hal::delay_us(100000);
            hal::gpio_reset(s_sd_led);
            hal::delay_us(100000);
        }
        hal::gpio_set(s_sd_led);

        sd_init();

        // Wait for card unplug.
        while (!s_sd_detect.read()) {
            __NOP();
        }
    }
}
