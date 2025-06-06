#include <hal.hh>

namespace {

hal::Gpio s_adc_cs(hal::GpioPort::A, 8);
hal::Gpio s_eeprom_wc(hal::GpioPort::A, 9);
hal::Gpio s_current_switch(hal::GpioPort::A, 10);
hal::Gpio s_oc_n(hal::GpioPort::A, 11);
hal::Gpio s_oc_p(hal::GpioPort::A, 12);
hal::Gpio s_charge_en(hal::GpioPort::B, 0);
hal::Gpio s_shutdown(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

// SPI pins.
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);

} // namespace

void app_main() {
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    while (true) {
        hal::gpio_set(s_led);
        hal::delay_us(1000000);
        hal::gpio_reset(s_led);
        hal::delay_us(1000000);
    }
}
