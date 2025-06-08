#include <can.hh>
#include <hal.hh>
#include <stm32f103xb.h>

namespace {

hal::Gpio s_lv_reading(hal::GpioPort::A, 1);
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
hal::Gpio s_adc_cs(hal::GpioPort::A, 8);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);

} // namespace

void app_main() {
    // Configure LED and 12 volt rail measurement pin.
    s_lv_reading.configure(hal::GpioInputMode::Analog);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // These outputs are open-drain as they require 5 volts logic high, and so have external pull-ups.
    s_eeprom_wc.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);
    s_current_switch.configure(hal::GpioOutputMode::OpenDrain, hal::GpioOutputSpeed::Max2);

    // Configure overcurrent inputs as floating as they have external pull-ups.
    s_oc_n.configure(hal::GpioInputMode::Floating);
    s_oc_p.configure(hal::GpioInputMode::Floating);

    // Charge enable and shutdown are push-pull. They have external pull-downs in case the MCU has a problem.
    s_charge_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_shutdown.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure I2C pins.
    for (const auto pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
        pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    }

    // Configure ADC SPI pins. Enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Default open drain pins to high impedance state.
    hal::gpio_set(s_eeprom_wc, s_current_switch);

    // Initialise CAN on PB8 (RX) and PB9 (TX).
    if (!can::init(can::Port::B, can::Speed::_500)) {
        // TODO: Handle failure.
    }
}
