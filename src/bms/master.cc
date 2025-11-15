#include <hal.hh>
#include <stm32f103xb.h>

#include <FreeRTOS.h>
#include <task.h>

namespace {

hal::Gpio s_led(hal::GpioPort::B, 5);

void blink_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    while (true) {
        hal::swd_printf("time: %u\n", pdTICKS_TO_MS(xTaskGetTickCount()));
        s_led.write(!s_led.read());
        xTaskDelayUntil(&wake_time, pdMS_TO_TICKS(500));
    }
}

} // namespace

void app_main() {
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    xTaskCreate(&blink_task, "blink", 512, nullptr, 1, nullptr);
    vTaskStartScheduler();
    while (true) {
        __NOP();
    }
}
