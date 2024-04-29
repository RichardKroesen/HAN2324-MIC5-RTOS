#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "tusb.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include <task.h>

#include <lv_drv_conf.h>
#include <lvgl.h>
#include "lv_port_disp.h"
#include "ui.h"

#include <bgt60ltr11XXX_driver.hpp>

TaskHandle_t Task1Handle = NULL;

void error_handler() {
    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();
    while (1) {
        ;
    }
}

void display_task(void *Pvarg) {
    lv_init();
    lv_port_disp_init();
    ui_init();

    while (1) {
        lv_tick_inc(1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        lv_task_handler();
    }
}

void radarReading_task(void *taskvParameters) {
    BGT60::BGT60_DRIVER radarDriver{};
    //UART_STREAMER::UartStreamer streamer{};

    radarDriver.initCwMode();

    uint16_t valueQ = 0;
    uint16_t valueI = 0;

    while (true) {
        valueQ = BGT60::BGT60_DRIVER::read_AdChannel(BGT60::ADC_REG_CHANNELS::IFQ);
        valueI = BGT60::BGT60_DRIVER::read_AdChannel(BGT60::ADC_REG_CHANNELS::IFI);
        
        if (valueQ != 0 && valueI != 0) {
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "%d,%d\n", valueQ, valueI);
            printf(buffer);
        }

        // if (valueQ > 200 && valueI > 200) {
        //     xTaskNotify(Task1Handle, 0x01, eSetValueWithOverwrite);
        // }

        vTaskDelay(150/portTICK_PERIOD_MS);
    }
}

void taskNotify(void *taskvParameters) {
    uint32_t notificationValue;

    for (;;) {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, portMAX_DELAY)) { 
            printf("[Task Notify]: Motion Detected.\n");
        }
    }
}

int main() {
    stdio_init_all();

    printf("waiting for usb host");
    while (! tud_cdc_connected()) {
        printf(".");
        sleep_ms(500);
    }

    xTaskCreate(display_task, "display_task", 750, NULL, 3, NULL);
    xTaskCreate(radarReading_task, "RadarReading", 400, NULL, 2, NULL);
    // xTaskCreate(taskNotify, "Task1", 200, NULL, 1, &Task1Handle);

    vTaskStartScheduler();
    
    for (;;) {
        ;
    }
    
    return 0;
}

