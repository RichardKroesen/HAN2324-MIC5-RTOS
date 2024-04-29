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

int main() {
    stdio_init_all();

    printf("waiting for usb host");
    while (! tud_cdc_connected()) {
        printf(".");
        sleep_ms(500);
    }

    xTaskCreate(display_task, "display_task", 750, NULL, 3, NULL);
    vTaskStartScheduler();
    
    for (;;) {
        ;
    }
    return 0;
}

