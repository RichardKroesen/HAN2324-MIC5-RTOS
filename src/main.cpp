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

#include "board_defines.h"

#include <bgt60ltr11XXX_driver.hpp>
#include <step_motor.hpp>
#include <audio_buffer.hpp>
#include <speaker_controller.hpp>
#include <sample.h>
#include <UART_streamer.hpp>

TaskHandle_t Task1Handle = NULL;

static SERIAL::UART_RTOS_Driver driver(115200, 16, 17);

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

        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

void stepMotor_task(void *Pvarg) {
    static MOTOR::StepMotorDriver motor{STEP_MOTOR_PIN,DIR_MOTOR_PIN,EN_MOTOR_PIN}; /// (0, 1);
    motor.initPulseGenerator(0, 30, 100);
    motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_90);
    gpio_init( 25 );
    gpio_set_dir( 25, GPIO_OUT );
    
    while (true)
    {
        uint32_t speed = motor.getSpeed_rpm();
        printf("Speed: %d deg/sec.\n", speed);
        gpio_put(25, 1);
        vTaskDelay(5000/portTICK_PERIOD_MS);
        motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_90);
        gpio_put(25, 0);
        vTaskDelay(5000/portTICK_PERIOD_MS);
        motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_360);
    }
}

void AudioTask(void *pvParameters) {
    vTaskDelay(5000/portTICK_PERIOD_MS);
    AUDIO::AudioBuffer buffer(WAV_DATA, WAV_DATA_LENGTH);

    uint8_t audioPin = AUDIO_PIN; 
    uint32_t pwmWrapValue = 255;  
    float clkDivValue = 5.55f; 

    AUDIO::SpeakerController speakerController(audioPin, pwmWrapValue, clkDivValue, buffer);

    while (true) {
        speakerController.start();
        vTaskDelay(5000/portTICK_PERIOD_MS);
        speakerController.stop();
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
};

void taskNotify(void *taskvParameters) {
    uint32_t notificationValue;

    for (;;) {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, portMAX_DELAY)) { 
            printf("[Task Notify]: Motion Detected.\n");
        }
    }
}

void mainTask(void *params) {
    SERIAL::uart_buffer_t rxBuffer;

    while (1) {
        // Check for received UART data without blocking
        if (driver.uart_check_rx_non_blocking(&rxBuffer, sizeof(rxBuffer), pdMS_TO_TICKS(100))) {
            // Process received UART data
            printf("Received UART data: %s\n", rxBuffer.data);
        } else {
            // Yield to other tasks
            taskYIELD();
        }
    }
}

void uart_send_task(void *params) {
    const char* message = "Hello, UART World!\n";
    driver.uart_send_non_blocking(message);

    while (1) {
        driver.uart_send_non_blocking(message); // Non-blocking send
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

int main() {
    stdio_init_all();

    // printf("\tProgram Start\n");
    // while (! tud_cdc_connected()) {
    //     printf(".");
    //     sleep_ms(500);
    // }

    printf("\tProgram Start\n");

    BaseType_t xReturned[6];

    xReturned[0] = xTaskCreate(display_task, "DisplayTask", 900, NULL, 3, NULL);
    xReturned[1] = xTaskCreate(radarReading_task, "RadarReadingTask", 400, NULL, 2, NULL);
    xReturned[2] = xTaskCreate(stepMotor_task, "MotorTask", 150, NULL, 1, NULL);
    xReturned[3] = xTaskCreate(AudioTask, "AudioTask", 400, NULL, 3, NULL);
    xReturned[4] = xTaskCreate(mainTask, "MainTask", 200, NULL, 1, NULL); 
    xReturned[5] = xTaskCreate(uart_send_task, "UARTSendTask", 200, NULL, 2, NULL);

    // xTaskCreate(taskNotify, "Task1", 200, NULL, 1, &Task1Handle);

    // bool flag = false;
    // for (int i = 0; i < 5; i++) {
    //     if (xReturned[i] != pdPASS) {
    //         printf("Task %d  creation failed.\n", i);
    //         flag = false;
    //         break;
    //     } else {
    //         flag = true;
    //     }
    // }

    vTaskStartScheduler();

    // if (flag) {
    //     printf("Scheduling Started.\n\n");
    //     vTaskStartScheduler();
    // } else {
    //     printf("Tasks are not scheduled.\n");
    // }
    
    for (;;) {
        ;
    }
    
    return 0;
}

