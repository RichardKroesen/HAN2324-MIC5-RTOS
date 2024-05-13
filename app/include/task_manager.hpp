#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP    

#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "tusb.h"

#if ( mainRUN_ON_CORE == 1 )
#include "pico/multicore.h"
#endif

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

#include <lv_drv_conf.h>
#include <lvgl.h>
#include "lv_port_disp.h"
#include "ui.h"

#include "board_defines.h"

#include <controller.hpp>
#include <bgt60ltr11XXX_driver.hpp>
#include <step_motor.hpp>
#include <audio_buffer.hpp>
#include <speaker_controller.hpp>
#include <sample.h>
#include <UART_streamer.hpp>

namespace CONTROLLER {
class TaskManager {
public:
    TaskManager() {
        driver = new CoreDriver();
        driver->initialize();
        setup_gpio();

        switch_event_group = xEventGroupCreate();
        if (switch_event_group == NULL) {
            printf("Event group creation failed.\n");
        }
    }

    void enableSystem() {
        createTasks();
        driver->startScheduler();
    }

    ~TaskManager() {
        delete driver;
    }
        
    static void display_task(void *Pvarg) {
        lv_init();
        lv_port_disp_init();
        ui_init();

        while (1) {
            lv_tick_inc(1);
            vTaskDelay(1 / portTICK_PERIOD_MS);
            lv_task_handler();
        }
    }

    static void radarReading_task(void *taskvParameters) {
        BGT60::BGT60_DRIVER radarDriver{};
        vTaskDelay(1000 / portTICK_PERIOD_MS);

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

    static void stepMotor_task(void *Pvarg) {
        static MOTOR::StepMotorDriver motor{STEP_MOTOR_PIN, DIR_MOTOR_PIN, EN_MOTOR_PIN}; /// (0, 1);
        motor.initPulseGenerator(1, 30, 250);
        motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_90);
        
        while (true)
        {
            uint32_t speed = motor.getSpeed_rpm();
            printf("Speed: %d deg/sec.\n", speed);
            vTaskDelay(5000/portTICK_PERIOD_MS);
            motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_90);
            vTaskDelay(5000/portTICK_PERIOD_MS);
            motor.rotateAngle(MOTOR::DefaultAngles::ANGLE_360);
        }
    }

    static void AudioTask(void *pvParameters) {
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

    static void taskNotify(void *taskvParameters) {
        uint32_t notificationValue;

        for (;;) {
            if (xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, portMAX_DELAY)) { 
                printf("[Task Notify]: Motion Detected.\n");
            }
        }
    }

    static void mainTask(void *params) {
        SERIAL::uart_buffer_t rxBuffer;

        while (1) {
            // Check for received UART data without blocking
            if (uartStreamer.uart_check_rx_non_blocking(&rxBuffer, sizeof(rxBuffer), pdMS_TO_TICKS(100))) {
                // Process received UART data
                printf("Received UART data: %s\n", rxBuffer.data);
            } else {
                // Yield to other tasks
                taskYIELD();
            }
        }
    }

    static void uart_send_task(void *params) {
        const char* message = "Hello, UART World!\n";
        uartStreamer.uart_send_non_blocking(message);

        while (1) {
            uartStreamer.uart_send_non_blocking(message); // Non-blocking send
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }

    static void prvLedTask(void *pvParameters) {
        TickType_t xNextWakeTime;
        xNextWakeTime = xTaskGetTickCount();

        for( ;; ) {
            printf("LED Task\n");
            vTaskDelayUntil( &xNextWakeTime, (1000 / portTICK_PERIOD_MS));
            gpio_xor_mask(1u << (uint32_t)pvParameters);
        }
    }
        
    static void led_control_task(void *params) {
        const static uint8_t led_pins[IO_DEMO_PINS] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
        const static uint8_t bits_of_interest = 0x0F;
        EventBits_t bits;

        while (true) {
            // Wait for any bit to be set here: 
            bits = xEventGroupWaitBits(switch_event_group, bits_of_interest, pdTRUE, pdFALSE, portMAX_DELAY);
            
            for (int i = 0; i < IO_DEMO_PINS; i++) {
                gpio_put(led_pins[i], (bits & (1 << i)) ? 1 : 0);
            }
        }
    }

private: 
    /* Class Defaults */
    TaskManager(const TaskManager &other) = delete;
    TaskManager &operator=(const TaskManager &other) = delete;

    /* Driver Variables: */
    CoreDriver* driver;
    static inline SERIAL::UART_RTOS_Driver uartStreamer{UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN};

    /* RTOS Variables:*/
    static inline EventGroupHandle_t switch_event_group = nullptr;
    const static inline uint8_t CORE0 = 0;  
    const static inline uint8_t CORE1 = 1;

    void createTasks() {
        TaskHandle_t task_handle;

        driver->addTask(&task_handle, "RadarReadingTask", 400, radarReading_task, NULL, 2);
        driver->addTask(&task_handle, "MotorTask", 400, stepMotor_task, NULL, 1);
        driver->addTask(&task_handle, "AudioTask", 400, AudioTask, NULL, 1);
        driver->addTask(&task_handle, "UARTReceiveTask", 400, mainTask, NULL, 3);
        driver->addTask(&task_handle, "UARTSendTask", 200, uart_send_task, NULL, 2);

        driver->setTaskCore(task_handle, CORE0);
        driver->addTask(&task_handle, "LedControlTask", led_control_task, NULL, 2);
        driver->addTask(&task_handle, "LedTask", prvLedTask, (void *)25, 1);
        driver->addTask(&task_handle, "DisplayTask", 900, display_task, NULL, 3);
        driver->setTaskCore(task_handle, CORE1);
    }

    static void gpio_callback(uint gpio, uint32_t events) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (gpio == SWITCH1_PIN) {
            xEventGroupSetBitsFromISR(switch_event_group, 1 << 0, &xHigherPriorityTaskWoken);
        } else if (gpio == SWITCH2_PIN) {
            xEventGroupSetBitsFromISR(switch_event_group, 1 << 1, &xHigherPriorityTaskWoken);
        } else if (gpio == SWITCH3_PIN) {
            xEventGroupSetBitsFromISR(switch_event_group, 1 << 2, &xHigherPriorityTaskWoken);
        } else if (gpio == SWITCH4_PIN) {
            xEventGroupSetBitsFromISR(switch_event_group, 1 << 3, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void setup_gpio() {
        const static uint8_t switch_pins[IO_DEMO_PINS] = {SWITCH1_PIN, SWITCH2_PIN, SWITCH3_PIN, SWITCH4_PIN};
        const static uint8_t led_pins[IO_DEMO_PINS] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};

        for (uint8_t pin : switch_pins) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_up(pin);
            gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
        }

        for (uint8_t pin : led_pins) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, 0);
        }
    }
};

}; // End of CONTROLLER namespace.

#endif // TASK_MANAGER_HPP