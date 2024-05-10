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

namespace CONTROLLER {

class CoreDriver {
public:
    CoreDriver() = default;
    ~CoreDriver() = default;

    static inline void initialize() {
        stdio_init_all();
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, HIGH);
    }

    static inline bool addTask(TaskHandle_t *taskHandle, const char *taskName, TaskFunction_t taskFunction, void *taskParameters, UBaseType_t taskPriority) {
        BaseType_t xReturned = pdFAIL;
        xReturned = xTaskCreate(taskFunction, taskName, configMINIMAL_STACK_SIZE, taskParameters, taskPriority, taskHandle);
        if (xReturned != pdPASS) {
            serialDebugPrint("Task creation failed.\n");
            return false;
        }
        return true; 
    }

    static inline bool addTask(TaskHandle_t *taskHandle, const char *taskName, const uint32_t stackDepth, TaskFunction_t taskFunction, void *taskParameters, UBaseType_t taskPriority) {
        BaseType_t xReturned = pdFAIL;
        xReturned = xTaskCreate(taskFunction, taskName, stackDepth, taskParameters, taskPriority, taskHandle);
        if (xReturned != pdPASS) {
            serialDebugPrint("Task creation failed.\n");
            return false; 
        }
        return true; 
    }

    static inline void removeTask(TaskHandle_t taskHandle) {
        vTaskDelete(taskHandle);
    }

    static inline void startScheduler() {
        vTaskStartScheduler();
    }

    static inline void stopScheduler() {
        vTaskEndScheduler();
    }

    static inline void setTaskCore(TaskHandle_t taskHandle, const uint8_t coreId) {
        vTaskCoreAffinitySet(taskHandle, (1 << coreId));
    }

    static inline void notifyTask(TaskHandle_t taskHandle, uint32_t notificationValue) {
        xTaskNotify(taskHandle, notificationValue, eSetValueWithOverwrite);
    }

    static inline void delay(const uint32_t delayTime_ms) {
        vTaskDelay(delayTime_ms / portTICK_PERIOD_MS);
    }

    static inline void serialDebugPrint(const char *message) {
        printf(message);
    }

private:
    CoreDriver(const CoreDriver &other) = delete;
    CoreDriver &operator=(const CoreDriver &other) = delete;

    const static inline uint8_t LED_PIN = 25;
};

class TaskManager {
public:
    TaskManager() {
        driver = new CoreDriver();
        driver->initialize();
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

private: 
    TaskManager(const TaskManager &other) = delete;
    TaskManager &operator=(const TaskManager &other) = delete;

    CoreDriver* driver;
    static inline SERIAL::UART_RTOS_Driver uartStreamer{115200, 16, 17};
    const static inline uint8_t CORE0 = 0;  
    const static inline uint8_t CORE1 = 1;

    void createTasks() {
        TaskHandle_t task_handle;

        driver->addTask(&task_handle, "RadarReadingTask", 400, radarReading_task, NULL, 2);
        driver->addTask(&task_handle, "MotorTask", stepMotor_task, NULL, 3);
        driver->addTask(&task_handle, "AudioTask", 400, AudioTask, NULL, 2);
        driver->addTask(&task_handle, "UARTReceiveTask", 400, mainTask, NULL, 3);
        driver->addTask(&task_handle, "UARTSendTask", 200, uart_send_task, NULL, 2);
        driver->setTaskCore(task_handle, CORE0);
        driver->addTask(&task_handle, "LedTask", 200, prvLedTask, (void *)25, 1);
        driver->addTask(&task_handle, "DisplayTask", 900, display_task, NULL, 2);
        driver->setTaskCore(task_handle, CORE1);
        driver->startScheduler();
    }
};

}; // End of CONTROLLER namespace.

int main() {
    CONTROLLER::TaskManager taskManager;
    taskManager.enableSystem();
    
    for (;;) {
        ;
    }
    
    return 0;
}
