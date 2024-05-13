#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <stdint.h>
#include "pico/stdlib.h"

namespace CONTROLLER {

class CoreDriver {
public:
    CoreDriver() = default;
    ~CoreDriver() = default;

    static inline void initialize() {
        stdio_init_all();
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 1);
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

} // namespace CONTROLLER
#endif // CONTROLLER_HPP 