#include "task_manager.hpp"

int main() {
    CONTROLLER::TaskManager taskManager{};
    taskManager.enableSystem();
    
    for (;;);
    return 0;
}
