cmake_minimum_required(VERSION 3.18)
set(PICO_BOARD pico_w)
set(PICO_SDK_FETCH_FROM_GIT ON)
set(NAME MIC5_ASSIGNMENT)

include(FetchContent)
include(pico_sdk_import.cmake)

FetchContent_Declare(
  lvgl
  GIT_REPOSITORY https://github.com/lvgl/lvgl
  GIT_TAG        release/v8.3
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/lvgl
)

FetchContent_Declare(
  lv_drivers
  GIT_REPOSITORY https://github.com/HCL-Hbot/lv_drivers
  GIT_TAG release/v8.3
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/lv_drivers
)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

#include Libraries
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel" CACHE STRING "FreeRTOS Kernel Path")
include(FreeRTOS_Kernel_import.cmake)

add_compile_options(-Wall -Wno-format -Wno-unused-function -Wno-maybe-uninitialized)

add_compile_definitions(
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

# Link to libraries and set target executable
if (PICO_CYW43_SUPPORTED)
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping build as support is not available")
    else()
        FetchContent_MakeAvailable(lvgl)
        FetchContent_MakeAvailable(lv_drivers)

        add_subdirectory(app)
        add_subdirectory(bgt60)
        add_subdirectory(motor_driver)
        add_subdirectory(speaker)
        add_subdirectory(samples)
        add_subdirectory(async_uart)

        add_subdirectory(src)
    endif ()
endif()