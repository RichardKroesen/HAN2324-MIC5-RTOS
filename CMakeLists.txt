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

        add_subdirectory(bgt60)
        add_subdirectory(motor_driver)
        add_subdirectory(speaker)
        add_subdirectory(samples)
        add_subdirectory(async_uart)

        add_subdirectory(src)
    endif ()
endif()


#     # Specify FreeRTOS configuration path
#     set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/config" CACHE STRING "Local Config")
#     include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}")
#     set(FREERTOS_PORT GCC_RP2040 CACHE STRING \"\")


#     add_library(freertos_config INTERFACE)
#     target_include_directories(freertos_config INTERFACE FREERTOS_CONFIG_FILE_DIRECTORY)
#     include(${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

#     # Make FreeRTOS, LVGL, and other external dependencies available
#     FetchContent_MakeAvailable(FreeRTOS)
#     FetchContent_MakeAvailable(lvgl)
#     FetchContent_MakeAvailable(lv_drivers)

#     add_library(lv_driver_hooks "src/lvgl_hooks/lvgl_hooks.c" "src/lvgl_hooks/lv_port_disp.c")
#     target_include_directories(lv_driver_hooks PUBLIC "src/lvgl_hooks/" "config/")
#     target_link_libraries(lv_driver_hooks pico_stdlib hardware_spi freertos_kernel lvgl::lvgl lv_drivers)

#     set(LV_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/lv_conf.h CACHE STRING "" FORCE)
#     target_include_directories(lv_drivers PUBLIC "config/")
#     target_compile_definitions(lv_drivers PUBLIC "LV_CONF_INCLUDE_SIMPLE" "LV_LVGL_H_INCLUDE_SIMPLE")
#     target_link_libraries(lv_drivers PUBLIC lv_driver_hooks)
#     target_compile_definitions(lvgl PUBLIC "LV_CONF_INCLUDE_SIMPLE")

#     file(GLOB_RECURSE SRC_FILES src/ui/*.c)
#     add_library(ui ${SRC_FILES})
#     target_include_directories(ui PUBLIC "src/ui/")
#     target_link_libraries(ui lvgl)

#     target_link_libraries(MIC5_ASSIGNMENT pico_cyw43_arch_lwip_poll
#       BGT60LTR11XXX
#       STEPMOTOR_DRIVER
#       SPEAKER_CONTROLLER
#       ASYNC_UART
#       audio_sample
#       pico_stdlib
#       pico_lwip_mbedtls
#       pico_mbedtls
#       freertos_kernel
#       freertos_config
#       lvgl::lvgl
#       lvgl::examples
#       lvgl::demos
#       lv_drivers
#       lv_driver_hooks
#       hardware_spi
#       ui
#     )
