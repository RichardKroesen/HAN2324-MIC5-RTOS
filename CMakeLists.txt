cmake_minimum_required(VERSION 3.18)
set(PICO_BOARD pico_w)
set(PICO_SDK_FETCH_FROM_GIT ON)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(FetchContent)

FetchContent_Declare(
  FreeRTOS
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel
  GIT_TAG        V11.0.1
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS
)

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

project(MIC5_ASSIGNMENT C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif ()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
)

add_subdirectory(bgt60)

if (PICO_CYW43_SUPPORTED) # set by BOARD=pico-w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping build as support is not available")
    else ()
        add_executable(MIC5_ASSIGNMENT "src/main.c")
        # enable usb output, disable uart output
        pico_enable_stdio_usb(MIC5_ASSIGNMENT 1)
        pico_enable_stdio_uart(MIC5_ASSIGNMENT 0)

        target_include_directories(MIC5_ASSIGNMENT PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/src
            ${CMAKE_CURRENT_LIST_DIR}/bgt60
        )

        # Select the heap port.  values between 1-4 will pick a heap.
        set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)

        # Select the native compile PORT
        set(FREERTOS_PORT "GCC_ARM_CM0" CACHE STRING "" FORCE)
        add_library(freertos_config INTERFACE)
        FetchContent_MakeAvailable(FreeRTOS)

        add_library(lv_driver_hooks "src/lvgl_hooks/lvgl_hooks.c" "src/lvgl_hooks/lv_port_disp.c")
        target_include_directories(lv_driver_hooks PUBLIC "src/lvgl_hooks/" "config/")
        target_link_libraries(lv_driver_hooks pico_stdlib hardware_spi freertos_kernel lvgl::lvgl lv_drivers)

        # Specify path to own LVGL config header
        set(LV_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/lv_conf.h CACHE STRING "" FORCE)
        FetchContent_MakeAvailable(lvgl)
        FetchContent_MakeAvailable(lv_drivers)
        target_include_directories(lv_drivers PUBLIC "config/")

        target_compile_definitions(lv_drivers PUBLIC "LV_CONF_INCLUDE_SIMPLE" "LV_LVGL_H_INCLUDE_SIMPLE")
        target_link_libraries(lv_drivers PUBLIC lv_driver_hooks)

        target_compile_definitions(lvgl PUBLIC "LV_CONF_INCLUDE_SIMPLE")
        target_include_directories(freertos_config INTERFACE "config/")

        file(GLOB_RECURSE SRC_FILES src/ui/*.c)

        add_library(ui ${SRC_FILES})

        target_include_directories(ui PUBLIC "src/ui/")

        target_link_libraries(ui lvgl)

        target_link_libraries(MIC5_ASSIGNMENT pico_cyw43_arch_lwip_poll
            pico_stdlib
            pico_lwip_mbedtls
            pico_mbedtls
            freertos_kernel
            freertos_config
            lvgl::lvgl
            lvgl::examples
            lvgl::demos
            lv_drivers
            lv_driver_hooks
            hardware_spi
            ui
        )

        pico_add_extra_outputs(MIC5_ASSIGNMENT)

    endif ()
endif()