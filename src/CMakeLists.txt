add_executable(${NAME}
    main.cpp
)

add_library(lv_driver_hooks 
    "lvgl_hooks/lvgl_hooks.c" 
    "lvgl_hooks/lv_port_disp.c"
)

target_include_directories(lv_driver_hooks PUBLIC 
    "lvgl_hooks/" 
    "../config/"
    ${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel/include
)

target_link_libraries(lv_driver_hooks 
    FreeRTOS-Kernel-Heap4
    pico_stdlib 
    hardware_spi 
    lvgl::lvgl 
    lv_drivers
)

set(LV_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/lv_conf.h CACHE STRING "" FORCE)

target_include_directories(lv_drivers PUBLIC "../config/")

target_compile_definitions(lv_drivers PUBLIC 
    "LV_CONF_INCLUDE_SIMPLE" 
    "LV_LVGL_H_INCLUDE_SIMPLE"
)

target_link_libraries(lv_drivers PUBLIC lv_driver_hooks)
target_compile_definitions(lvgl PUBLIC "LV_CONF_INCLUDE_SIMPLE")
file(GLOB_RECURSE SRC_FILES ui/*.c)
add_library(ui ${SRC_FILES})

target_include_directories(ui PUBLIC 
    "ui/"
    ${CMAKE_CURRENT_LIST_DIR}/../lib/lvgl
)

target_link_libraries(${NAME} 
    pico_cyw43_arch_lwip_poll
    pico_stdlib
    pico_lwip_mbedtls
    pico_mbedtls

    FreeRTOS-Kernel-Heap4
    
    BGT60LTR11XXX
    STEPMOTOR_DRIVER
    SPEAKER_CONTROLLER
    ASYNC_UART
    audio_sample

    hardware_spi
    lvgl::lvgl
    lvgl::examples
    lvgl::demos
    lv_drivers
    lv_driver_hooks
    ui
)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel/include
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/bgt60
    ${CMAKE_CURRENT_LIST_DIR}/motor_driver
    ${CMAKE_CURRENT_LIST_DIR}/speaker
    ${CMAKE_CURRENT_LIST_DIR}/async_uart

    ${CMAKE_CURRENT_LIST_DIR}/../lib/lvgl
    ${CMAKE_CURRENT_LIST_DIR}/../lib/lv_drivers
    ${CMAKE_CURRENT_LIST_DIR}/../config
)
     
target_compile_definitions(${NAME} PRIVATE
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

pico_add_extra_outputs(${NAME})
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)