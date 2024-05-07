#ifndef LVGL_HOOKS_H
#define LVGL_HOOKS_H

#include <FreeRTOS.h>
#include "board_defines.h"

void display_cmd_data(const uint8_t val);

void display_rst(const uint8_t val);

void display_spi_cs(const uint8_t val);

void display_spi_wr_byte(const uint8_t data);

void display_spi_wr_array(char *src, const size_t n);

void display_spi_mode(const uint8_t bits, const uint8_t mode);

#endif