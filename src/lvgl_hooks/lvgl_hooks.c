#include "lvgl_hooks.h"
#include <hardware/gpio.h>
#include <hardware/spi.h>


void display_cmd_data(const uint8_t val) {
    gpio_put(DISPLAY_CMD, val);
}

void display_rst(const uint8_t val) {
    gpio_put(DISPLAY_RST, val);
}

void display_spi_cs(const uint8_t val) {
    gpio_put(DISPLAY_CS, val);
}

void display_spi_wr_byte(const uint8_t data) {
    spi_write_blocking(SPI_INST, &data, 1);
}

void display_spi_wr_array(char *src, const size_t n) {
    spi_write_blocking(SPI_INST, (uint8_t *)src, n);
}