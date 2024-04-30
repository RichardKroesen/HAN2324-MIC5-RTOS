#ifndef BOARD_DEFINES_H
#define BOARD_DEFINES_H

/* DISPLAY PINS: */
#define SPI_INST            spi0
#define SPI_BAUD_RATE_DISPLAY 20000000

#define DISPLAY_MISO        4
#define DISPLAY_CS          7
#define DISPLAY_SCK         6
#define DISPLAY_MOSI        3
#define DISPLAY_RST         1
#define DISPLAY_CMD         5

/* MMWAVE RADAR PINS */
#define SPI_BAUD_RATE_RADAR 20000000
#define SPI_INST_RADAR      spi0
#define SPI_SCK_RADAR       6  // SPI Clock
#define SPI_MOSI_RADAR      3  // Master Out Slave In (MOSI)
#define SPI_MISO_RADAR      4  // Master In Slave Out (MISO)
#define SPI_CS_RADAR        2  // Chip Select (CS)
#define SPI_RS_RADAR        0  // Reset Line

/* STEP MOTOR PINS */
#define STEP_MOTOR_PIN      8
#define DIR_MOTOR_PIN       9
#define EN_MOTOR_PIN        10

/* AUDIO CONTROL PINS */
#define AUDIO_PIN           11

#endif