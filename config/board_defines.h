#ifndef BOARD_DEFINES_H
#define BOARD_DEFINES_H

/* DISPLAY CONFIGURATION: */
#define SPI_INST            spi0
#define SPI_BAUD_RATE_DISPLAY 20000000

#define DISPLAY_MISO        4
#define DISPLAY_CS          7
#define DISPLAY_SCK         6
#define DISPLAY_MOSI        3
#define DISPLAY_RST         1
#define DISPLAY_CMD         5

#define SPI_BAUD_RATE_RADAR 20000000
#define SPI_INST_RADAR      spi0
#define SPI_SCK_RADAR       6  // SPI Clock
#define SPI_MOSI_RADAR      3  // Master Out Slave In (MOSI)
#define SPI_MISO_RADAR      4  // Master In Slave Out (MISO)
#define SPI_CS_RADAR        2  // Chip Select (CS)
#define SPI_RS_RADAR        0  // Reset Line

#endif