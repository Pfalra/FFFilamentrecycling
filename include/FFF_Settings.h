#ifndef FFF_SETTINGS_H
#define FFF_SETTINGS_H

#define NL "\r\n"


#define SERIAL_BAUDRATE 250000


/* ANALYZER */
#define MEANFILTER_ACTIVE       TRUE
#define MEANFILTER_WEIGHT_KOEFF 10

/* OLED */
#define OLED_ADDR 0x3C
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64

/* ADC */
#define EXT_ADC_ADDR 0x48
#define EXT_ADC_TEMP_PIN 0


/* UDP */
#define UDP_PORT 666 // Doom intensifies
#define UDP_PBUF_SIZE 255
#define UDP_CONFIRM "OK" NL


/* PIN ASSIGNMENTS */
#define HEATER_OUTPUT_PIN   33

#define TX_TO_FPGA_PIN      16
#define RX_TO_FPGA_PIN      17

#define OLED_SCL_PIN        22
#define OLED_SDA_PIN        21

#define STEPPER_EN_PIN      27
#define EXTRUDER_STEP_PIN   26
#define PULLER_STEP_PIN     25
#define WINCH_STEP_PIN      33

#define EMERGENCY_BUTTON_PIN 36

#define SPI_SCK_PIN         14
#define SPI_CS_PIN_SD       15
#define SPI_MISO_PIN        12
#define SPI_MOSI_PIN        13


/* TASK INTERVALS */
#define OLED_UPDATE_INTERVAL_MS         500
#define UDP_HANDLE_INTERVAL_MS          400
#define SD_LOG_INTERVAL_MS              1000
#define PID_TEMP_INTERVAL_MS            1000
#define PID_DIAMETER_INTERVAL_MS        1000
#define FPGA_CALCULATE_DIAMETER_INTERVAL_MS 1000
#define ADC_SAMPLE_INTERVAL_MS          200

/* TASK PRIORITIES */
#define OLED_TASK_PRIO          1
#define SD_LOG_TASK_PRIO        2
#define UDP_TASK_PRIO           3
#define FPGA_CALC_TASK_PRIO     4
#define PID_TEMP_TASK_PRIO      5
#define PID_DIAMETER_TASK_PRIO  6


#endif