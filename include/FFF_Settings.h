#ifndef FFF_SETTINGS_H
#define FFF_SETTINGS_H


#define OLED_ADDR 0x3C
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64


#define EXT_ADC_ADDR 0x48
#define EXT_ADC_TEMP_PIN 0


/* PIN ASSIGNMENTS */
#define HEATER_OUTPUT_PIN   0
#define TX_TO_FPGA_PIN      1
#define SPI_SCK_PIN         2 
#define RX_TO_FPGA_PIN      3
#define OLED_SCL_PIN        4
#define OLED_SDA_PIN        5
#define SPI_CS_PIN_SD       12
#define STEPPER_EN_PIN      13
#define EXTRUDER_STEP_PIN   14
#define PULLER_STEP_PIN     15
#define WINCH_STEP_PIN      16
#define SPI_MISO_PIN        25
#define SPI_MOSI_PIN        26
#define EMERGENCY_BUTTON_PIN 36


/* TASK INTERVALS */
#define OLED_UPDATE_INTERVAL_MS         500
#define UDP_HANDLE_INTERVAL_MS          200
#define SD_LOG_INTERVAL_MS              1000
#define PID_TEMP_INTERVAL_MS            1000
#define PID_DIAMETER_INTERVAL_MS        1000
#define FPGA_CALCULATE_DIAMETER_INTERVAL_MS 1000

/* TASK PRIORITIES */
#define OLED_TASK_PRIO          1
#define SD_LOG_TASK_PRIO        2
#define UDP_TASK_PRIO           3
#define FPGA_CALC_TASK_PRIO     4
#define PID_TEMP_TASK_PRIO      5
#define PID_DIAMETER_TASK_PRIO  6


#endif