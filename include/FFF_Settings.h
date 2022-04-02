#ifndef FFF_SETTINGS_H
#define FFF_SETTINGS_H

#include <FFF_Types.h>

/*********************************************************/
/* GENERAL SETTINGS */
/*********************************************************/

#define NL "\r\n"
#define FFF_LOG_FILE_FORMAT ".txt"
#define FFF_VERSION "V0.1"
#define SERIAL0_BAUDRATE 250000
#define SERIAL2_BAUDRATE 500000 //FPGA
#define DELIMITER ";"
#define FFF_DEVICE_SUPPLY 3.3

/*********************************************************/
/* TEMPERATURE CALUCLATION METHODS */
/*********************************************************/
#define LUT_TRAVERSE_AND_LINEARIZATION 0
#define STEINHART_HART 1

/*********************************************************/
/* TARGET VALUES */
/*********************************************************/
#define TARGET_DIAMETER 1.75


/*********************************************************/
/* THERMISTORS */
/*********************************************************/
#define DUMMY_THERMISTOR 0
#define NTC_3950 1

#define FFF_THERMISTOR0 NTC_3950 // Sets the thermistor 


/*********************************************************/
/* PWM */
/*********************************************************/
#define PWM_HEATER_FREQUENCY    2500



/*********************************************************/
/* TEMPERATURE COEFFICIENTS  */
/*********************************************************/
#define TEMPERATURE_CALC_METHOD STEINHART_HART_METHOD
#define NORM_TEMP   25.0
#define NORM_RES    100000.0

// The following coefficients were calculated with the NTC 
// calculator at 
#define ALPHA_COEFF -0.0559
#define BETA_COEFF 3950

/*********************************************************/
/* ANALYZER */
/*********************************************************/
#define MEANFILTER_ACTIVE       TRUE
#define MEANFILTER_WEIGHT_COEFF 10
#define DIAMETER_SCALING_COEFF  100
#define MEASUREMENT_LENGTH      2048
#define FIRST_DEAD_PIX          128
#define LAST_DEAD_PIX           60
#define SYNC_LENGTH             4
#define DIA_MEAS_HYST           32

/*********************************************************/
/* OLED */
/*********************************************************/
#define OLED_ADDR 0x3C
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64
#define OLED_MAX_CHARS 24


/*********************************************************/
/* DUMMY VALUES */
/*********************************************************/
#define DUMMY_VAL_TEMPERATURE 333
#define DUMMY_VAL_DIAMETER 1.75
#define DUMMY_VAL_EXTMOT_SPEED 100
#define DUMMY_VAL_PULLMOT_SPEED 300
#define EXTRUDE_RATE_STEPS_PS 42


/*********************************************************/
/* ADC */
/*********************************************************/
#define EXT_ADC_ADDR 0x48
#define EXT_ADC_TEMP_CHANNEL 3
#define MAX_TEMP_DELTA_DEG  25.0


/*********************************************************/
/* PID */
/*********************************************************/
#define KP_PULL_MOT_DEFAULT 1.0
#define KI_PULL_MOT_DEFAULT 1.0
#define KD_PULL_MOT_DEFAULT 1.0


/*********************************************************/
/* UDP */
/*********************************************************/
#define UDP_PORT 666 // Doom intensifies
#define UDP_PBUF_SIZE 255
#define UDP_CONFIRM "OK" NL

#define UDP_APP_START_CMD "START"
#define UDP_APP_STOP_CMD "STOP"
#define UDP_APP_PAUSE_CMD "PAUSE"
#define UDP_APP_LIST_CMD "LIST"
#define UDP_SET_EXTRUDER_SPEED_CMD "EX"
#define UDP_SET_PULLER_SPEED_CMD "PU"
#define UDP_SET_WINCH_SPEED_CMD "WI"

#define UDP_START_CONFIRM "STARTING APP" NL
#define UDP_STOP_CONFIRM "STOPPING APP" NL
#define UDP_PAUSE_CONFIRM "PAUSING APP" NL



/*********************************************************/
/* PIN ASSIGNMENTS AND HARDWARE */
/*********************************************************/
#define THERMISTOR_PULL_UP_VAL 4700.0f
#define STEPPER_MICROSTEPS  16

#define HEATER_OUTPUT_PIN   33
#define RX_TO_FPGA_PIN      16
#define TX_TO_FPGA_PIN      17
#define OLED_SCL_PIN        22
#define OLED_SDA_PIN        21
#define STEPPER_EN_PIN      27
#define EXTRUDER_STEP_PIN   26
#define PULLER_STEP_PIN     25
#define WINCH_STEP_PIN      33
#define SPI_SCK_PIN         14
#define SPI_CS_PIN_SD       15
#define SPI_MISO_PIN        12
#define SPI_MOSI_PIN        13
#define EMERGENCY_BUTTON_PIN 36



/*********************************************************/
/* TASK INTERVAL SETTINGS */
/*********************************************************/
#define UDP_HANDLE_INTERVAL_MS              400
#define LOG_INTERVAL_MS                     500
#define PID_TEMP_INTERVAL_MS                1000
#define PID_DIAMETER_INTERVAL_MS            1000
#define FPGA_CALCULATE_DIAMETER_INTERVAL_MS 1000
#define ADC_SAMPLE_INTERVAL_MS              500


/*********************************************************/
/* TASK PRIORITIES */
/*********************************************************/
#define PID_TEMP_TASK_PRIO      6
#define LOG_TASK_PRIO           2
#define UDP_TASK_PRIO           3
#define FPGA_CALC_TASK_PRIO     4
#define PID_DIAMETER_TASK_PRIO  5
#define ADC_TASK_PRIO           5


/*********************************************************/
/* DEBUG SWITCHES */
/*********************************************************/
#define DEBUG_LUT_HANDLING  FALSE
#define DEBUG_OLED          FALSE
#define DEBUG_ADC           FALSE
#define DEBUG_STEPPER       FALSE

#endif