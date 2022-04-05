/**************************************************************************************
* \author Raphael Pfaller (raphael.pfaller.dev@googlemail.com)
* \date 02.04.2022
* \note SD functionality is foreseen but was taken out due to technical and timeline 
* problems 
**************************************************************************************/

/* Framework and external libs */
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <WiFiUDP.h>
#include <math.h>

/* FFF includes */
#include <FFF_Types.h>
#include <FFF_Rtos.h>
#include <FFF_Settings.h>
#include <FFF_Oled.h>
#include <FFF_Adc.h>
#include <FFF_Stepper.h>
#include <FFF_Uart.h>
#include <FFF_Pid.h>
#include <FFF_WiFi.h>

const char paramArr[] = "TEMP" DELIMITER \
              "DIAMETER" DELIMITER \
              "PULLMOT_SPD" DELIMITER \
              "EXTMOT_SPD" DELIMITER;

void setup()
{  
  FFF_Uart_init();
  Serial.println();
  Serial.println("FFF Device starting up...");

  /* Initialize WiFi */
  FFF_WiFi_init();
  
  /* Initialize OLED */
  FFF_Oled_init();
  delay(200);

  /* Initialize ADC */
  Serial.println("\r\nI>Initializing ADC");
  FFF_Adc_init();

  /* Initialize Steppers */
  Serial.println(NL "I>Initializing Steppers");
  FFF_Stepper_init();

  /* Initialize the PID control */
  Serial.println(NL "I>Initializing PID control");
  FFF_Pid_init();


  /* Create tasks for initialization */
  Serial.println(NL "Starting InitTasks");
  FFF_Rtos_StartOS();
}


void loop()
{
  TASK_mainControlApp();
}
