#ifndef FFF_OLED_H
#define FFF_OLED_H

#include <Adafruit_SSD1306.h>

void FFF_Oled_init();

void FFF_Oled_updateTemperature(double tempVal);
void FFF_Oled_updateDiameter(double diaVal);
void FFF_Oled_updateExtruderMotSpeed(double motSpeed);
void FFF_Oled_updatePullMotSpeed(double motSpeed);

void FFF_Oled_updateDisplay();

void FFF_Oled_clearDisplay();

Adafruit_SSD1306* FFF_Oled_getOLED();

TaskHandle_t* FFF_Oled_getTaskHandle();

#endif