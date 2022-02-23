#include <Wire.h>
#include <Adafruit_GFX.h>

#include <FFF_Oled.h>
#include <FFF_Graphics.h>

#include <FFF_Settings.h>

#include <pgmspace.h>

Adafruit_SSD1306 oled(OLED_WIDTH_PX, OLED_HEIGHT_PX);

typedef struct
{
    uint8_t lineNum;
    uint8_t textSize;
    char chars[OLED_MAX_CHARS];
} DisplayLine;


DisplayLine tempLine;
DisplayLine diameterLine;

DisplayLine* lineArr[] = {&tempLine, &diameterLine, NULL};

/* Graphics */
extern const uint8_t myFFFLogo[];


void initTextLayout()
{
    tempLine.lineNum = 0;
    tempLine.textSize = 1;

    diameterLine.lineNum = 1;
    diameterLine.textSize = 2;
}


void FFF_Oled_init()
{
    Serial.println("Starting OLED");
    if(!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 allocation failed"));
    }

    /* Show Logo */    
    vTaskDelay(200);
    oled.clearDisplay();
    oled.clearDisplay();
    oled.clearDisplay();
    oled.clearDisplay();
    oled.clearDisplay();
    oled.drawBitmap(0, 0, myFFFLogo, 128, 64, WHITE);
    oled.display();
    vTaskDelay(2000);
    oled.clearDisplay();
    oled.setTextColor(WHITE);
    oled.setTextSize(3);
    oled.setCursor(1,0);
    oled.println("FFF" NL "Device");
    oled.setTextSize(1);
    oled.println("V0.1");
    oled.display();
    Serial.println("Showing Logo");

    initTextLayout();
    /* Show the main menu after startup*/
    vTaskDelay(1000);
    FFF_Oled_clearDisplay();
    FFF_Oled_updateTemperature(DUMMY_VAL_TEMPERATURE);
    FFF_Oled_updateDiameter(DUMMY_VAL_DIAMETER);
    FFF_Oled_updateDisplay();

}


void FFF_Oled_updateTemperature(double tempVal)
{
    snprintf(tempLine.chars, OLED_MAX_CHARS - 1, "%u °C", (uint16_t) tempVal);
    FFF_Oled_updateDisplay();
}


void FFF_Oled_updateDiameter(double diaVal)
{
    snprintf(diameterLine.chars, OLED_MAX_CHARS - 1, "%.2fmm", diaVal);
    FFF_Oled_updateDisplay();
}


void FFF_Oled_updateDisplay()
{
    for (int i = 0; lineArr[i] != NULL; i++)
    {
        oled.setTextSize(lineArr[i]->textSize);
        oled.println(lineArr[i]->chars);
    }

    oled.display();
}


void FFF_Oled_clearDisplay()
{
    oled.clearDisplay();
}


Adafruit_SSD1306* FFF_Oled_getOLED()
{
    return &oled;
}
