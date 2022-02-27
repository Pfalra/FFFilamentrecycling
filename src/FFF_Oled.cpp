#include <Wire.h>
#include <Adafruit_GFX.h>

#include <FFF_Oled.h>
#include <FFF_Graphics.h>

#include <FFF_Settings.h>

#include <pgmspace.h>

Adafruit_SSD1306 oled(OLED_WIDTH_PX, OLED_HEIGHT_PX);

typedef struct
{
    char shortName[OLED_MAX_CHARS];
    uint8_t lineNum;
    uint8_t textSize;
    char chars[OLED_MAX_CHARS];
} DisplayLine;


DisplayLine tempLine;
DisplayLine diameterLine;
DisplayLine extVelLine;
DisplayLine pullVelLine;

DisplayLine* lineArr[] = {&tempLine, &diameterLine, &extVelLine, &pullVelLine, NULL};

/* Graphics */
extern const uint8_t myFFFLogo[];


// FIXME: This cannot be the most elegant way of handling it 
void initTextLayout()
{
    tempLine.lineNum = 0;
    tempLine.textSize = 1;
    tempLine.chars[OLED_MAX_CHARS-1] = '\0';

    diameterLine.lineNum = 1;
    diameterLine.textSize = 2;
    diameterLine.chars[OLED_MAX_CHARS-1] = '\0';

    extVelLine.lineNum = 2;
    extVelLine.textSize = 1;
    extVelLine.chars[OLED_MAX_CHARS-1] = '\0';

    pullVelLine.lineNum = 3;
    pullVelLine.textSize = 1;
    pullVelLine.chars[OLED_MAX_CHARS-1] = '\0';
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
    oled.println(FFF_VERSION);
    oled.display();
    Serial.println("Showing Logo");

    initTextLayout();
    /* Show the main menu after startup*/
    vTaskDelay(1000);
    FFF_Oled_clearDisplay();
    FFF_Oled_updateTemperature(DUMMY_VAL_TEMPERATURE);
    FFF_Oled_updateDiameter(DUMMY_VAL_DIAMETER);
    FFF_Oled_updateExtruderMotSpeed(DUMMY_VAL_EXTMOT_SPEED);
    FFF_Oled_updatePullMotSpeed(DUMMY_VAL_PULLMOT_SPEED);
    FFF_Oled_updateDisplay();

}


void FFF_Oled_updateTemperature(double tempVal)
{
    snprintf(tempLine.chars, OLED_MAX_CHARS - 1, "%.1f  degC", tempVal);
    // Serial.println(tempLine.chars);
    FFF_Oled_updateDisplay();
}


void FFF_Oled_updateDiameter(double diaVal)
{
    snprintf(diameterLine.chars, OLED_MAX_CHARS - 1, "%.2fmm", diaVal);
    // Serial.println(tempLine.chars);
    FFF_Oled_updateDisplay();
}


void FFF_Oled_updateExtruderMotSpeed(double motSpeed)
{
    snprintf(extVelLine.chars, OLED_MAX_CHARS - 1, "%u steps/s", (uint16_t) motSpeed);
    // Serial.println(extVelLine.chars);
    FFF_Oled_updateDisplay();
}

void FFF_Oled_updatePullMotSpeed(double motSpeed)
{
    snprintf(pullVelLine.chars, OLED_MAX_CHARS - 1, "%u steps/s", (uint16_t) motSpeed);
    // Serial.println(pullVelLine.chars);
    FFF_Oled_updateDisplay();
}


void FFF_Oled_updateDisplay()
{
    oled.clearDisplay();
    oled.setCursor(1,0);
    for (int i = 0; lineArr[i] != NULL; i++)
    {
        oled.setTextSize(lineArr[i]->textSize);
        oled.println(lineArr[i]->chars);
#if DEBUG_OLED == TRUE
        Serial.print("Display showing: ");
        Serial.println(lineArr[i]->chars);
#endif
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
