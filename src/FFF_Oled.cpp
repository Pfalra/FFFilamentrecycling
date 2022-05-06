#include <Wire.h>
#include <Adafruit_GFX.h>

#include <FFF_Oled.h>
#include <FFF_Graphics.h>
#include <FFF_Temperature.h>
#include <FFF_Stepper.h>
#include <FFF_Settings.h>
#include <FFF_DiaAnalyzer.h>

#include <pgmspace.h>


TaskHandle_t OledTaskHandle;


Adafruit_SSD1306 oled(OLED_WIDTH_PX, OLED_HEIGHT_PX);

typedef struct
{
    String shortName;
    uint8_t lineNum;
    uint8_t textSize;
    String outputStr;
    String unitStr;
} DisplayLine;


DisplayLine tempLine = 
{
    "T0",                       // shortName
    1,                          // lineNumber
    1,                          // textSize
    "" + DUMMY_VAL_TEMPERATURE, // Dummy
    "degC"                      // unit
};

DisplayLine diameterLine = 
{
    "DIA",
    0,
    1,
    "1.75",
    "mm"
};

DisplayLine extVelLine = 
{
    "SPS",
    2,
    1,
    "" + DUMMY_VAL_EXTMOT_SPEED,
    "mm"
};

DisplayLine pullVelLine = 
{
    "SPS",
    3,
    1,
    "" + DUMMY_VAL_PULLMOT_SPEED,
    "mm"
};

DisplayLine* lineArr[] = {&tempLine, &diameterLine, &extVelLine, &pullVelLine, NULL};

/* Graphics */
extern const uint8_t myFFFLogo[];



void FFF_Oled_init()
{
    Serial.println("Starting OLED");
    while(!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
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
    String tempValStr = String(tempVal, 2);
    tempLine.outputStr = tempValStr;
}


void FFF_Oled_updateDiameter(double diaVal)
{
    String diaValStr = String(diaVal, 2);
    diameterLine.outputStr = diaValStr;
}


void FFF_Oled_updateExtruderMotSpeed(double motSpeed)
{
    String motSpeedStr = String(motSpeed, 2);
    extVelLine.outputStr = motSpeedStr;
}


void FFF_Oled_updatePullMotSpeed(double motSpeed)
{
    String pullMotStr = String(motSpeed, 2);
    pullVelLine.outputStr = pullMotStr;
}


void FFF_Oled_updateDisplay()
{
    oled.clearDisplay();
    oled.setCursor(1,0);
    for (int i = 0; lineArr[i] != NULL; i++)
    {
        String tmpStr = lineArr[i]->shortName + ":" + lineArr[i]->outputStr + " " + lineArr[i]->unitStr;
        oled.setTextSize(lineArr[i]->textSize);
        oled.println(tmpStr);
#if DEBUG_OLED == TRUE
        Serial.print("Display showing: ");
        Serial.println(tmpStr);
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


void TASK_handleOled(void* param)
{
    while(1)
    {
        // Serial.println("OLED fired");
        // Serial.println(FFF_Temp_getTemperature());
        FFF_Oled_updateExtruderMotSpeed(FFF_Stepper_getExtruderStepperSpeed());
        FFF_Oled_updatePullMotSpeed(FFF_Stepper_getPullStepperSpeed());
        FFF_Oled_updateTemperature(FFF_Temp_getTemperature());
        FFF_Oled_updateDiameter(FFF_DiaAn_getDiameter());
        FFF_Oled_updateDisplay();
        vTaskDelay(OLED_REFRESH_INTERVAL_MS);
    }
}


TaskHandle_t* FFF_Oled_getTaskHandle()
{
    return &OledTaskHandle;
}
