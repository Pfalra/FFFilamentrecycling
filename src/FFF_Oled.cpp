#include <Wire.h>
#include <Adafruit_GFX.h>

#include <FFF_Oled.h>
#include <FFF_Graphics.h>

#include <FFF_Settings.h>

#include <pgmspace.h>

Adafruit_SSD1306 oled(OLED_WIDTH_PX, OLED_HEIGHT_PX);

/* Graphics */
extern const uint8_t myFFFLogo[];

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

    /* Show the main menu after startup*/
    vTaskDelay(1000);
}

Adafruit_SSD1306* FFF_Oled_getOLED()
{
    return &oled;
}
