#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "HWCDC.h"

HWCDC USBSerial;

// Power management pins
const int powerOutputPin = 41;  // SYS_EN
const int powerInputPin = 40;   // SYS_OUT

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
                                      0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

void setup() {
    USBSerial.begin(115200);

    // Initialize power management pins
    pinMode(powerOutputPin, OUTPUT);
    pinMode(powerInputPin, INPUT);
    
    // Keep power on
    digitalWrite(powerOutputPin, HIGH);
    
    // Initialize display
    gfx->begin();
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, HIGH);
    
    // Draw something to show we're on
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(10, 10);
    gfx->println("Power ON!");
}

void loop() {
    // Check for long press to power off
    static bool buttonPressed = false;
    static unsigned long pressStartTime = 0;
    const unsigned long LONG_PRESS_TIME = 2000; // 2 seconds for long press
    
    int buttonState = digitalRead(powerInputPin);
    
    if (buttonState == LOW) { // Button is pressed
        if (!buttonPressed) {
            buttonPressed = true;
            pressStartTime = millis();
        }
        
        // Check if this is a long press
        if (buttonPressed && (millis() - pressStartTime > LONG_PRESS_TIME)) {
            // Show power off message
            gfx->fillScreen(BLACK);
            gfx->setCursor(10, 10);
            gfx->println("Powering off...");
            delay(500); // Let message be visible briefly
            
            // Power off by setting SYS_EN low
            digitalWrite(powerOutputPin, LOW);
        }
    } else {
        buttonPressed = false;
    }
    
    delay(100); // Small delay to prevent too rapid checking
}