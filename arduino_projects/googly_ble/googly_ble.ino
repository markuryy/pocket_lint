#include <lvgl.h>                          // LVGL graphics library for eyes mode
#include "Arduino_GFX_Library.h"           // Display driver (for our Waveshare ESP32‐S3 dev board)
#include "pin_config.h"                    // Board-specific pin definitions
#include "lv_conf.h"                       // LVGL configuration
#include <Wire.h>
#include "SensorQMI8658.hpp"               // IMU sensor driver
#include "HWCDC.h"                         // USB serial (debug) interface
#include <Preferences.h>
#include <math.h>

// --- BLE libraries for image transfer ---
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Global Definitions ----------------
// Power and button pins
const int powerOutputPin = 41;             // SYS_EN pin for power control
const int powerInputPin  = 40;             // Button input pin

// Timing constants
const unsigned long LONG_PRESS_TIME = 3000;    // 3 sec for power off
const unsigned long MEDIUM_PRESS_TIME = 1500;  // 1.5 sec for calibration
const uint32_t BLE_MODE_TIMEOUT       = 30000;   // 30 sec inactivity auto–revert in BLE mode

// Smoothing factor for pupil motion
const float SMOOTHING_FACTOR = 0.6;

// ---------------- Global Objects ----------------
HWCDC USBSerial;
Preferences preferences;

// Sensor objects for accelerometer and gyroscope data—
SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;

// ---------------- LVGL "Googly Eyes" Globals ----------------
// LVGL objects for the eyes and pupils (created in eyes mode)
lv_obj_t *leftEye;
lv_obj_t *rightEye;
lv_obj_t *leftPupil;
lv_obj_t *rightPupil;

// ---------------- Display Setup ----------------
// Create a data bus using SPI and a display instance (we use the Arduino_GFX library)
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, 
                                         LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

// LVGL display buffer and flush callback
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  #if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #endif
  lv_disp_flush_ready(disp);
}

// LVGL tick increment callback – called periodically via an esp_timer.
void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(2);
}

// ---------------- Calibration Data & State ----------------
// The calibration routine uses four reference orientations.
enum CalibrationOrientation {
  CAL_ORIENT_BOTTOM = 0,
  CAL_ORIENT_LEFT,
  CAL_ORIENT_TOP,
  CAL_ORIENT_BACK,
  CAL_ORIENT_COUNT  // Total orientations
};

enum CalStepState {
  CAL_PROMPT,    // Waiting for user to set orientation and press button
  CAL_COUNTDOWN, // Count down after button press before sampling
  CAL_SAMPLING,  // Collecting samples
  CAL_DONE       // Calibration complete
};

struct CalibrationData {
  float bottom[3];   // BOTTOM reference (resting on table)
  float left[3];     // LEFT
  float top[3];      // TOP (upside–down)
  float back[3];     // BACK
  bool isValid;
  float sensitivityX;  // horizontal sensitivity
  float sensitivityY;  // vertical sensitivity
};

CalibrationData calibrationData;
CalibrationOrientation currentOrientation = CAL_ORIENT_BOTTOM;
CalStepState currentCalStep = CAL_PROMPT;
unsigned long calStepStartTime = 0;
const unsigned long COUNTDOWN_DURATION = 3000;   // 3 seconds
const int CALIBRATION_SAMPLES = 50;
int calSampleCount = 0;
float calSampleSum[3] = {0, 0, 0};
bool calButtonTriggered = false;   // set when user presses button during calibration prompt
bool debugMode = false;            // if true, show calibration debug info

// ---------------- Operational Mode ----------------
// We want to switch between "eyed" (accelerometer–driven) and BLE (image transfer) modes.
enum OperationMode { OPERATION_EYES, OPERATION_BLE };
OperationMode currentMode = OPERATION_EYES;

// ---------------- BLE Global Variables ----------------
// BLE UART Service – use the standard UUIDs for a UART service.
#define UART_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;

// ---------------- BLE Image Transfer State ----------------
// Structure storing details about the incoming image transfer.
struct ImageState {
  uint16_t width;
  uint16_t height;
  uint8_t colorBit;       // 24 for RGB888 or 16 for RGB565
  uint32_t totalPixels;   // total pixels expected
  uint32_t receivedPixels;// pixels received so far
  uint16_t startX;        // X offset for centering the image
  uint16_t startY;        // Y offset for centering
  bool isReceiving;       // flag if transfer is active
  uint8_t currentGroup;   // group packet counter (for ack)
  bool groupPackets[50];  // group packet status (placeholder for future enhancements)
  bool imageDisplayed;    // flag to indicate we're displaying a static image
} imgState;

// Buffer for one line of pixel data (allocated when image starts)
uint16_t* lineBuffer = NULL;
uint32_t rxStartTime = 0;
uint32_t lastBLEActivity = 0;   // updated on every BLE data write

// ---------------- Display Message Helpers ----------------
// In eyes mode we use LVGL; in BLE mode we use direct gfx calls.
// Show an LVGL message (centered). (Used during calibration and eyes mode.)
void showLVGLMessage(const char* message) {
  lv_obj_clean(lv_scr_act());
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, message);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_timer_handler();
}

// Show a message using direct gfx drawing (used in BLE mode).
void showBLEMessage(const char* message, uint16_t color = 0xFFFF) {
  gfx->setTextColor(color);
  gfx->setCursor(0, gfx->height() - 20);
  // clear previous message
  gfx->fillRect(0, gfx->height() - 20, gfx->width(), 20, 0);
  gfx->println(message);
  USBSerial.println(message);
}

// ---------------- "Googly Eyes" UI Functions ----------------

// Create the two eyes and pupils using LVGL objects.
void createEyes() {
  lv_obj_clean(lv_scr_act());
  
  // Left eye background
  leftEye = lv_obj_create(lv_scr_act());
  lv_obj_set_size(leftEye, 80, 80);
  lv_obj_align(leftEye, LV_ALIGN_CENTER, -50, 0);
  lv_obj_set_style_bg_color(leftEye, lv_color_white(), 0);
  lv_obj_set_style_radius(leftEye, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_clip_corner(leftEye, true, 0);
  lv_obj_set_scrollbar_mode(leftEye, LV_SCROLLBAR_MODE_OFF);

  // Left pupil (black circle)
  leftPupil = lv_obj_create(leftEye);
  lv_obj_set_size(leftPupil, 30, 30);
  lv_obj_align(leftPupil, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(leftPupil, lv_color_black(), 0);
  lv_obj_set_style_radius(leftPupil, LV_RADIUS_CIRCLE, 0);

  // Right eye background
  rightEye = lv_obj_create(lv_scr_act());
  lv_obj_set_size(rightEye, 80, 80);
  lv_obj_align(rightEye, LV_ALIGN_CENTER, 50, 0);
  lv_obj_set_style_bg_color(rightEye, lv_color_white(), 0);
  lv_obj_set_style_radius(rightEye, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_clip_corner(rightEye, true, 0);
  lv_obj_set_scrollbar_mode(rightEye, LV_SCROLLBAR_MODE_OFF);

  // Right pupil
  rightPupil = lv_obj_create(rightEye);
  lv_obj_set_size(rightPupil, 30, 30);
  lv_obj_align(rightPupil, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(rightPupil, lv_color_black(), 0);
  lv_obj_set_style_radius(rightPupil, LV_RADIUS_CIRCLE, 0);
  
  lv_timer_handler();
}

// Update pupil positions based on smoothed accelerometer data.
// The function uses the calibration data (if valid) to map acc.x and acc.y differences.
void updateEyes() {
  if (!calibrationData.isValid || debugMode)
    return;
  
  if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
    // Vertical mapping: from acc.x deviation (with scale+offset)
    float deltaX = acc.x - calibrationData.bottom[0]; 
    float computedY = deltaX * 200;  // Scale factor chosen to map full range to ~50px movement
    float finalY = computedY + 25;     // shift so that baseline is central

    // Horizontal mapping: from acc.y
    float deltaY = acc.y - calibrationData.bottom[1];
    float computedX = -deltaY * (25.0 / calibrationData.sensitivityX);
    
    // Constrain to a circle of radius 25
    const int maxOffset = 25;
    float r = sqrt(computedX * computedX + finalY * finalY);
    if (r > maxOffset) {
      computedX *= (maxOffset / r);
      finalY   *= (maxOffset / r);
    }
    
    // Apply simple smoothing (low–pass filter) to simulate inertia.
    static float currentX = 0, currentY = 0;
    currentX = currentX + (computedX - currentX) * SMOOTHING_FACTOR;
    currentY = currentY + (finalY - currentY) * SMOOTHING_FACTOR;
    
    lv_obj_align(leftPupil, LV_ALIGN_CENTER, (int)currentX, (int)currentY);
    lv_obj_align(rightPupil, LV_ALIGN_CENTER, (int)currentX, (int)currentY);
    lv_timer_handler();
  }
}

// Show calibration debug data on LVGL.
void showCalibrationDebug() {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "Calibration Data:\n"
           "BOTTOM: %.2f, %.2f, %.2f\n"
           "LEFT:   %.2f, %.2f, %.2f\n"
           "TOP:    %.2f, %.2f, %.2f\n"
           "BACK:   %.2f, %.2f, %.2f\n"
           "SensX: %.2f\nSensY: %.2f\n\n"
           "Press button to continue...",
           calibrationData.bottom[0], calibrationData.bottom[1], calibrationData.bottom[2],
           calibrationData.left[0],   calibrationData.left[1],   calibrationData.left[2],
           calibrationData.top[0],    calibrationData.top[1],    calibrationData.top[2],
           calibrationData.back[0],   calibrationData.back[1],   calibrationData.back[2],
           calibrationData.sensitivityX, calibrationData.sensitivityY);
  showLVGLMessage(buf);
  USBSerial.println(buf);
}

// Begin calibration: reset all calibration data and start with BOTTOM orientation.
void startCalibration() {
  // Reset all calibration-related state
  memset(&calibrationData, 0, sizeof(calibrationData));  // Clear all calibration data
  calibrationData.isValid = false;
  currentMode = OPERATION_EYES;
  debugMode = false;
  currentOrientation = CAL_ORIENT_BOTTOM;
  currentCalStep = CAL_PROMPT;
  calSampleCount = 0;
  memset(calSampleSum, 0, sizeof(calSampleSum));
  calStepStartTime = millis();
  calButtonTriggered = false;
  
  // Clear display and show initial calibration message
  lv_obj_clean(lv_scr_act());
  showLVGLMessage("Calibration:\nPlace me on my BOTTOM side\nand press the button.");
  USBSerial.println("Starting calibration: BOTTOM");
}

// Handle calibration state machine (PROMPT, COUNTDOWN, SAMPLING)
void handleCalibration() {
  if (!qmi.getDataReady() || !qmi.getAccelerometer(acc.x, acc.y, acc.z))
    return;
    
  char msg[128];
  unsigned long now = millis();
  
  switch (currentCalStep) {
    case CAL_PROMPT:
      if (currentOrientation == CAL_ORIENT_BOTTOM)
        showLVGLMessage("Calibration:\nPlace me on my BOTTOM side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_LEFT)
        showLVGLMessage("Calibration:\nPlace me on my LEFT side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_TOP)
        showLVGLMessage("Calibration:\nPlace me on my TOP side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_BACK)
        showLVGLMessage("Calibration:\nPlace me on my BACK side\nand press the button.");
      
      if (calButtonTriggered) {
        calButtonTriggered = false;
        currentCalStep = CAL_COUNTDOWN;
        calStepStartTime = now;
      }
      break;
      
    case CAL_COUNTDOWN: {
      unsigned long elapsed = now - calStepStartTime;
      int secondsLeft = (int)((COUNTDOWN_DURATION - elapsed) / 1000) + 1;
      snprintf(msg, sizeof(msg), "Hold still...\nSampling in %d", secondsLeft);
      showLVGLMessage(msg);
      if (elapsed >= COUNTDOWN_DURATION) {
        currentCalStep = CAL_SAMPLING;
        calSampleCount = 0;
        memset(calSampleSum, 0, sizeof(calSampleSum));
      }
      break;
    }
      
    case CAL_SAMPLING:
      calSampleSum[0] += acc.x;
      calSampleSum[1] += acc.y;
      calSampleSum[2] += acc.z;
      calSampleCount++;
      snprintf(msg, sizeof(msg), "Sampling %s...\n%d%%",
               (currentOrientation == CAL_ORIENT_BOTTOM) ? "BOTTOM" :
               (currentOrientation == CAL_ORIENT_LEFT)   ? "LEFT" :
               (currentOrientation == CAL_ORIENT_TOP)    ? "TOP" : "BACK",
               (calSampleCount * 100) / CALIBRATION_SAMPLES);
      showLVGLMessage(msg);
      if (calSampleCount >= CALIBRATION_SAMPLES) {
        float avg[3] = {
          calSampleSum[0] / CALIBRATION_SAMPLES,
          calSampleSum[1] / CALIBRATION_SAMPLES,
          calSampleSum[2] / CALIBRATION_SAMPLES
        };
        if (currentOrientation == CAL_ORIENT_BOTTOM) {
          memcpy(calibrationData.bottom, avg, sizeof(avg));
          USBSerial.printf("BOTTOM calibrated: %.2f, %.2f, %.2f\n", avg[0], avg[1], avg[2]);
        } else if (currentOrientation == CAL_ORIENT_LEFT) {
          memcpy(calibrationData.left, avg, sizeof(avg));
          USBSerial.printf("LEFT calibrated: %.2f, %.2f, %.2f\n", avg[0], avg[1], avg[2]);
        } else if (currentOrientation == CAL_ORIENT_TOP) {
          memcpy(calibrationData.top, avg, sizeof(avg));
          USBSerial.printf("TOP calibrated: %.2f, %.2f, %.2f\n", avg[0], avg[1], avg[2]);
        } else if (currentOrientation == CAL_ORIENT_BACK) {
          memcpy(calibrationData.back, avg, sizeof(avg));
          USBSerial.printf("BACK calibrated: %.2f, %.2f, %.2f\n", avg[0], avg[1], avg[2]);
        }
        
        if (currentOrientation < CAL_ORIENT_COUNT - 1) {
          currentOrientation = (CalibrationOrientation)(currentOrientation + 1);
          currentCalStep = CAL_PROMPT;
          calStepStartTime = now;
        } else {
          // Compute sensitivities based on the differences.
          calibrationData.sensitivityX = fabs(calibrationData.left[1] - calibrationData.bottom[1]);
          calibrationData.sensitivityY = fabs(calibrationData.bottom[0] - calibrationData.top[0]);
          calibrationData.isValid = true;
          currentCalStep = CAL_DONE;
          debugMode = true; // Show calibration data until user confirms
          showLVGLMessage("Calibration Complete!\nPress button to continue.");
          USBSerial.println("Calibration complete!");
          // Save calibration to nonvolatile storage
          preferences.begin("googly", false);
          preferences.putBytes("cal_data", &calibrationData, sizeof(CalibrationData));
          preferences.end();
          USBSerial.println("Calibration saved to NVS.");
          showCalibrationDebug();
        }
      }
      break;
      
    case CAL_DONE:
      // Nothing to do
      break;
  }
}

// Load calibration from nonvolatile storage.
void loadCalibration() {
  preferences.begin("googly", false);
  size_t readBytes = preferences.getBytes("cal_data", &calibrationData, sizeof(CalibrationData));
  preferences.end();
  
  if (readBytes == sizeof(CalibrationData) && calibrationData.isValid) {
    USBSerial.println("Calibration loaded from NVS.");
  } else {
    calibrationData.isValid = false;
    USBSerial.println("No valid calibration found.");
  }
}

// ---------------- BLE Image Transfer Helper Functions ----------------

// Draw a horizontal line of pixels on the display.
void drawPixelLine(int16_t x, int16_t y, uint16_t* pixels, int16_t w) {
  for (int16_t i = 0; i < w; i++) {
    gfx->drawPixel(x + i, y, pixels[i]);
  }
}

// Called when a new image begins. The header contains a marker '!I', the color depth,
// and the image width and height (each in little–endian order). Also center the image.
void startNewImage(uint16_t width, uint16_t height, uint8_t colorBit) {
  if (lineBuffer) {
    free(lineBuffer);
    lineBuffer = NULL;
  }
  
  // Allocate a buffer for one line of pixels.
  lineBuffer = (uint16_t*)malloc(width * sizeof(uint16_t));
  if (!lineBuffer) {
    showBLEMessage("Error: Out of memory");
    return;
  }
  
  imgState.width = width;
  imgState.height = height;
  imgState.colorBit = colorBit;
  imgState.startX = (LCD_WIDTH - width) / 2;
  imgState.startY = (LCD_HEIGHT - height) / 2;
  imgState.totalPixels = width * height;
  imgState.receivedPixels = 0;
  imgState.isReceiving = true;
  imgState.currentGroup = 0;
  memset(imgState.groupPackets, 0, sizeof(imgState.groupPackets));
  
  // Clear display and show initial transfer message
  gfx->fillScreen(0);
  showBLEMessage("BLE Mode:\nStarting transfer...");
  
  USBSerial.printf("New image: %dx%d, %d-bit\n", width, height, colorBit);
}

// Process incoming pixel data. Data is either 3 bytes per pixel (for RGB888)
// or 2 bytes per pixel (for RGB565). The function buffers one line and draws it when full.
void processPixelData(const uint8_t* data, size_t len) {
  if (!imgState.isReceiving || !lineBuffer) return;
  
  uint16_t x = imgState.receivedPixels % imgState.width;
  uint16_t y = imgState.receivedPixels / imgState.width;
  
  while (len >= ((imgState.colorBit == 24) ? 3 : 2) && y < imgState.height) {
    uint16_t pixel;
    if (imgState.colorBit == 24) {
      // Convert 24-bit RGB888 to 16-bit RGB565 
      pixel = ((data[0] & 0xF8) << 8) | ((data[1] & 0xFC) << 3) | (data[2] >> 3);
      data += 3;
      len -= 3;
    } else {
      pixel = (data[0] << 8) | data[1];
      data += 2;
      len -= 2;
    }
    
    lineBuffer[x] = pixel;
    x++;
    imgState.receivedPixels++;
    
    // When one full line is received, draw it.
    if (x == imgState.width) {
      drawPixelLine(imgState.startX, imgState.startY + y, lineBuffer, imgState.width);
      x = 0;
      y++;
      // Show progress every 10 lines.
      if (y % 10 == 0) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Loading: %d%%", (imgState.receivedPixels * 100UL) / imgState.totalPixels);
        showBLEMessage(msg);
      }
    }
  }
  
  // Every 50 packets (or when complete), send an acknowledgment.
  if ((imgState.receivedPixels % (50 * imgState.width)) == 0 ||
       imgState.receivedPixels == imgState.totalPixels) {
    uint8_t ackData[51];
    ackData[0] = imgState.currentGroup;
    memcpy(ackData + 1, imgState.groupPackets, 50);
    if (pTxCharacteristic) {
      pTxCharacteristic->setValue(ackData, 51);
      pTxCharacteristic->notify();
    }
    imgState.currentGroup++;
    memset(imgState.groupPackets, 0, sizeof(imgState.groupPackets));
  }
  
  // When the entire image has been received…
  if (imgState.receivedPixels == imgState.totalPixels) {
    uint32_t duration = millis() - rxStartTime;
    float speed = (imgState.totalPixels * ((imgState.colorBit == 24) ? 3 : 2)) / (duration/1000.0f);
    char msg[64];
    snprintf(msg, sizeof(msg), "Complete: %.1f KB/s", speed/1024.0f);
    showBLEMessage(msg, 0x07E0); // green text
    imgState.isReceiving = false;
    imgState.imageDisplayed = true;  // Set the flag
    if (lineBuffer) {
      free(lineBuffer);
      lineBuffer = NULL;
    }
    
    // Stop advertising since we're displaying an image
    if (pServer) {
      pServer->getAdvertising()->stop();
    }
  }
}

// ---------------- Mode Switching Functions ----------------
// When switching modes, we want to provide visual feedback and start/stop BLE advertising.

// Enter BLE mode: stop LVGL eyes mode, clear the screen, and start BLE advertising.
void enterBLEMode() {
  currentMode = OPERATION_BLE;
  USBSerial.println("Switching to BLE mode");
  
  lv_obj_clean(lv_scr_act());
  gfx->fillScreen(0);
  
  if (!imgState.imageDisplayed) {
    showBLEMessage("BLE Mode:\nAdvertising...");
    memset(&imgState, 0, sizeof(imgState));
    if (lineBuffer) {
      free(lineBuffer);
      lineBuffer = NULL;
    }
    
    if (pServer) {
      pServer->getAdvertising()->start();
    }
  } else {
    showBLEMessage("BLE Mode:\nPress button to advertise");
  }
  
  lastBLEActivity = millis();
}

// Exit BLE mode: stop advertising/disconnect BLE and return to eyes mode.
void exitBLEMode() {
  currentMode = OPERATION_EYES;
  USBSerial.println("Exiting BLE mode, switching to Googly Eyes");
  
  // If a client is connected, disconnect it.
  if (pServer)
    pServer->disconnect(0);
  
  // Stop advertising.
  if (pServer)
    pServer->getAdvertising()->stop();
  
  // Free any pending image state.
  if (lineBuffer) {
    free(lineBuffer);
    lineBuffer = NULL;
  }
  memset(&imgState, 0, sizeof(imgState));
  
  // Reinitialize the eyes UI.
  createEyes();
  showLVGLMessage("Googly Eyes Mode");
}

// ---------------- Button Handling ----------------
// The same physical button is used both for power management/calibration and for mode switching.
// • Long press (>= LONG_PRESS_TIME) powers off.
// • In calibration mode (if calibration isn't complete) a short press triggers calibration steps.
// • Otherwise, a short press in eyes mode toggles to BLE mode (or a double–click re–starts calibration).
// • In BLE mode a short press toggles back to eyes mode.
void handleButton() {
  int buttonState = digitalRead(powerInputPin);
  static bool wasPressed = false;
  static unsigned long pressStart = 0;
  
  if (buttonState == LOW && !wasPressed) {
    pressStart = millis();
    wasPressed = true;
  }
  else if (buttonState == HIGH && wasPressed) {
    unsigned long pressDuration = millis() - pressStart;
    wasPressed = false;
    
    if (pressDuration >= LONG_PRESS_TIME) {
      showLVGLMessage("Powering off...");
      delay(500);
      digitalWrite(powerOutputPin, LOW);  // Shut down power via the SYS_EN line
    }
    else if (pressDuration >= MEDIUM_PRESS_TIME) {
      // Medium press triggers calibration
      startCalibration();
      USBSerial.println("Medium press: Starting calibration.");
    }
    else {
      // Short press handling
      if (!calibrationData.isValid || debugMode) {
        if (currentCalStep == CAL_PROMPT) {
          calButtonTriggered = true;
        }
        else if (debugMode) {
          debugMode = false;
          createEyes();
          USBSerial.println("Exiting calibration debug mode, showing eyes.");
        }
      }
      else {
        if (currentMode == OPERATION_EYES) {
          enterBLEMode();
        }
        else {  // currentMode == OPERATION_BLE
          if (imgState.imageDisplayed) {
            // If displaying image, first press restarts advertising
            imgState.imageDisplayed = false;
            showBLEMessage("BLE Mode:\nAdvertising...");
            if (pServer) {
              pServer->getAdvertising()->start();
            }
            lastBLEActivity = millis();
          } else {
            // If already advertising or no image, switch back to eyes
            exitBLEMode();
          }
        }
      }
    }
  }
}

// ---------------- BLE Callback Classes ----------------

// Server callbacks: notify when a client connects/disconnects.
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    USBSerial.println("BLE Client connected");
    if (currentMode == OPERATION_BLE)
      showBLEMessage("Client connected\nReady for image");
    lastBLEActivity = millis();
  }
  
  void onDisconnect(BLEServer* pServer) {
    USBSerial.println("BLE Client disconnected");
    if (currentMode == OPERATION_BLE && !imgState.imageDisplayed) {
      showBLEMessage("Client disconnected\nAdvertising...");
      pServer->getAdvertising()->start();
    } else if (currentMode == OPERATION_BLE && imgState.imageDisplayed) {
      // If we're displaying an image, just show status but don't restart advertising
      showBLEMessage("Client disconnected\nPress button to advertise");
    }
  }
};

// RX callback: process writes to the RX characteristic.
class RxCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Only process if we are in BLE mode.
    if (currentMode != OPERATION_BLE)
      return;
    
    String value = pCharacteristic->getValue();
    const uint8_t* data = (const uint8_t*)value.c_str();
    size_t len = value.length();
    
    lastBLEActivity = millis();
    
    // If we are not yet receiving an image, look for the header "!I"
    if (!imgState.isReceiving) {
      rxStartTime = millis();
      if (len < 7 || data[0] != '!' || data[1] != 'I')
        return;
      
      uint8_t colorBit = data[2];
      uint16_t width = data[3] | (data[4] << 8);
      uint16_t height = data[5] | (data[6] << 8);
      
      // Validate dimensions
      if (width == 0 || height == 0 || width > LCD_WIDTH || height > LCD_HEIGHT)
      {
        showBLEMessage("Error: Invalid dimensions");
        return;
      }
      
      startNewImage(width, height, colorBit);
      data += 7;
      len -= 7;
    }
    
    // Process any remaining pixel data.
    if (len > 0) {
      processPixelData(data, len);
    }
  }
};

// ---------------- LVGL Setup Function ----------------
void setupLVGL() {
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
}

// ---------------- Setup ----------------
void setup() {
  USBSerial.begin(115200);
  USBSerial.println("Starting up...");

  // Initialize power management pins.
  pinMode(powerOutputPin, OUTPUT);
  pinMode(powerInputPin, INPUT_PULLUP);
  digitalWrite(powerOutputPin, HIGH);

  // Initialize display.
  gfx->begin();
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  setupLVGL();
  showLVGLMessage("Initializing...");

  // Initialize IMU via I2C.
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    showLVGLMessage("IMU Error!");
    while (1) delay(1000);
  }
  
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_2G,
    SensorQMI8658::ACC_ODR_1000Hz,
    SensorQMI8658::LPF_MODE_0,
    true
  );
  qmi.enableAccelerometer();
  
  qmi.configGyroscope(
    SensorQMI8658::GYR_RANGE_64DPS,
    SensorQMI8658::GYR_ODR_896_8Hz,
    SensorQMI8658::LPF_MODE_3,
    true
  );
  qmi.enableGyroscope();

  // Set up a periodic timer for LVGL tick increments.
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, 2 * 1000);  // period in microseconds
  
  // Load calibration data.
  loadCalibration();
  if (calibrationData.isValid) {
    createEyes();  // Go straight to eyes mode if we have calibration
    USBSerial.println("Using stored calibration.");
  } else {
    startCalibration();
  }

  // ---------------- BLE Setup ----------------
  // Initialize BLE but do not start advertising yet (only when switching to BLE mode).
  BLEDevice::init("ESP32 Eyes");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(UART_SERVICE_UUID);
  
  // Create TX (notify) characteristic.
  pTxCharacteristic = pService->createCharacteristic(
    UART_TX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  // Create RX (write) characteristic.
  pRxCharacteristic = pService->createCharacteristic(
    UART_RX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallback());
  
  pService->start();
  
  // As we start in eyes mode, do not advertise until the user toggles mode.
  // (Once in BLE mode, enterBLEMode() calls startAdvertising().)
  
  USBSerial.println("Setup complete");
}

// ---------------- Main Loop ----------------
void loop() {
  handleButton();
  
  if (currentMode == OPERATION_EYES) {
    if (!calibrationData.isValid || debugMode) {
      handleCalibration();
    }
    else {
      updateEyes();
    }
  }
  else if (currentMode == OPERATION_BLE) {
    // Only check timeout if we're advertising (not displaying image)
    if (!imgState.imageDisplayed && millis() - lastBLEActivity > BLE_MODE_TIMEOUT) {
      // Stop advertising but stay in BLE mode if we have an image
      if (pServer) {
        pServer->getAdvertising()->stop();
      }
      showBLEMessage("BLE timeout\nPress button to advertise");
    }
  }
  
  delay(2);
}