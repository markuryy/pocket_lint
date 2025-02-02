#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "lv_conf.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"
#include "HWCDC.h"
#include <Preferences.h>
#include <math.h>

HWCDC USBSerial;
Preferences preferences;

// ----- Power management & button pins -----
const int powerOutputPin = 41;  // SYS_EN pin (to control power)
const int powerInputPin  = 40;  // Button pin

const unsigned long LONG_PRESS_TIME = 3000;  // 3 sec for power off
const unsigned long DOUBLE_CLICK_TIME = 300;   // 300ms for double click

// ----- IMU and sensor data -----
SensorQMI8658 qmi;
IMUdata acc;
IMUdata gyr;

// ----- LVGL globals for "googly eyes" -----
lv_obj_t *leftEye;
lv_obj_t *rightEye;
lv_obj_t *leftPupil;
lv_obj_t *rightPupil;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

// ----- Display setup -----
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

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

void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void showMessage(const char* message) {
  lv_obj_clean(lv_scr_act());
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, message);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_timer_handler();
}

// ----- Create the Eyes (with clipping & no scroll bars) -----
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

// ----- Calibration Enums & Structures -----
// We use 4 faces (BOTTOM, LEFT, TOP, BACK); FRONT and RIGHT are not available.
enum CalibrationOrientation {
  CAL_ORIENT_BOTTOM = 0,
  CAL_ORIENT_LEFT,
  CAL_ORIENT_TOP,
  CAL_ORIENT_BACK,
  CAL_ORIENT_COUNT  // Total orientations
};

enum CalStepState {
  CAL_PROMPT,    // Waiting for user placement & button press
  CAL_COUNTDOWN, // Countdown period after button press
  CAL_SAMPLING,  // Collecting samples
  CAL_DONE       // Finished with this orientation
};

struct CalibrationData {
  float bottom[3];   // Device resting on its BOTTOM face (table)
  float left[3];     // LEFT face down
  float top[3];      // TOP face down (upside down relative to bottom)
  float back[3];     // BACK face down
  bool isValid;
  // For horizontal mapping, sensitivityX is the difference in Y between LEFT and BOTTOM.
  // For vertical mapping, the full range is derived from (BOTTOM.x - TOP.x).
  float sensitivityX;
  float sensitivityY;
};

CalibrationData calibrationData;

// ----- Calibration State Machine Globals -----
CalibrationOrientation currentOrientation = CAL_ORIENT_BOTTOM;
CalStepState currentCalStep = CAL_PROMPT;
unsigned long calStepStartTime = 0;
const unsigned long COUNTDOWN_DURATION = 3000; // 3-second countdown
const int CALIBRATION_SAMPLES = 50;
int calSampleCount = 0;
float calSampleSum[3] = {0, 0, 0};
bool calButtonTriggered = false; // Set when user presses button in PROMPT

// ----- Debug Mode Flag -----
// After calibration, debugMode displays calibration data until exited.
bool debugMode = false;

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
  showMessage(buf);
  USBSerial.println(buf);
}

// ----- Start Calibration -----
// Resets calibration data and begins with the BOTTOM orientation.
void startCalibration() {
  calibrationData.isValid = false;
  debugMode = false;
  currentOrientation = CAL_ORIENT_BOTTOM;
  currentCalStep = CAL_PROMPT;
  calSampleCount = 0;
  memset(calSampleSum, 0, sizeof(calSampleSum));
  calStepStartTime = millis();
  calButtonTriggered = false;
  showMessage("Calibration:\nPlace me on my BOTTOM side\nand press the button.");
  USBSerial.println("Starting calibration: BOTTOM");
}

// ----- Handle Calibration State Machine -----
void handleCalibration() {
  if (!qmi.getDataReady() || !qmi.getAccelerometer(acc.x, acc.y, acc.z))
    return;
    
  char msg[128];
  unsigned long now = millis();
  
  switch (currentCalStep) {
    case CAL_PROMPT:
      if (currentOrientation == CAL_ORIENT_BOTTOM)
        showMessage("Calibration:\nPlace me on my BOTTOM side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_LEFT)
        showMessage("Calibration:\nPlace me on my LEFT side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_TOP)
        showMessage("Calibration:\nPlace me on my TOP side\nand press the button.");
      else if (currentOrientation == CAL_ORIENT_BACK)
        showMessage("Calibration:\nPlace me on my BACK side\nand press the button.");
      
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
      showMessage(msg);
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
      showMessage(msg);
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
          // Compute sensitivities.
          calibrationData.sensitivityX = fabs(calibrationData.left[1] - calibrationData.bottom[1]);
          calibrationData.sensitivityY = fabs(calibrationData.bottom[0] - calibrationData.top[0]);
          calibrationData.isValid = true;
          currentCalStep = CAL_DONE;
          debugMode = true; // Enter debug mode so the data stays on screen.
          showMessage("Calibration Complete!\nPress button to continue.");
          USBSerial.println("Calibration complete!");
          // Save calibration to nonvolatile storage.
          preferences.begin("googly", false);
          preferences.putBytes("cal_data", &calibrationData, sizeof(CalibrationData));
          preferences.end();
          USBSerial.println("Calibration saved to NVS.");
          showCalibrationDebug();
        }
      }
      break;
      
    case CAL_DONE:
      // Nothing to do.
      break;
  }
}

// ----- updateEyes() -----
// When calibration is valid and debug mode is off, update the pupils.
// Vertical mapping: we use acc.x differences so that the BOTTOM reading maps to +25 pixels (pupil at bottom)
// and the TOP reading maps to -25 pixels (pupil at top).
// Horizontal mapping uses acc.y differences (with a negative multiplier so that tilting left moves the pupil left).
// Finally, we constrain the (x,y) offset to a circle of radius 25.
void updateEyes() {
  if (!calibrationData.isValid || debugMode)
    return;
  
  if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
    float deltaX = acc.x - calibrationData.bottom[0]; // vertical difference (using X axis)
    float computedY = deltaX * 200;  // scale factor (200 = 50 / 0.25) so that full range maps to 50 pixels
    float finalY = computedY + 25;   // shift so that BOTTOM (deltaX=0) maps to +25 (pupil at bottom)

    float deltaY = acc.y - calibrationData.bottom[1]; // horizontal difference (using Y axis)
    float computedX = -deltaY * (25.0 / calibrationData.sensitivityX);
    
    // Constrain to circle of radius 25.
    const int maxOffset = 25;
    float r = sqrt(computedX * computedX + finalY * finalY);
    if (r > maxOffset) {
      computedX *= (maxOffset / r);
      finalY   *= (maxOffset / r);
    }
    
    // Apply smoothing to simulate inertia.
    static float currentX = 0, currentY = 0;
    float smoothing = 0.6;
    currentX = currentX + (computedX - currentX) * smoothing;
    currentY = currentY + (finalY - currentY) * smoothing;
    
    lv_obj_align(leftPupil, LV_ALIGN_CENTER, (int)currentX, (int)currentY);
    lv_obj_align(rightPupil, LV_ALIGN_CENTER, (int)currentX, (int)currentY);
    lv_timer_handler();
  }
}

// ----- Button Handling -----
// In calibration PROMPT mode, a short press sets a flag.
// In debug mode, a short press exits debug mode.
// Otherwise, a double-click triggers recalibration.
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
      showMessage("Powering off...");
      delay(500);
      digitalWrite(powerOutputPin, LOW);
    }
    else {
      if (!calibrationData.isValid && currentCalStep == CAL_PROMPT) {
        calButtonTriggered = true;
      }
      else if (debugMode) {
        debugMode = false;
        createEyes(); // Exit debug mode and show the eyes.
        USBSerial.println("Exiting debug mode, showing eyes.");
      }
      else {
        static unsigned long lastClickTime = 0;
        unsigned long now = millis();
        if (now - lastClickTime <= DOUBLE_CLICK_TIME) {
          startCalibration();
          USBSerial.println("Double-click: Restarting calibration.");
        }
        lastClickTime = now;
      }
    }
  }
}

// ----- Load Calibration from NVS -----
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
  showMessage("Initializing...");

  // Initialize IMU.
  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);
  
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    showMessage("IMU Error!");
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

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  loadCalibration();
  if (calibrationData.isValid) {
    debugMode = true;
    showCalibrationDebug();
    USBSerial.println("Using stored calibration.");
  } else {
    startCalibration();
  }

  USBSerial.println("Setup complete");
}

void loop() {
  handleButton();
  
  if (!calibrationData.isValid || debugMode) {
    handleCalibration();
  }
  else {
    updateEyes();
  }
  
  // --- Additional: Log gyroscope data for debugging ---
  if (qmi.getDataReady()) {
    if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
      USBSerial.print("{GYRO: ");
      USBSerial.print(gyr.x);
      USBSerial.print(", ");
      USBSerial.print(gyr.y);
      USBSerial.print(", ");
      USBSerial.print(gyr.z);
      USBSerial.println("}");
    }
  }
  
  delay(2);
}
