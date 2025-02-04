#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "HWCDC.h"
#include <Preferences.h>

HWCDC USBSerial;
Preferences preferences;

// Display setup
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

// BLE UART Service
#define UART_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_TX_CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;

// Image transfer state
struct {
  uint16_t width;
  uint16_t height;
  uint8_t colorBit;
  uint32_t totalPixels;
  uint32_t receivedPixels;
  uint16_t startX;
  uint16_t startY;
  bool isReceiving;
  uint8_t currentGroup;
  bool groupPackets[50];
} imgState;

// Pixel line buffer
uint16_t* lineBuffer = NULL;

// Statistics
uint32_t rxStartTime = 0;
uint32_t rxLastTime = 0;

void showMessage(const char* message, uint16_t color = 0xFFFF) {
  gfx->setTextColor(color);
  gfx->setCursor(0, gfx->height() - 20);
  gfx->fillRect(0, gfx->height() - 20, gfx->width(), 20, 0);
  gfx->println(message);
  USBSerial.println(message);
}

// Draw a line of pixels
void drawPixelLine(int16_t x, int16_t y, uint16_t* pixels, int16_t w) {
  for (int16_t i = 0; i < w; i++) {
    gfx->drawPixel(x + i, y, pixels[i]);
  }
}

void startNewImage(uint16_t width, uint16_t height, uint8_t colorBit) {
  // Clear previous state
  if (lineBuffer) {
    free(lineBuffer);
  }
  
  // Allocate line buffer
  lineBuffer = (uint16_t*)malloc(width * sizeof(uint16_t));
  if (!lineBuffer) {
    showMessage("Error: Out of memory");
    return;
  }
  
  // Calculate centered position
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
  
  // Clear display and show initial progress
  gfx->fillScreen(0);
  showMessage("Starting transfer...");
  
  USBSerial.printf("New image: %dx%d, %d-bit\n", width, height, colorBit);
}

void processPixelData(const uint8_t* data, size_t len) {
  if (!imgState.isReceiving || !lineBuffer) return;
  
  uint16_t x = imgState.receivedPixels % imgState.width;
  uint16_t y = imgState.receivedPixels / imgState.width;
  
  while (len >= (imgState.colorBit == 24 ? 3 : 2) && y < imgState.height) {
    // Convert pixel data
    uint16_t pixel;
    if (imgState.colorBit == 24) {
      pixel = ((data[0] & 0xF8) << 8) | ((data[1] & 0xFC) << 3) | (data[2] >> 3);
      data += 3;
      len -= 3;
    } else {
      pixel = (data[0] << 8) | data[1];
      data += 2;
      len -= 2;
    }
    
    // Store pixel in line buffer
    lineBuffer[x] = pixel;
    x++;
    imgState.receivedPixels++;
    
    // Draw line when complete
    if (x == imgState.width) {
      drawPixelLine(imgState.startX, imgState.startY + y, lineBuffer, imgState.width);
      x = 0;
      y++;
      
      // Show progress every 10 lines
      if (y % 10 == 0) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Loading: %d%%", (imgState.receivedPixels * 100) / imgState.totalPixels);
        showMessage(msg);
      }
    }
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    USBSerial.println("Client connected");
    showMessage("Connected - Ready for image");
  }
  
  void onDisconnect(BLEServer* pServer) {
    USBSerial.println("Client disconnected");
    BLEDevice::startAdvertising();
    
    if (imgState.isReceiving) {
      imgState.isReceiving = false;
      if (lineBuffer) {
        free(lineBuffer);
        lineBuffer = NULL;
      }
    }
    
    gfx->fillScreen(0);
    showMessage("Advertising...");
  }
};

class RxCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    const uint8_t* data = (const uint8_t*)value.c_str();
    size_t len = value.length();
    
    rxLastTime = millis();
    
    // New image starting
    if (!imgState.isReceiving) {
      rxStartTime = millis();
      
      // Look for "!I" header
      if (len < 7 || data[0] != '!' || data[1] != 'I') return;
      
      uint8_t colorBit = data[2];
      uint16_t width = data[3] | (data[4] << 8);
      uint16_t height = data[5] | (data[6] << 8);
      
      // Validate dimensions
      if (width == 0 || height == 0 || width > LCD_WIDTH || height > LCD_HEIGHT) {
        showMessage("Error: Invalid dimensions");
        return;
      }
      
      startNewImage(width, height, colorBit);
      data += 7;
      len -= 7;
    }
    
    // Process pixel data
    if (len > 0) {
      processPixelData(data, len);
      
      // Check if group complete (every 50 packets)
      if ((imgState.receivedPixels % (50 * imgState.width)) == 0 || 
          imgState.receivedPixels == imgState.totalPixels) {
        // Send acknowledgment
        uint8_t ackData[51];
        ackData[0] = imgState.currentGroup;
        memcpy(ackData + 1, imgState.groupPackets, 50);
        pTxCharacteristic->setValue(ackData, 51);
        pTxCharacteristic->notify();
        
        imgState.currentGroup++;
        memset(imgState.groupPackets, 0, sizeof(imgState.groupPackets));
      }
      
      // Check if image complete
      if (imgState.receivedPixels == imgState.totalPixels) {
        uint32_t duration = rxLastTime - rxStartTime;
        float speed = (imgState.totalPixels * (imgState.colorBit/8)) / (duration/1000.0f);
        
        char msg[64];
        snprintf(msg, sizeof(msg), "Complete: %.1f KB/s", speed/1024.0f);
        showMessage(msg, 0x07E0);
        
        imgState.isReceiving = false;
        if (lineBuffer) {
          free(lineBuffer);
          lineBuffer = NULL;
        }
      }
    }
  }
};

void setup() {
  USBSerial.begin(115200);
  USBSerial.println("Starting Image Display...");
  
  // Initialize display
  gfx->begin();
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  gfx->fillScreen(0);
  gfx->setTextSize(1);
  USBSerial.println("Display initialized");
  
  // Initialize BLE
  BLEDevice::init("ESP32 Display");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create UART service
  pService = pServer->createService(UART_SERVICE_UUID);
  
  // Create characteristics
  pTxCharacteristic = pService->createCharacteristic(
    UART_TX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  
  pRxCharacteristic = pService->createCharacteristic(
    UART_RX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallback());
  
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  showMessage("Advertising...");
  USBSerial.println("Setup complete");
}

void loop() {
  delay(10);
}
