# ESP32-S3-LCD Image Display

This project implements a BLE image receiver for the ESP32-S3-LCD-1.69 display. It's compatible with the Adafruit Bluefruit LE Connect app's image transfer protocol.

## Features

- BLE image transfer with packet acknowledgment
- Progress display during transfer
- Optional NVM storage for last image
- Error handling and user feedback
- Support for RGB565 color format
- Automatic image scaling and centering

## Protocol

### Image Metadata
- Header: "!I" (0x21, 0x49)
- Color space: 16 (RGB565)
- Width: 2 bytes (little-endian)
- Height: 2 bytes (little-endian)

### Image Data
- Packet size: 16 bytes
- Interleaved transfer mode (50:1)
- Acknowledgment after each group of 50 packets
- RGB565 pixel format

### Status Updates
- Progress bar shows transfer percentage
- Serial output for debugging
- Error messages displayed on screen

## Hardware Requirements

- ESP32-S3-LCD-1.69 display
- BLE-capable device with Adafruit Bluefruit LE Connect app

## Usage

1. Upload sketch to ESP32-S3-LCD
2. Open Bluefruit LE Connect app
3. Connect to "ESP32 Display"
4. Select Image Transfer mode
5. Choose an image and send
6. Monitor progress on display

## Error Handling

- Invalid image format
- Memory allocation failures
- Connection loss
- Dimension validation
- Storage space checks

## Notes

- Maximum image dimensions: 240x280 (display resolution)
- Images are automatically centered
- Last successful image can be saved to NVM if space allows
- Uses LVGL for display management
