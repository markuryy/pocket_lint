#pragma once

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

// Color depth (16 for RGB565)
#define LV_COLOR_DEPTH          16

// Enable features needed for image display
#define LV_USE_IMG             1
#define LV_USE_LABEL           1
#define LV_USE_BAR             1
#define LV_USE_ANIMIMG         1

// Memory usage settings
#define LV_MEM_CUSTOM          1
#define LV_MEM_SIZE           (48U * 1024U)    // 48KB for LVGL memory
#define LV_MEM_POOL_INCLUDE   <stdlib.h>
#define LV_MEM_POOL_ALLOC     malloc
#define LV_MEM_POOL_FREE      free

// Display settings
#define LV_HOR_RES_MAX        240
#define LV_VER_RES_MAX        280
#define LV_DPI_DEF            130
#define LV_DISP_DEF_REFR_PERIOD 30

// Color settings
#define LV_COLOR_16_SWAP      0

// Input device settings (disabled for now)
#define LV_USE_INDEV_TOUCHPAD 0
#define LV_USE_INDEV_MOUSE    0
#define LV_USE_INDEV_KEYPAD   0
#define LV_USE_INDEV_ENCODER  0

// Theme settings
#define LV_USE_THEME_DEFAULT  1
#define LV_THEME_DEFAULT_DARK 0
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT       &lv_font_montserrat_14

// Debug settings
#define LV_USE_LOG            1
#define LV_LOG_LEVEL         LV_LOG_LEVEL_INFO
#define LV_LOG_PRINTF        1

// Image decoder and cache
#define LV_IMG_CACHE_DEF_SIZE 5
#define LV_USE_IMG_TRANSFORM  1

// Animation settings
#define LV_USE_ANIMATION      1
#define LV_USE_TRANSITION     1

// Draw buffers
#define LV_VDB_SIZE          (LCD_WIDTH * 10)
#define LV_VDB_ADR           0

// Garbage collection settings
#define LV_MEMCPY_MEMSET_STD 1

// Enable GPU
#define LV_USE_GPU           0
#define LV_USE_GPU_STM32_DMA2D 0
#define LV_USE_GPU_NXP_PXP   0
#define LV_USE_GPU_NXP_VG_LITE 0
#define LV_USE_GPU_SDL       0

// Disable unused features to save memory
#define LV_USE_ARC           0
#define LV_USE_CALENDAR      0
#define LV_USE_CHART         0
#define LV_USE_CHECKBOX      0
#define LV_USE_DROPDOWN      0
#define LV_USE_KEYBOARD      0
#define LV_USE_LIST          0
#define LV_USE_MENU          0
#define LV_USE_METER         0
#define LV_USE_MSGBOX        0
#define LV_USE_SPINBOX       0
#define LV_USE_SPINNER       0
#define LV_USE_SWITCH        0
#define LV_USE_TEXTAREA      0
#define LV_USE_TABLE         0
#define LV_USE_TABVIEW       0
#define LV_USE_TILEVIEW      0
#define LV_USE_WIN           0

#endif // LV_CONF_H
