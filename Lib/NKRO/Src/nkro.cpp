#include "nkro.hpp"
#include "stm32g4xx_hal.h"
#include <stdio.h>

// ===== Flash設定 =====
// STM32G431KB: 128KB Flash, Bank 1, Page 2KB × 64
// 3ページ使用 (60-62): 6KB for 108 keys
#define FLASH_USER_START_ADDR   0x0801E000
#define FLASH_PAGE_START        60
#define FLASH_PAGE_COUNT        3
#define FLASH_MAGIC_NUMBER      0xC00F0005  // v5: full keyboard

// デフォルト値
#define DEFAULT_SENSITIVITY  50
#define DEFAULT_DEAD_ZONE    30
#define DEFAULT_LED_MODE     LED_MODE_FADE
#define DEFAULT_LED_BRIGHT   255
#define DEFAULT_LED_SPEED    50
#define INITIAL_CALIBRATION_SAMPLES 16

// デフォルトキーコード (キーインデックス順: 108キー)
static const uint8_t DEFAULT_KEYCODES[108] = {
    // Source 0 (ADC1_IN1, PA0) MUX 0-15: 16 keys
    0x28, // 0:  Enter
    0xE5, // 1:  Right Shift
    0x50, // 2:  ← (Left Arrow)
    0x51, // 3:  ↓ (Down Arrow)
    0x4F, // 4:  → (Right Arrow)
    0x52, // 5:  ↑ (Up Arrow)
    0x62, // 6:  Numpad 0
    0x63, // 7:  Numpad .
    0x31, // 8:  Backslash
    0x2A, // 9:  Backspace
    0x44, // 10: F11
    0x43, // 11: F10
    0x2E, // 12: = (Equal)
    0x30, // 13: ] (Right Bracket)
    0x2D, // 14: - (Minus)
    0x42, // 15: F9

    // Source 1 (ADC1_IN2, PA1) MUX 0-15: 16 keys
    0x33, // 16: ; (Semicolon)
    0x0F, // 17: L
    0x37, // 18: . (Period)
    0x38, // 19: / (Slash)
    0xE7, // 20: Right GUI (Windows)
    0x34, // 21: ' (Quote)
    0x65, // 22: Menu (Application)
    0xE4, // 23: Right Ctrl
    0x2F, // 24: [ (Left Bracket)
    0x41, // 25: F8
    0x27, // 26: 0
    0x13, // 27: P
    0x40, // 28: F7
    0x26, // 29: 9
    0x3F, // 30: F6
    0x12, // 31: O

    // Source 2 (ADC2_IN3, PA6) MUX 0-3,5-15 (skip MUX4): 15 keys
    0x0B, // 32: H
    0x0D, // 33: J
    0x11, // 34: N
    0x10, // 35: M
    // MUX4 skipped (割当なし)
    0x0E, // 36: K
    0x36, // 37: , (Comma)
    0xE6, // 38: Right Alt
    0x25, // 39: 8
    0x0C, // 40: I
    0x3E, // 41: F5
    0x24, // 42: 7
    0x18, // 43: U
    0x3D, // 44: F4
    0x1C, // 45: Y
    0x23, // 46: 6

    // Source 3 (ADC2_IN4, PA7) MUX 0-15: 16 keys
    0x1B, // 47: X
    0x07, // 48: D
    0x09, // 49: F
    0x06, // 50: C
    0x2C, // 51: Space
    0x0A, // 52: G
    0x19, // 53: V
    0x05, // 54: B
    0x22, // 55: 5
    0x17, // 56: T
    0x3C, // 57: F3
    0x21, // 58: 4
    0x15, // 59: R
    0x08, // 60: E
    0x3B, // 61: F2
    0x20, // 62: 3

    // Source 4 (ADC2_IN10, PF1) MUX 0-15: 16 keys
    0x61, // 63: Numpad 9
    0x56, // 64: Numpad -
    0x60, // 65: Numpad 8
    0x00, // 66: ユーザーキー1 (初期未割当)
    0x55, // 67: Numpad *
    0x00, // 68: ユーザーキー2 (初期未割当)
    0x5F, // 69: Numpad 7
    0x54, // 70: Numpad /
    0x59, // 71: Numpad 1
    0x5C, // 72: Numpad 4
    0x5A, // 73: Numpad 2
    0x58, // 74: Numpad Enter
    0x5D, // 75: Numpad 5
    0x57, // 76: Numpad +
    0x5E, // 77: Numpad 6
    0x5B, // 78: Numpad 3

    // Source 5 (ADC1_IN10, PF0) MUX 0-6,10-15 (skip MUX7-9): 13 keys
    0x47, // 79: Scroll Lock
    0x4A, // 80: Home
    0x46, // 81: Print Screen
    0x45, // 82: F12
    0x49, // 83: Insert
    0x4C, // 84: Delete
    0x4D, // 85: End
    // MUX7-9 skipped (未割り当て)
    0x4E, // 86: Page Down
    0x53, // 87: Num Lock
    0x00, // 88: ユーザーキー3 (初期未割当)
    0x4B, // 89: Page Up
    0x00, // 90: ユーザーキー4 (初期未割当)
    0x48, // 91: Pause

    // Source 6 (ADC1_IN15, PB0) MUX 0-15: 16 keys
    0x39, // 92:  Caps Lock
    0xE1, // 93:  Left Shift
    0xE0, // 94:  Left Ctrl
    0x04, // 95:  A
    0x16, // 96:  S
    0x1D, // 97:  Z
    0xE3, // 98:  Left GUI (Windows)
    0xE2, // 99:  Left Alt
    0x3A, // 100: F1
    0x1F, // 101: 2
    0x1A, // 102: W
    0x1E, // 103: 1
    0x14, // 104: Q
    0x2B, // 105: Tab
    0x29, // 106: Escape
    0x35, // 107: ` (Grave)
};

RapidTriggerKeyboard::RapidTriggerKeyboard() {
    flash_status = -1;
    init();
}

void RapidTriggerKeyboard::init() {
    ledConfig.mode = DEFAULT_LED_MODE;
    ledConfig.brightness = DEFAULT_LED_BRIGHT;
    ledConfig.speed = DEFAULT_LED_SPEED;

    // マッピング配列初期化
    for (int src = 0; src < SOURCE_COUNT; src++) {
        for (int i = 0; i < MUX_CH_COUNT; i++) {
            keyMapping[src][i] = -1;
        }
    }

    int keyCounter = 0;

    auto registerKey = [&](int source, int muxChannel, uint8_t hidCode) {
        if (keyCounter >= TOTAL_KEY_COUNT) return;

        keyMapping[source][muxChannel] = keyCounter;

        keyStates[keyCounter].high_peak = 0;
        keyStates[keyCounter].low_peak = 4096;
        keyStates[keyCounter].is_active = false;
        keyStates[keyCounter].baseline = 0;
        keyStates[keyCounter].calibrated = false;
        keyStates[keyCounter].sensitivity = DEFAULT_SENSITIVITY;
        keyStates[keyCounter].keycode = hidCode;
        keyStates[keyCounter].dead_zone = DEFAULT_DEAD_ZONE;
        keyStates[keyCounter].calibration_samples = 0;
        keyStates[keyCounter].calibration_sum = 0;
        keyStates[keyCounter].macro_step_count = 0;
        memset(keyStates[keyCounter].macro_steps, 0, sizeof(keyStates[keyCounter].macro_steps));
        keyStates[keyCounter].was_active = false;
        keyStates[keyCounter].macro_exec_step = 0;
        keyStates[keyCounter].macro_exec_pressing = false;
        keyStates[keyCounter].macro_exec_tick = 0;

        keyCounter++;
    };

    // === Source 0: ADC1_IN1 (PA0) - MUX 0-15 ===
    registerKey(0, 0,  0x28); // Enter
    registerKey(0, 1,  0xE5); // Right Shift
    registerKey(0, 2,  0x50); // ←
    registerKey(0, 3,  0x51); // ↓
    registerKey(0, 4,  0x4F); // →
    registerKey(0, 5,  0x52); // ↑
    registerKey(0, 6,  0x62); // Numpad 0
    registerKey(0, 7,  0x63); // Numpad .
    registerKey(0, 8,  0x31); // Backslash
    registerKey(0, 9,  0x2A); // Backspace
    registerKey(0, 10, 0x44); // F11
    registerKey(0, 11, 0x43); // F10
    registerKey(0, 12, 0x2E); // =
    registerKey(0, 13, 0x30); // ]
    registerKey(0, 14, 0x2D); // -
    registerKey(0, 15, 0x42); // F9

    // === Source 1: ADC1_IN2 (PA1) - MUX 0-15 ===
    registerKey(1, 0,  0x33); // ;
    registerKey(1, 1,  0x0F); // L
    registerKey(1, 2,  0x37); // .
    registerKey(1, 3,  0x38); // /
    registerKey(1, 4,  0xE7); // Right GUI
    registerKey(1, 5,  0x34); // '
    registerKey(1, 6,  0x65); // Menu
    registerKey(1, 7,  0xE4); // Right Ctrl
    registerKey(1, 8,  0x2F); // [
    registerKey(1, 9,  0x41); // F8
    registerKey(1, 10, 0x27); // 0
    registerKey(1, 11, 0x13); // P
    registerKey(1, 12, 0x40); // F7
    registerKey(1, 13, 0x26); // 9
    registerKey(1, 14, 0x3F); // F6
    registerKey(1, 15, 0x12); // O

    // === Source 2: ADC2_IN3 (PA6) - skip MUX4 ===
    registerKey(2, 0,  0x0B); // H
    registerKey(2, 1,  0x0D); // J
    registerKey(2, 2,  0x11); // N
    registerKey(2, 3,  0x10); // M
    // MUX 4: 割当なし (keyMapping[2][4] = -1)
    registerKey(2, 5,  0x0E); // K
    registerKey(2, 6,  0x36); // ,
    registerKey(2, 7,  0xE6); // Right Alt
    registerKey(2, 8,  0x25); // 8
    registerKey(2, 9,  0x0C); // I
    registerKey(2, 10, 0x3E); // F5
    registerKey(2, 11, 0x24); // 7
    registerKey(2, 12, 0x18); // U
    registerKey(2, 13, 0x3D); // F4
    registerKey(2, 14, 0x1C); // Y
    registerKey(2, 15, 0x23); // 6

    // === Source 3: ADC2_IN4 (PA7) - MUX 0-15 ===
    registerKey(3, 0,  0x1B); // X
    registerKey(3, 1,  0x07); // D
    registerKey(3, 2,  0x09); // F
    registerKey(3, 3,  0x06); // C
    registerKey(3, 4,  0x2C); // Space
    registerKey(3, 5,  0x0A); // G
    registerKey(3, 6,  0x19); // V
    registerKey(3, 7,  0x05); // B
    registerKey(3, 8,  0x22); // 5
    registerKey(3, 9,  0x17); // T
    registerKey(3, 10, 0x3C); // F3
    registerKey(3, 11, 0x21); // 4
    registerKey(3, 12, 0x15); // R
    registerKey(3, 13, 0x08); // E
    registerKey(3, 14, 0x3B); // F2
    registerKey(3, 15, 0x20); // 3

    // === Source 4: ADC2_IN10 (PF1) - MUX 0-15 ===
    registerKey(4, 0,  0x61); // Numpad 9
    registerKey(4, 1,  0x56); // Numpad -
    registerKey(4, 2,  0x60); // Numpad 8
    registerKey(4, 3,  0x00); // User Key 1
    registerKey(4, 4,  0x55); // Numpad *
    registerKey(4, 5,  0x00); // User Key 2
    registerKey(4, 6,  0x5F); // Numpad 7
    registerKey(4, 7,  0x54); // Numpad /
    registerKey(4, 8,  0x59); // Numpad 1
    registerKey(4, 9,  0x5C); // Numpad 4
    registerKey(4, 10, 0x5A); // Numpad 2
    registerKey(4, 11, 0x58); // Numpad Enter
    registerKey(4, 12, 0x5D); // Numpad 5
    registerKey(4, 13, 0x57); // Numpad +
    registerKey(4, 14, 0x5E); // Numpad 6
    registerKey(4, 15, 0x5B); // Numpad 3

    // === Source 5: ADC1_IN10 (PF0) - skip MUX 7,8,9 ===
    registerKey(5, 0,  0x47); // Scroll Lock
    registerKey(5, 1,  0x4A); // Home
    registerKey(5, 2,  0x46); // Print Screen
    registerKey(5, 3,  0x45); // F12
    registerKey(5, 4,  0x49); // Insert
    registerKey(5, 5,  0x4C); // Delete
    registerKey(5, 6,  0x4D); // End
    // MUX 7,8,9: 未割り当て (keyMapping[5][7..9] = -1)
    registerKey(5, 10, 0x4E); // Page Down
    registerKey(5, 11, 0x53); // Num Lock
    registerKey(5, 12, 0x00); // User Key 3
    registerKey(5, 13, 0x4B); // Page Up
    registerKey(5, 14, 0x00); // User Key 4
    registerKey(5, 15, 0x48); // Pause

    // === Source 6: ADC1_IN15 (PB0) - MUX 0-15 ===
    registerKey(6, 0,  0x39); // Caps Lock
    registerKey(6, 1,  0xE1); // Left Shift
    registerKey(6, 2,  0xE0); // Left Ctrl
    registerKey(6, 3,  0x04); // A
    registerKey(6, 4,  0x16); // S
    registerKey(6, 5,  0x1D); // Z
    registerKey(6, 6,  0xE3); // Left GUI (Windows)
    registerKey(6, 7,  0xE2); // Left Alt
    registerKey(6, 8,  0x3A); // F1
    registerKey(6, 9,  0x1F); // 2
    registerKey(6, 10, 0x1A); // W
    registerKey(6, 11, 0x1E); // 1
    registerKey(6, 12, 0x14); // Q
    registerKey(6, 13, 0x2B); // Tab
    registerKey(6, 14, 0x29); // Escape
    registerKey(6, 15, 0x35); // ` (Grave)

    printf("[INIT] Registered %d keys\r\n", keyCounter);
}

void RapidTriggerKeyboard::updateKeyByMux(int muxIndex, int source, uint32_t adcValue) {
    if (source < 0 || source >= SOURCE_COUNT || muxIndex < 0 || muxIndex >= MUX_CH_COUNT) return;

    int stateIndex = keyMapping[source][muxIndex];
    if (stateIndex != -1) {
        updateRapidTriggerState(keyStates[stateIndex], adcValue);
    }
}

// ===== Getter / Setter =====

void RapidTriggerKeyboard::setSensitivity(int keyIndex, uint32_t value) {
    if (value < 1) value = 1;
    if (value > 1000) value = 1000;

    if (keyIndex == 255 || keyIndex == -1) {
        for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
            keyStates[i].sensitivity = value;
        }
    } else if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].sensitivity = value;
    }
}

uint32_t RapidTriggerKeyboard::getSensitivity(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].sensitivity;
    }
    return 0;
}

void RapidTriggerKeyboard::setKeycode(int keyIndex, uint8_t code) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].keycode = code;
    }
}

uint8_t RapidTriggerKeyboard::getKeycode(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].keycode;
    }
    return 0;
}

void RapidTriggerKeyboard::setDeadZone(int keyIndex, uint32_t value) {
    if (value > 500) value = 500;

    if (keyIndex == 255 || keyIndex == -1) {
        for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
            keyStates[i].dead_zone = value;
        }
    } else if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].dead_zone = value;
    }
}

uint32_t RapidTriggerKeyboard::getDeadZone(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].dead_zone;
    }
    return 0;
}

void RapidTriggerKeyboard::setMacro(int keyIndex, uint8_t stepCount, const MacroStep* steps) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        if (stepCount > MAX_MACRO_STEPS) stepCount = MAX_MACRO_STEPS;
        keyStates[keyIndex].macro_step_count = stepCount;
        for (int s = 0; s < stepCount; s++) {
            keyStates[keyIndex].macro_steps[s] = steps[s];
        }
        for (int s = stepCount; s < MAX_MACRO_STEPS; s++) {
            keyStates[keyIndex].macro_steps[s] = {0, 0};
        }
    }
}

uint8_t RapidTriggerKeyboard::getMacroStepCount(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].macro_step_count;
    }
    return 0;
}

const MacroStep* RapidTriggerKeyboard::getMacroSteps(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].macro_steps;
    }
    return nullptr;
}

int RapidTriggerKeyboard::getMappedKeyIndex(int source, int muxIndex) {
    if (source < 0 || source >= SOURCE_COUNT || muxIndex < 0 || muxIndex >= MUX_CH_COUNT) {
        return -1;
    }
    return keyMapping[source][muxIndex];
}

bool RapidTriggerKeyboard::isKeyActive(int keyIndex) {
    if (keyIndex < 0 || keyIndex >= TOTAL_KEY_COUNT) {
        return false;
    }
    return keyStates[keyIndex].is_active;
}

// ===== Flash 保存 / 読み込み =====
// Layout v5 (3 pages: 60-62, 6KB total)
// [0]       Magic (8B)
// [1]       LED Config (8B)
// [2..109]  KeyData × 108: each = 5 doublewords (40B)
//   DW0: sensitivity
//   DW1: dead_zone
//   DW2: keycode
//   DW3: macro_step_count | steps[0-3]
//   DW4: steps[4-7]
// Total: 16 + 108*40 = 4336B (fits in 3 × 2KB pages)

void RapidTriggerKeyboard::saveToFlash() {
    HAL_FLASH_Unlock();

    flash_status = 0;
    flash_error_code = 0;

    // Erase all 3 pages
    for (int p = 0; p < FLASH_PAGE_COUNT; p++) {
        FLASH_EraseInitTypeDef EraseInitStruct;
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.Banks = FLASH_BANK_1;
        EraseInitStruct.Page = FLASH_PAGE_START + p;
        EraseInitStruct.NbPages = 1;

        uint32_t PageError = 0;
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
            flash_status = 1;
            flash_error_code = PageError;
            HAL_FLASH_Lock();
            return;
        }
    }

    uint32_t address = FLASH_USER_START_ADDR;

    // Write Magic
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, (uint64_t)FLASH_MAGIC_NUMBER) != HAL_OK) {
        flash_status = 2;
        flash_error_code = HAL_FLASH_GetError();
        HAL_FLASH_Lock();
        return;
    }
    if (*(__IO uint32_t*)address != FLASH_MAGIC_NUMBER) {
        flash_status = 4;
        flash_debug_val = *(__IO uint32_t*)address;
        HAL_FLASH_Lock();
        return;
    }
    address += 8;

    // Write LED Config
    uint64_t led_data = (uint64_t)ledConfig.mode
                      | ((uint64_t)ledConfig.brightness << 8)
                      | ((uint64_t)ledConfig.speed << 16);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, led_data) != HAL_OK) {
        flash_status = 2;
        flash_error_code = HAL_FLASH_GetError();
        HAL_FLASH_Lock();
        return;
    }
    address += 8;

    // Write Key Data (5 DWs per key)
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        // DW0: sensitivity
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].sensitivity) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW1: dead_zone
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].dead_zone) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW2: keycode
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].keycode) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW3: macro steps[0-3] + step_count
        uint64_t macro_dw3 = (uint64_t)keyStates[i].macro_step_count;
        for (int s = 0; s < 4 && s < MAX_MACRO_STEPS; s++) {
            macro_dw3 |= ((uint64_t)keyStates[i].macro_steps[s].modifiers << (8 + s*16));
            macro_dw3 |= ((uint64_t)keyStates[i].macro_steps[s].keycode   << (16 + s*16));
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw3) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW4: macro steps[4-7]
        uint64_t macro_dw4 = 0;
        for (int s = 0; s < 4; s++) {
            int si = s + 4;
            if (si < MAX_MACRO_STEPS) {
                macro_dw4 |= ((uint64_t)keyStates[i].macro_steps[si].modifiers << (s*16));
                macro_dw4 |= ((uint64_t)keyStates[i].macro_steps[si].keycode   << (8 + s*16));
            }
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw4) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;
    }

    HAL_FLASH_Lock();
    printf("[FLASH] Saved %d keys\r\n", TOTAL_KEY_COUNT);
}

void RapidTriggerKeyboard::loadFromFlash() {
    uint32_t address = FLASH_USER_START_ADDR;

    uint32_t magic = *(__IO uint32_t*)address;
    if (magic != FLASH_MAGIC_NUMBER) {
        printf("[FLASH] No valid v5 data (magic=0x%08lX)\r\n", magic);
        return;
    }
    address += 8;

    // LED Config
    uint32_t led_raw = *(__IO uint32_t*)address;
    uint8_t m = led_raw & 0xFF;
    if (m < LED_MODE_COUNT) ledConfig.mode = (LedMode)m;
    ledConfig.brightness = (led_raw >> 8) & 0xFF;
    uint8_t s = (led_raw >> 16) & 0xFF;
    if (s <= 100) ledConfig.speed = s;
    address += 8;

    // Key Data
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        uint32_t sens = *(__IO uint32_t*)address;
        if (sens >= 1 && sens <= 1000) keyStates[i].sensitivity = sens;
        address += 8;

        uint32_t dz = *(__IO uint32_t*)address;
        if (dz <= 500) keyStates[i].dead_zone = dz;
        address += 8;

        uint32_t kc = *(__IO uint32_t*)address;
        // 0x00 is valid (user key), up to 0xE7 (modifiers)
        if (kc <= 0xE7) keyStates[i].keycode = (uint8_t)kc;
        address += 8;

        // DW3: macro steps[0-3] + step_count
        uint64_t macro_dw3 = *(__IO uint64_t*)address;
        uint8_t sc = macro_dw3 & 0xFF;
        if (sc <= MAX_MACRO_STEPS) {
            keyStates[i].macro_step_count = sc;
            for (int s = 0; s < 4 && s < MAX_MACRO_STEPS; s++) {
                keyStates[i].macro_steps[s].modifiers = (macro_dw3 >> (8 + s*16)) & 0xFF;
                keyStates[i].macro_steps[s].keycode   = (macro_dw3 >> (16 + s*16)) & 0xFF;
            }
        }
        address += 8;

        // DW4: macro steps[4-7]
        uint64_t macro_dw4 = *(__IO uint64_t*)address;
        for (int s = 0; s < 4; s++) {
            int si = s + 4;
            if (si < MAX_MACRO_STEPS) {
                keyStates[i].macro_steps[si].modifiers = (macro_dw4 >> (s*16)) & 0xFF;
                keyStates[i].macro_steps[si].keycode   = (macro_dw4 >> (8 + s*16)) & 0xFF;
            }
        }
        address += 8;
    }

    printf("[FLASH] Loaded %d keys\r\n", TOTAL_KEY_COUNT);
}

void RapidTriggerKeyboard::resetDefaults() {
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        keyStates[i].sensitivity = DEFAULT_SENSITIVITY;
        keyStates[i].dead_zone = DEFAULT_DEAD_ZONE;
        keyStates[i].keycode = DEFAULT_KEYCODES[i];
        keyStates[i].macro_step_count = 0;
        memset(keyStates[i].macro_steps, 0, sizeof(keyStates[i].macro_steps));
        keyStates[i].was_active = false;
        keyStates[i].macro_exec_step = 0;
        keyStates[i].macro_exec_pressing = false;
        keyStates[i].macro_exec_tick = 0;
        keyStates[i].calibrated = false;
        keyStates[i].calibration_samples = 0;
        keyStates[i].calibration_sum = 0;
        keyStates[i].is_active = false;
    }
    ledConfig.mode = DEFAULT_LED_MODE;
    ledConfig.brightness = DEFAULT_LED_BRIGHT;
    ledConfig.speed = DEFAULT_LED_SPEED;
    flash_status = 0;
}

void RapidTriggerKeyboard::updateRapidTriggerState(RapidTriggerState& state, uint32_t currentVal) {
    if (!state.calibrated) {
        if (state.calibration_samples == 0) {
            state.low_peak = currentVal;
            state.high_peak = currentVal;
            state.calibration_sum = 0;
        }

        if (currentVal < state.low_peak) state.low_peak = currentVal;
        if (currentVal > state.high_peak) state.high_peak = currentVal;

        state.calibration_sum += currentVal;
        state.calibration_samples++;

        if (state.calibration_samples >= INITIAL_CALIBRATION_SAMPLES) {
            state.baseline = state.calibration_sum / state.calibration_samples;
            state.low_peak = state.baseline;
            state.high_peak = state.baseline;
            state.calibrated = true;
            state.is_active = false;
            state.was_active = false;
        }
        return;
    }

    if (currentVal > state.high_peak) state.high_peak = currentVal;
    if (currentVal < state.low_peak) state.low_peak = currentVal;

    uint32_t activation_floor = state.baseline + state.dead_zone;
    uint32_t release_threshold = activation_floor + (state.sensitivity / 2);

    if (state.is_active) {
        bool fellFromPeak = (currentVal < (state.high_peak - state.sensitivity));
        bool fellBelowRelease = (currentVal <= release_threshold);

        if (fellFromPeak || fellBelowRelease) {
            state.is_active = false;
            state.low_peak = currentVal;
            state.high_peak = currentVal;
        } else if (currentVal > state.high_peak) {
            state.high_peak = currentVal;
        }
    } else {
        bool movedEnough = (currentVal > (state.low_peak + state.sensitivity));
        bool aboveBaseline = (currentVal > activation_floor);

        if (movedEnough && aboveBaseline) {
            state.is_active = true;
            state.high_peak = currentVal;
        }

        if (currentVal <= activation_floor) {
            state.is_active = false;
            state.high_peak = currentVal;
            state.low_peak = currentVal;

            // キーが完全にオフ(デッドゾーン以下)の時のみ、温度ドリフト等のためベースラインを追従
            int32_t diff = (int32_t)currentVal - (int32_t)state.baseline;
            state.baseline += (diff / 32);
        }
    }
}

#define MACRO_PRESS_MS   20
#define MACRO_RELEASE_MS 10

// 修飾キーコードをMODIFIERビットに変換するヘルパー
static inline void applyKeyToReport(KeyboardReport& report, uint8_t code) {
    if (code >= 0xE0 && code <= 0xE7) {
        // 修飾キー → MODIFIER byte
        report.MODIFIER |= (1 << (code - 0xE0));
    } else if (code > 0 && code < 120) {
        // 通常キー → KEYS bitmap
        report.KEYS[code / 8] |= (1 << (code % 8));
    }
}

KeyboardReport* RapidTriggerKeyboard::getReport() {
    memset(&report, 0, sizeof(KeyboardReport));
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        RapidTriggerState& ks = keyStates[i];

        if (ks.macro_step_count > 0) {
            // === マクロシーケンス実行 ===
            if (ks.is_active && !ks.was_active) {
                ks.macro_exec_step = 0;
                ks.macro_exec_pressing = true;
                ks.macro_exec_tick = now;
            }
            if (!ks.is_active && ks.was_active) {
                ks.macro_exec_step = ks.macro_step_count;
            }
            ks.was_active = ks.is_active;

            if (ks.macro_exec_step < ks.macro_step_count) {
                if (ks.macro_exec_pressing) {
                    MacroStep& step = ks.macro_steps[ks.macro_exec_step];
                    report.MODIFIER |= step.modifiers;
                    applyKeyToReport(report, step.keycode);
                    if ((now - ks.macro_exec_tick) >= MACRO_PRESS_MS) {
                        ks.macro_exec_pressing = false;
                        ks.macro_exec_tick = now;
                    }
                } else {
                    if ((now - ks.macro_exec_tick) >= MACRO_RELEASE_MS) {
                        ks.macro_exec_step++;
                        if (ks.macro_exec_step < ks.macro_step_count) {
                            ks.macro_exec_pressing = true;
                            ks.macro_exec_tick = now;
                        }
                    }
                }
            }
        } else {
            // === 通常キーコード ===
            if (ks.is_active) {
                applyKeyToReport(report, ks.keycode);
            }
        }
    }

    return &report;
}
