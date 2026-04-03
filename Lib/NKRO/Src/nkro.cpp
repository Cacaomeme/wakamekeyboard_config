#include "nkro.hpp"
#include "stm32g4xx_hal.h"
#include <stdio.h>

// ===== Flash設定 =====
// STM32G431KB: 128KB Flash, Bank 1, Page 2KB × 64
// 3ページ使用 (60-62): 6KB for 82 keys
#define FLASH_USER_START_ADDR   0x0801E000
#define FLASH_PAGE_START        60
#define FLASH_PAGE_COUNT        3
#define FLASH_MAGIC_NUMBER      0xC00F0006  // v6: 70% JIS keyboard

// デフォルト値
#define DEFAULT_SENSITIVITY  50
#define DEFAULT_DEAD_ZONE    30
#define DEFAULT_LED_MODE     LED_MODE_FADE
#define DEFAULT_LED_BRIGHT   255
#define DEFAULT_LED_SPEED    50
#define INITIAL_CALIBRATION_SAMPLES 16

// デフォルトキーコード (6 source × 16 ch = 96 entries, NC=0x00)
// 0x0000-0x00E7 = 標準キーボード, 0x8xxx = Consumer Control
static const uint16_t DEFAULT_KEYCODES[96] = {
    // Source 0: PB0 (ADC1_IN15) MUX 0-15: 16 keys (all connected)
    0x39, // 0:  CapsLock
    0xE1, // 1:  Left Shift
    0xE0, // 2:  Left Ctrl
    0xE3, // 3:  Left Win (GUI)
    0x04, // 4:  A
    0xE2, // 5:  Left Alt
    0x1D, // 6:  Z
    0x16, // 7:  S
    0x1A, // 8:  W
    0x1F, // 9:  2
    0x3A, // 10: F1
    0x1E, // 11: 1
    0x14, // 12: Q
    0x29, // 13: Esc
    0x35, // 14: 半角全角 (Grave)
    0x2B, // 15: Tab

    // Source 1: PA7 (ADC2_IN4) MUX 0-15: 15 connected, 1 NC
    0x07, // 16: D
    0x1B, // 17: X
    0x8B, // 18: 無変換 (Muhenkan/International5)
    0x06, // 19: C
    0x2C, // 20: L_Space
    0x09, // 21: F
    0x19, // 22: V
    0x00, // 23: NC
    0x22, // 24: 5
    0x3D, // 25: F4
    0x3C, // 26: F3
    0x21, // 27: 4
    0x15, // 28: R
    0x3B, // 29: F2
    0x20, // 30: 3
    0x08, // 31: E

    // Source 2: PA6 (ADC2_IN3) MUX 0-15: 9 connected, 7 NC
    0x0A, // 32: G
    0x00, // 33: NC
    0x00, // 34: NC
    0x05, // 35: B
    0x00, // 36: NC
    0x00, // 37: NC
    0x00, // 38: NC
    0x0B, // 39: H
    0x24, // 40: 7
    0x1C, // 41: Y
    0x3F, // 42: F6
    0x00, // 43: NC
    0x23, // 44: 6
    0x00, // 45: NC
    0x3E, // 46: F5
    0x17, // 47: T

    // Source 3: PA1 (ADC1_IN2) MUX 0-15: 16 keys (all connected)
    0x28, // 48: Enter
    0x2A, // 49: BackSpace
    0x4C, // 50: Delete
    0x89, // 51: ￥ (Yen/International3)
    0x45, // 52: F12
    0x30, // 53: [ (「)
    0x2E, // 54: ^ (Equal position)
    0x44, // 55: F11
    0x34, // 56: : (Apostrophe position)
    0x87, // 57: \\ (Ro/International1)
    0x50, // 58: ←
    0x51, // 59: ↓
    0x52, // 60: ↑
    0x32, // 61: ] (」/Non-US Hash)
    0x4F, // 62: →
    0x00, // 63: User Key (unassigned)

    // Source 4: PA0 (ADC1_IN1) MUX 0-15: 16 keys (all connected)
    0x2F, // 64: @ (Left Bracket position)
    0x13, // 65: P
    0x2D, // 66: -
    0x43, // 67: F10
    0x27, // 68: 0
    0x42, // 69: F9
    0x26, // 70: 9
    0x12, // 71: O
    0x0F, // 72: L
    0x36, // 73: , (Comma)
    0x8A, // 74: 変換 (Henkan/International4)
    0x88, // 75: かな (Kana/International2)
    0x37, // 76: . (Period)
    0xE6, // 77: Right Alt
    0x38, // 78: / (Slash)
    0x33, // 79: ; (Semicolon)

    // Source 5: PF1 (ADC2_IN10) MUX 0-15: 10 connected, 6 NC
    0x0C, // 80: I
    0x41, // 81: F8
    0x25, // 82: 8
    0x40, // 83: F7
    0x00, // 84: NC
    0x18, // 85: U
    0x00, // 86: NC
    0x00, // 87: NC
    0x00, // 88: NC
    0x00, // 89: NC
    0x00, // 90: NC
    0x0D, // 91: J
    0x11, // 92: N
    0x2C, // 93: R_Space
    0x10, // 94: M
    0x0E, // 95: K
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

    auto registerKey = [&](int source, int muxChannel, uint16_t hidCode) {
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

    // === Source 0: PB0 (ADC1_IN15) - 16 keys, all connected ===
    registerKey(0, 0,  0x39); // CapsLock
    registerKey(0, 1,  0xE1); // Left Shift
    registerKey(0, 2,  0xE0); // Left Ctrl
    registerKey(0, 3,  0xE3); // Left Win
    registerKey(0, 4,  0x04); // A
    registerKey(0, 5,  0xE2); // Left Alt
    registerKey(0, 6,  0x1D); // Z
    registerKey(0, 7,  0x16); // S
    registerKey(0, 8,  0x1A); // W
    registerKey(0, 9,  0x1F); // 2
    registerKey(0, 10, 0x3A); // F1
    registerKey(0, 11, 0x1E); // 1
    registerKey(0, 12, 0x14); // Q
    registerKey(0, 13, 0x29); // Esc
    registerKey(0, 14, 0x35); // 半角全角
    registerKey(0, 15, 0x2B); // Tab

    // === Source 1: PA7 (ADC2_IN4) - 15 connected, skip MUX7 ===
    registerKey(1, 0,  0x07); // D
    registerKey(1, 1,  0x1B); // X
    registerKey(1, 2,  0x8B); // 無変換
    registerKey(1, 3,  0x06); // C
    registerKey(1, 4,  0x2C); // L_Space
    registerKey(1, 5,  0x09); // F
    registerKey(1, 6,  0x19); // V
    // MUX 7: NC
    registerKey(1, 8,  0x22); // 5
    registerKey(1, 9,  0x3D); // F4
    registerKey(1, 10, 0x3C); // F3
    registerKey(1, 11, 0x21); // 4
    registerKey(1, 12, 0x15); // R
    registerKey(1, 13, 0x3B); // F2
    registerKey(1, 14, 0x20); // 3
    registerKey(1, 15, 0x08); // E

    // === Source 2: PA6 (ADC2_IN3) - 9 connected, many NC ===
    registerKey(2, 0,  0x0A); // G
    // MUX 1,2: NC
    registerKey(2, 3,  0x05); // B
    // MUX 4,5,6: NC
    registerKey(2, 7,  0x0B); // H
    registerKey(2, 8,  0x24); // 7
    registerKey(2, 9,  0x1C); // Y
    registerKey(2, 10, 0x3F); // F6
    // MUX 11: NC
    registerKey(2, 12, 0x23); // 6
    // MUX 13: NC
    registerKey(2, 14, 0x3E); // F5
    registerKey(2, 15, 0x17); // T

    // === Source 3: PA1 (ADC1_IN2) - 16 keys, all connected ===
    registerKey(3, 0,  0x28); // Enter
    registerKey(3, 1,  0x2A); // BackSpace
    registerKey(3, 2,  0x4C); // Delete
    registerKey(3, 3,  0x89); // ￥ (Yen)
    registerKey(3, 4,  0x45); // F12
    registerKey(3, 5,  0x30); // [ (「)
    registerKey(3, 6,  0x2E); // ^
    registerKey(3, 7,  0x44); // F11
    registerKey(3, 8,  0x34); // :
    registerKey(3, 9,  0x87); // \ (Ro)
    registerKey(3, 10, 0x50); // ←
    registerKey(3, 11, 0x51); // ↓
    registerKey(3, 12, 0x52); // ↑
    registerKey(3, 13, 0x32); // ] (」)
    registerKey(3, 14, 0x4F); // →
    registerKey(3, 15, 0x00); // User Key

    // === Source 4: PA0 (ADC1_IN1) - 16 keys, all connected ===
    registerKey(4, 0,  0x2F); // @
    registerKey(4, 1,  0x13); // P
    registerKey(4, 2,  0x2D); // -
    registerKey(4, 3,  0x43); // F10
    registerKey(4, 4,  0x27); // 0
    registerKey(4, 5,  0x42); // F9
    registerKey(4, 6,  0x26); // 9
    registerKey(4, 7,  0x12); // O
    registerKey(4, 8,  0x0F); // L
    registerKey(4, 9,  0x36); // ,
    registerKey(4, 10, 0x8A); // 変換
    registerKey(4, 11, 0x88); // かな
    registerKey(4, 12, 0x37); // .
    registerKey(4, 13, 0xE6); // Right Alt
    registerKey(4, 14, 0x38); // /
    registerKey(4, 15, 0x33); // ;

    // === Source 5: PF1 (ADC2_IN10) - 10 connected, skip MUX 4,6-10 ===
    registerKey(5, 0,  0x0C); // I
    registerKey(5, 1,  0x41); // F8
    registerKey(5, 2,  0x25); // 8
    registerKey(5, 3,  0x40); // F7
    // MUX 4: NC
    registerKey(5, 5,  0x18); // U
    // MUX 6,7,8,9,10: NC
    registerKey(5, 11, 0x0D); // J
    registerKey(5, 12, 0x11); // N
    registerKey(5, 13, 0x2C); // R_Space
    registerKey(5, 14, 0x10); // M
    registerKey(5, 15, 0x0E); // K

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

void RapidTriggerKeyboard::setKeycode(int keyIndex, uint16_t code) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].keycode = code;
    }
}

uint16_t RapidTriggerKeyboard::getKeycode(int keyIndex) {
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
        // 0x0000-0x00E7: keyboard, 0x8000+: consumer keys
        if (kc <= 0xFFFF) keyStates[i].keycode = (uint16_t)kc;
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
    } else if (code > 0 && code < 140) {
        // 通常キー → KEYS bitmap (0x01-0x8B)
        report.KEYS[code / 8] |= (1 << (code % 8));
    }
}

KeyboardReport* RapidTriggerKeyboard::getReport() {
    memset(&report, 0, sizeof(KeyboardReport));
    activeConsumerKey = 0;
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
                uint16_t code = ks.keycode;
                if (IS_CONSUMER_KEY(code)) {
                    // Consumer キー → consumer レポートへ
                    activeConsumerKey = CONSUMER_USAGE(code);
                } else {
                    // 標準キーボードキー
                    applyKeyToReport(report, (uint8_t)code);
                }
            }
        }
    }

    return &report;
}

uint16_t RapidTriggerKeyboard::getConsumerKey() {
    return activeConsumerKey;
}
