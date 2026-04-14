#include "nkro.hpp"
#include "stm32g4xx_hal.h"
#include <stdio.h>

// ===== Flash設定 =====
// STM32G431KB: 128KB Flash, Bank 1, Page 2KB × 64
// 3ページ使用 (60-62): 6KB for 82 keys
#define FLASH_USER_START_ADDR   0x0801E000
#define FLASH_PAGE_START        60
#define FLASH_PAGE_COUNT        3
#define FLASH_MAGIC_NUMBER      0xC00F0009  // v9: split ON/OFF thresholds + combo macros

// デフォルト値
#define DEFAULT_SENSITIVITY_ON  50
#define DEFAULT_SENSITIVITY_OFF 100
#define DEFAULT_DEAD_ZONE    30
#define DEFAULT_LED_MODE     LED_MODE_FADE
#define DEFAULT_LED_BRIGHT   255
#define DEFAULT_LED_SPEED    50
#define INITIAL_CALIBRATION_SAMPLES 16

static const uint16_t DEFAULT_KEYCODES[RapidTriggerKeyboard::TOTAL_KEY_COUNT] = {
    0x39, 0xE1, 0xE0, 0xE3, 0x04, 0xE2, 0x1D, 0x16, 0x1A, 0x1F, 0x3A, 0x1E, 0x14, 0x29, 0x35, 0x2B, // Src 0 (16)
    0x07, 0x1B, 0x8B, 0x06, 0x2C, 0x09, 0x19, 0x22, 0x3D, 0x3C, 0x21, 0x15, 0x3B, 0x20, 0x08,       // Src 1 (15)
    0x0A, 0x05, 0x0B, 0x24, 0x1C, 0x3F, 0x23, 0x3E, 0x17,                                           // Src 2 (9)
    0x28, 0x2A, 0x4C, 0x89, 0x45, 0x30, 0x2E, 0x44, 0x34, 0x87, 0x50, 0x51, 0x52, 0x32, 0x4F, 0x00, // Src 3 (16)
    0x2F, 0x13, 0x2D, 0x43, 0x27, 0x42, 0x26, 0x12, 0x0F, 0x36, 0x8A, 0x88, 0x37, 0xE6, 0x38, 0x33, // Src 4 (16)
    0x0C, 0x41, 0x25, 0x40, 0x18, 0x0D, 0x11, 0x2C, 0x10, 0x0E                                      // Src 5 (10)
};

RapidTriggerKeyboard::RapidTriggerKeyboard() {
    flash_status = -1;
    memset(combos, 0, sizeof(combos));
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
        keyStates[keyCounter].sensitivity_on = DEFAULT_SENSITIVITY_ON;
        keyStates[keyCounter].sensitivity_off = DEFAULT_SENSITIVITY_OFF;
        keyStates[keyCounter].keycode = hidCode;
        keyStates[keyCounter].dead_zone = DEFAULT_DEAD_ZONE;
        keyStates[keyCounter].calibration_samples = 0;
        keyStates[keyCounter].calibration_sum = 0;
        keyStates[keyCounter].on_debounce_count = 0;
        keyStates[keyCounter].macro_step_count = 0;
        memset(keyStates[keyCounter].macro_steps, 0, sizeof(keyStates[keyCounter].macro_steps));
        keyStates[keyCounter].was_active = false;
        keyStates[keyCounter].macro_exec_step = 0;
        keyStates[keyCounter].macro_exec_pressing = false;
        keyStates[keyCounter].macro_exec_tick = 0;
        keyStates[keyCounter].macro_completed = false;
        keyStates[keyCounter].macro_held_modifiers = 0;
        memset(keyStates[keyCounter].macro_held_keys, 0, sizeof(keyStates[keyCounter].macro_held_keys));
        keyStates[keyCounter].macro_held_count = 0;

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
            keyStates[i].sensitivity_on = value;
            keyStates[i].sensitivity_off = value;
        }
    } else if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].sensitivity_on = value;
        keyStates[keyIndex].sensitivity_off = value;
    }
}

uint32_t RapidTriggerKeyboard::getSensitivity(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        // 旧API互換: ONしきい値を返す
        return keyStates[keyIndex].sensitivity_on;
    }
    return 0;
}

void RapidTriggerKeyboard::setOnThreshold(int keyIndex, uint32_t value) {
    if (value < 1) value = 1;
    if (value > 1000) value = 1000;

    if (keyIndex == 255 || keyIndex == -1) {
        for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
            keyStates[i].sensitivity_on = value;
        }
    } else if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].sensitivity_on = value;
    }
}

uint32_t RapidTriggerKeyboard::getOnThreshold(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].sensitivity_on;
    }
    return 0;
}

void RapidTriggerKeyboard::setOffThreshold(int keyIndex, uint32_t value) {
    if (value < 1) value = 1;
    if (value > 1000) value = 1000;

    if (keyIndex == 255 || keyIndex == -1) {
        for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
            keyStates[i].sensitivity_off = value;
        }
    } else if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        keyStates[keyIndex].sensitivity_off = value;
    }
}

uint32_t RapidTriggerKeyboard::getOffThreshold(int keyIndex) {
    if (keyIndex >= 0 && keyIndex < TOTAL_KEY_COUNT) {
        return keyStates[keyIndex].sensitivity_off;
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
            keyStates[keyIndex].macro_steps[s] = {0, 0, 0};
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

// ===== コンボマクロ =====
void RapidTriggerKeyboard::setCombo(int index, uint8_t trigMod, uint8_t trigKey, uint8_t stepCount, const MacroStep* steps) {
    if (index < 0 || index >= MAX_COMBOS) return;
    combos[index].trigger_modifiers = trigMod;
    combos[index].trigger_keycode = trigKey;
    if (stepCount > MAX_COMBO_STEPS) stepCount = MAX_COMBO_STEPS;
    combos[index].step_count = stepCount;
    for (int s = 0; s < stepCount; s++) combos[index].steps[s] = steps[s];
    for (int s = stepCount; s < MAX_COMBO_STEPS; s++) combos[index].steps[s] = {0, 0, 0};
    // Reset runtime
    combos[index].is_active = false;
    combos[index].was_active = false;
    combos[index].exec_step = 0;
    combos[index].completed = false;
    combos[index].held_modifiers = 0;
    combos[index].held_count = 0;
}
uint8_t RapidTriggerKeyboard::getComboTriggerMod(int index) {
    return (index >= 0 && index < MAX_COMBOS) ? combos[index].trigger_modifiers : 0;
}
uint8_t RapidTriggerKeyboard::getComboTriggerKey(int index) {
    return (index >= 0 && index < MAX_COMBOS) ? combos[index].trigger_keycode : 0;
}
uint8_t RapidTriggerKeyboard::getComboStepCount(int index) {
    return (index >= 0 && index < MAX_COMBOS) ? combos[index].step_count : 0;
}
const MacroStep* RapidTriggerKeyboard::getComboSteps(int index) {
    return (index >= 0 && index < MAX_COMBOS) ? combos[index].steps : nullptr;
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

uint32_t RapidTriggerKeyboard::getCalibBase(int keyIndex) {
    if (keyIndex < 0 || keyIndex >= TOTAL_KEY_COUNT) return 0;
    return keyStates[keyIndex].baseline;
}

uint32_t RapidTriggerKeyboard::getLowPeak(int keyIndex) {
    if (keyIndex < 0 || keyIndex >= TOTAL_KEY_COUNT) return 0;
    return keyStates[keyIndex].low_peak;
}

uint32_t RapidTriggerKeyboard::getHighPeak(int keyIndex) {
    if (keyIndex < 0 || keyIndex >= TOTAL_KEY_COUNT) return 0;
    return keyStates[keyIndex].high_peak;
}

// ===== Flash 保存 / 読み込み =====
// Layout v9 (3 pages: 60-62, 6KB total)
// [0]       Magic (8B)
// [1]       LED Config (8B)
// [2..]     KeyData × TOTAL_KEY_COUNT: each = 8 doublewords (64B)
//   DW0: sensitivity_on
//   DW1: sensitivity_off
//   DW2: dead_zone
//   DW3: keycode
//   DW4-DW7: macro_step_count + steps[0-7]

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

    // Write Key Data (8 DWs per key)
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        // DW0: sensitivity_on
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].sensitivity_on) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW1: sensitivity_off
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].sensitivity_off) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW2: dead_zone
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].dead_zone) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW3: keycode
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address,
                              (uint64_t)keyStates[i].keycode) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW4: macro step_count(8) + step[0](24) + step[1](24) = 56 bits
        uint64_t macro_dw3 = (uint64_t)keyStates[i].macro_step_count;
        for (int s = 0; s < 2 && s < MAX_MACRO_STEPS; s++) {
            int base = 8 + s * 24;
            macro_dw3 |= ((uint64_t)keyStates[i].macro_steps[s].action    << base);
            macro_dw3 |= ((uint64_t)keyStates[i].macro_steps[s].modifiers << (base + 8));
            macro_dw3 |= ((uint64_t)keyStates[i].macro_steps[s].keycode   << (base + 16));
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw3) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW5: step[2](24) + step[3](24) = 48 bits
        uint64_t macro_dw4 = 0;
        for (int s = 0; s < 2; s++) {
            int si = s + 2;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                macro_dw4 |= ((uint64_t)keyStates[i].macro_steps[si].action    << base);
                macro_dw4 |= ((uint64_t)keyStates[i].macro_steps[si].modifiers << (base + 8));
                macro_dw4 |= ((uint64_t)keyStates[i].macro_steps[si].keycode   << (base + 16));
            }
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw4) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW6: step[4](24) + step[5](24) = 48 bits
        uint64_t macro_dw5 = 0;
        for (int s = 0; s < 2; s++) {
            int si = s + 4;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                macro_dw5 |= ((uint64_t)keyStates[i].macro_steps[si].action    << base);
                macro_dw5 |= ((uint64_t)keyStates[i].macro_steps[si].modifiers << (base + 8));
                macro_dw5 |= ((uint64_t)keyStates[i].macro_steps[si].keycode   << (base + 16));
            }
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw5) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;

        // DW7: step[6](24) + step[7](24) = 48 bits
        uint64_t macro_dw6 = 0;
        for (int s = 0; s < 2; s++) {
            int si = s + 6;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                macro_dw6 |= ((uint64_t)keyStates[i].macro_steps[si].action    << base);
                macro_dw6 |= ((uint64_t)keyStates[i].macro_steps[si].modifiers << (base + 8));
                macro_dw6 |= ((uint64_t)keyStates[i].macro_steps[si].keycode   << (base + 16));
            }
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, macro_dw6) != HAL_OK) {
            flash_status = 3; flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i; HAL_FLASH_Lock(); return;
        }
        address += 8;
    }

    // ===== Combo data (8 combos x 5 DW) =====
    for (int c = 0; c < MAX_COMBOS; c++) {
        // DW1: header (trigger_mod + trigger_key + step_count + pad)
        uint64_t hdr = (uint64_t)combos[c].trigger_modifiers |
                       ((uint64_t)combos[c].trigger_keycode << 8) |
                       ((uint64_t)combos[c].step_count << 16);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, hdr) != HAL_OK) {
            flash_status = 3; HAL_FLASH_Lock(); return;
        }
        address += 8;
        // DW2-DW5: steps (2 per DW)
        for (int dw = 0; dw < 4; dw++) {
            uint64_t val = 0;
            for (int s = 0; s < 2; s++) {
                int si = dw * 2 + s;
                if (si < MAX_COMBO_STEPS) {
                    int base = s * 24;
                    val |= ((uint64_t)combos[c].steps[si].action    << base);
                    val |= ((uint64_t)combos[c].steps[si].modifiers << (base + 8));
                    val |= ((uint64_t)combos[c].steps[si].keycode   << (base + 16));
                }
            }
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, val) != HAL_OK) {
                flash_status = 3; HAL_FLASH_Lock(); return;
            }
            address += 8;
        }
    }

    HAL_FLASH_Lock();
    printf("[FLASH] Saved %d keys + %d combos\r\n", TOTAL_KEY_COUNT, MAX_COMBOS);
}

void RapidTriggerKeyboard::loadFromFlash() {
    uint32_t address = FLASH_USER_START_ADDR;

    uint32_t magic = *(__IO uint32_t*)address;
    if (magic != FLASH_MAGIC_NUMBER) {
        printf("[FLASH] No valid v9 data (magic=0x%08lX)\r\n", magic);
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
        uint32_t sens_on = *(__IO uint32_t*)address;
        if (sens_on >= 1 && sens_on <= 1000) keyStates[i].sensitivity_on = sens_on;
        address += 8;

        uint32_t sens_off = *(__IO uint32_t*)address;
        if (sens_off >= 1 && sens_off <= 1000) keyStates[i].sensitivity_off = sens_off;
        address += 8;

        uint32_t dz = *(__IO uint32_t*)address;
        if (dz <= 500) keyStates[i].dead_zone = dz;
        address += 8;

        uint32_t kc = *(__IO uint32_t*)address;
        // 0x0000-0x00E7: keyboard, 0x8000+: consumer keys
        if (kc <= 0xFFFF) keyStates[i].keycode = (uint16_t)kc;
        address += 8;

        // DW4: macro step_count(8) + step[0-1]
        uint64_t macro_dw3 = *(__IO uint64_t*)address;
        uint8_t sc = macro_dw3 & 0xFF;
        if (sc <= MAX_MACRO_STEPS) {
            keyStates[i].macro_step_count = sc;
            for (int s = 0; s < 2 && s < MAX_MACRO_STEPS; s++) {
                int base = 8 + s * 24;
                keyStates[i].macro_steps[s].action    = (macro_dw3 >> base) & 0xFF;
                keyStates[i].macro_steps[s].modifiers = (macro_dw3 >> (base + 8)) & 0xFF;
                keyStates[i].macro_steps[s].keycode   = (macro_dw3 >> (base + 16)) & 0xFF;
            }
        }
        address += 8;

        // DW5: step[2-3]
        uint64_t macro_dw4 = *(__IO uint64_t*)address;
        for (int s = 0; s < 2; s++) {
            int si = s + 2;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                keyStates[i].macro_steps[si].action    = (macro_dw4 >> base) & 0xFF;
                keyStates[i].macro_steps[si].modifiers = (macro_dw4 >> (base + 8)) & 0xFF;
                keyStates[i].macro_steps[si].keycode   = (macro_dw4 >> (base + 16)) & 0xFF;
            }
        }
        address += 8;

        // DW6: step[4-5]
        uint64_t macro_dw5 = *(__IO uint64_t*)address;
        for (int s = 0; s < 2; s++) {
            int si = s + 4;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                keyStates[i].macro_steps[si].action    = (macro_dw5 >> base) & 0xFF;
                keyStates[i].macro_steps[si].modifiers = (macro_dw5 >> (base + 8)) & 0xFF;
                keyStates[i].macro_steps[si].keycode   = (macro_dw5 >> (base + 16)) & 0xFF;
            }
        }
        address += 8;

        // DW7: step[6-7]
        uint64_t macro_dw6 = *(__IO uint64_t*)address;
        for (int s = 0; s < 2; s++) {
            int si = s + 6;
            if (si < MAX_MACRO_STEPS) {
                int base = s * 24;
                keyStates[i].macro_steps[si].action    = (macro_dw6 >> base) & 0xFF;
                keyStates[i].macro_steps[si].modifiers = (macro_dw6 >> (base + 8)) & 0xFF;
                keyStates[i].macro_steps[si].keycode   = (macro_dw6 >> (base + 16)) & 0xFF;
            }
        }
        address += 8;
    }

    // ===== Combo data =====
    for (int c = 0; c < MAX_COMBOS; c++) {
        uint64_t hdr = *(__IO uint64_t*)address;
        combos[c].trigger_modifiers = hdr & 0xFF;
        combos[c].trigger_keycode = (hdr >> 8) & 0xFF;
        combos[c].step_count = (hdr >> 16) & 0xFF;
        if (combos[c].step_count > MAX_COMBO_STEPS) combos[c].step_count = 0;
        address += 8;
        for (int dw = 0; dw < 4; dw++) {
            uint64_t val = *(__IO uint64_t*)address;
            for (int s = 0; s < 2; s++) {
                int si = dw * 2 + s;
                if (si < MAX_COMBO_STEPS) {
                    int base = s * 24;
                    combos[c].steps[si].action    = (val >> base) & 0xFF;
                    combos[c].steps[si].modifiers = (val >> (base + 8)) & 0xFF;
                    combos[c].steps[si].keycode   = (val >> (base + 16)) & 0xFF;
                }
            }
            address += 8;
        }
        combos[c].is_active = false;
        combos[c].was_active = false;
    }

    printf("[FLASH] Loaded %d keys + %d combos\r\n", TOTAL_KEY_COUNT, MAX_COMBOS);
}

void RapidTriggerKeyboard::resetDefaults() {
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        keyStates[i].sensitivity_on = DEFAULT_SENSITIVITY_ON;
        keyStates[i].sensitivity_off = DEFAULT_SENSITIVITY_OFF;
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
        keyStates[i].on_debounce_count = 0;
        keyStates[i].is_active = false;
    }
    ledConfig.mode = DEFAULT_LED_MODE;
    ledConfig.brightness = DEFAULT_LED_BRIGHT;
    ledConfig.speed = DEFAULT_LED_SPEED;
    // コンボクリア
    for (int c = 0; c < MAX_COMBOS; c++) {
        combos[c].trigger_modifiers = 0;
        combos[c].trigger_keycode = 0;
        combos[c].step_count = 0;
        memset(combos[c].steps, 0, sizeof(combos[c].steps));
        combos[c].is_active = false;
        combos[c].was_active = false;
    }
    flash_status = 0;
}

void RapidTriggerKeyboard::updateRapidTriggerState(RapidTriggerState& state, uint32_t currentVal) {
    if (!state.calibrated) {
        // Phase 1: 最初の16サンプルは捨てる (ADC/MUXセトリング待ち)
        // Phase 2: 次の16サンプルで平均 → ベースライン
        const uint16_t DISCARD_SAMPLES = 16;
        const uint16_t CALIB_SAMPLES = INITIAL_CALIBRATION_SAMPLES; // 16

        if (state.calibration_samples < DISCARD_SAMPLES) {
            // 捨てるフェーズ: カウントだけ進める
            state.calibration_samples++;
            return;
        }

        // 蓄積フェーズ
        uint16_t real_idx = state.calibration_samples - DISCARD_SAMPLES;
        if (real_idx == 0) {
            state.low_peak = currentVal;
            state.high_peak = currentVal;
            state.calibration_sum = 0;
        }

        if (currentVal < state.low_peak) state.low_peak = currentVal;
        if (currentVal > state.high_peak) state.high_peak = currentVal;

        state.calibration_sum += currentVal;
        state.calibration_samples++;

        if (real_idx + 1 >= CALIB_SAMPLES) {
            state.baseline = state.calibration_sum / CALIB_SAMPLES;
            state.low_peak = state.baseline;
            state.high_peak = state.baseline;
            
            // 以降の追従のために、calibration_sum を高精度アキュムレータとして再利用 (係数1024)
            state.calibration_sum = state.baseline * 1024;

            state.calibrated = true;
            state.is_active = false;
            state.was_active = false;
            state.on_debounce_count = 0;
        }
        return;
    }

    if (currentVal > state.high_peak) state.high_peak = currentVal;
    if (currentVal < state.low_peak) state.low_peak = currentVal;

    uint32_t activation_floor = state.baseline + state.dead_zone;
    uint32_t release_threshold = activation_floor + (state.sensitivity_off / 2);

    if (state.is_active) {
        bool fellFromPeak = (currentVal < (state.high_peak - state.sensitivity_off));
        bool fellBelowRelease = (currentVal <= release_threshold);

        if (fellFromPeak || fellBelowRelease) {
            state.is_active = false;
            state.low_peak = currentVal;
            state.high_peak = currentVal;
            state.on_debounce_count = 0;
        } else if (currentVal > state.high_peak) {
            state.high_peak = currentVal;
        }
    } else {
        bool movedEnough = (currentVal > (state.low_peak + state.sensitivity_on));
        bool aboveBaseline = (currentVal > activation_floor);

        if (movedEnough && aboveBaseline) {
            if (state.on_debounce_count < 0xFF) state.on_debounce_count++;
            if (state.on_debounce_count >= 2) {
                state.is_active = true;
                state.high_peak = currentVal;
                state.on_debounce_count = 0;
            }
        } else {
            state.on_debounce_count = 0;
        }

        if (currentVal <= activation_floor) {
            state.is_active = false;
            state.high_peak = currentVal;
            state.low_peak = currentVal;
            state.on_debounce_count = 0;

            // キーが完全にオフ(デッドゾーン以下)の時のみ、温度や電源変動などにベースラインを追従させる
            // 整数丸めによる停止を防ぐため、1024倍精度のEMAフィルタを適用する
            state.calibration_sum = state.calibration_sum - (state.calibration_sum / 1024) + currentVal;
            state.baseline = state.calibration_sum / 1024;
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

            // 押し始め: シーケンス開始
            if (ks.is_active && !ks.was_active) {
                ks.macro_exec_step = 0;
                ks.macro_exec_pressing = true;
                ks.macro_exec_tick = now;
                ks.macro_completed = false;
                ks.macro_held_modifiers = 0;
                ks.macro_held_count = 0;
                memset(ks.macro_held_keys, 0, sizeof(ks.macro_held_keys));
            }

            // 離した: 保持状態をクリア
            if (!ks.is_active && ks.was_active) {
                ks.macro_exec_step = ks.macro_step_count; // ステップ実行停止
                ks.macro_completed = false;
                ks.macro_held_modifiers = 0;
                ks.macro_held_count = 0;
            }
            ks.was_active = ks.is_active;

            // ステップ実行中
            if (ks.macro_exec_step < ks.macro_step_count && !ks.macro_completed) {
                if (ks.macro_exec_pressing) {
                    MacroStep& step = ks.macro_steps[ks.macro_exec_step];

                    if (step.action == MACRO_ACTION_PRESS) {
                        // PRESS: キーを保持リストに追加
                        ks.macro_held_modifiers |= step.modifiers;
                        if (step.keycode > 0 && ks.macro_held_count < 6) {
                            ks.macro_held_keys[ks.macro_held_count++] = step.keycode;
                        }
                    } else if (step.action == MACRO_ACTION_RELEASE) {
                        // RELEASE: 指定キーを保持リストから削除
                        ks.macro_held_modifiers &= ~step.modifiers;
                        for (int k = 0; k < ks.macro_held_count; k++) {
                            if (ks.macro_held_keys[k] == step.keycode) {
                                // 削除: 後ろを詰める
                                for (int j = k; j < ks.macro_held_count - 1; j++) {
                                    ks.macro_held_keys[j] = ks.macro_held_keys[j+1];
                                }
                                ks.macro_held_count--;
                                ks.macro_held_keys[ks.macro_held_count] = 0;
                                break;
                            }
                        }
                    }

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
                        } else {
                            // 全ステップ完了
                            // RELEASEステップがあればループ、なければ保持
                            bool hasRelease = false;
                            for (int s = 0; s < ks.macro_step_count; s++) {
                                if (ks.macro_steps[s].action == MACRO_ACTION_RELEASE) {
                                    hasRelease = true;
                                    break;
                                }
                            }
                            if (hasRelease) {
                                // ループ: ステップ0に戻る
                                ks.macro_exec_step = 0;
                                ks.macro_exec_pressing = true;
                                ks.macro_exec_tick = now;
                                ks.macro_held_modifiers = 0;
                                ks.macro_held_count = 0;
                                memset(ks.macro_held_keys, 0, sizeof(ks.macro_held_keys));
                            } else {
                                // 押しっぱなし: 保持状態を継続
                                ks.macro_completed = true;
                            }
                        }
                    }
                }
            }

            // 保持中のキーをレポートに反映 (実行中 + 完了後も物理キーが押されている限り)
            if (ks.is_active) {
                report.MODIFIER |= ks.macro_held_modifiers;
                for (int k = 0; k < ks.macro_held_count; k++) {
                    applyKeyToReport(report, ks.macro_held_keys[k]);
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

    // ===== コンボマクロ処理 =====
    // 物理的に押されている修飾キーを取得
    uint8_t physicalModifiers = 0;
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        if (keyStates[i].is_active) {
            uint16_t kc = keyStates[i].keycode;
            if (kc >= 0xE0 && kc <= 0xE7) {
                physicalModifiers |= (1 << (kc - 0xE0));
            }
        }
    }

    for (int c = 0; c < MAX_COMBOS; c++) {
        ComboMacro& combo = combos[c];
        if (combo.step_count == 0) continue;

        // トリガー判定
        bool triggerActive = true;
        // 修飾キーチェック
        if (combo.trigger_modifiers != 0 &&
            (physicalModifiers & combo.trigger_modifiers) != combo.trigger_modifiers) {
            triggerActive = false;
        }
        // 通常キーチェック
        if (triggerActive && combo.trigger_keycode > 0) {
            bool found = false;
            for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
                if (keyStates[i].keycode == combo.trigger_keycode && keyStates[i].is_active) {
                    found = true;
                    break;
                }
            }
            if (!found) triggerActive = false;
        }

        combo.is_active = triggerActive;

        // 状態遷移
        if (combo.is_active && !combo.was_active) {
            combo.exec_step = 0;
            combo.exec_pressing = true;
            combo.exec_tick = now;
            combo.completed = false;
            combo.held_modifiers = 0;
            combo.held_count = 0;
            memset(combo.held_keys, 0, sizeof(combo.held_keys));
        }
        if (!combo.is_active && combo.was_active) {
            combo.exec_step = combo.step_count;
            combo.completed = false;
            combo.held_modifiers = 0;
            combo.held_count = 0;
        }
        combo.was_active = combo.is_active;

        if (combo.is_active) {
            // トリガーキーをレポートから除去
            report.MODIFIER &= ~combo.trigger_modifiers;
            if (combo.trigger_keycode > 0 && combo.trigger_keycode < 140) {
                report.KEYS[combo.trigger_keycode / 8] &= ~(1 << (combo.trigger_keycode % 8));
            }

            // ステップ実行
            if (combo.exec_step < combo.step_count && !combo.completed) {
                if (combo.exec_pressing) {
                    MacroStep& step = combo.steps[combo.exec_step];
                    if (step.action == MACRO_ACTION_PRESS) {
                        combo.held_modifiers |= step.modifiers;
                        if (step.keycode > 0 && combo.held_count < 6) {
                            combo.held_keys[combo.held_count++] = step.keycode;
                        }
                    } else if (step.action == MACRO_ACTION_RELEASE) {
                        combo.held_modifiers &= ~step.modifiers;
                        for (int k = 0; k < combo.held_count; k++) {
                            if (combo.held_keys[k] == step.keycode) {
                                for (int j = k; j < combo.held_count - 1; j++)
                                    combo.held_keys[j] = combo.held_keys[j+1];
                                combo.held_count--;
                                combo.held_keys[combo.held_count] = 0;
                                break;
                            }
                        }
                    }
                    if ((now - combo.exec_tick) >= MACRO_PRESS_MS) {
                        combo.exec_pressing = false;
                        combo.exec_tick = now;
                    }
                } else {
                    if ((now - combo.exec_tick) >= MACRO_RELEASE_MS) {
                        combo.exec_step++;
                        if (combo.exec_step < combo.step_count) {
                            combo.exec_pressing = true;
                            combo.exec_tick = now;
                        } else {
                            // 完了: ループ or ホールド判定
                            bool hasRelease = false;
                            for (int s = 0; s < combo.step_count; s++) {
                                if (combo.steps[s].action == MACRO_ACTION_RELEASE) {
                                    hasRelease = true; break;
                                }
                            }
                            if (hasRelease) {
                                combo.exec_step = 0;
                                combo.exec_pressing = true;
                                combo.exec_tick = now;
                                combo.held_modifiers = 0;
                                combo.held_count = 0;
                                memset(combo.held_keys, 0, sizeof(combo.held_keys));
                            } else {
                                combo.completed = true;
                            }
                        }
                    }
                }
            }

            // 保持キーをレポートに反映
            report.MODIFIER |= combo.held_modifiers;
            for (int k = 0; k < combo.held_count; k++) {
                applyKeyToReport(report, combo.held_keys[k]);
            }
        }
    }

    return &report;
}

uint16_t RapidTriggerKeyboard::getConsumerKey() {
    return activeConsumerKey;
}
