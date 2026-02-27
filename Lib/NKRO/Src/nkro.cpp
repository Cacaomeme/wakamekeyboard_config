#include "nkro.hpp"
#include "stm32g4xx_hal.h" // HALライブラリ (Flash操作用)
#include <stdio.h>

// Flash設定
// STM32G431KB = 128KB Flash, Bank 1, Page Size = 2KB, 64 Pages (0-63)
#define FLASH_USER_START_ADDR   0x0801F000
#define FLASH_PAGE_INDEX        62
#define FLASH_MAGIC_NUMBER      0xC00F0002  // v2: dead_zone + keycode + LED対応

// デフォルト値
#define DEFAULT_SENSITIVITY  50
#define DEFAULT_DEAD_ZONE    30
#define DEFAULT_LED_MODE     LED_MODE_FADE
#define DEFAULT_LED_BRIGHT   255
#define DEFAULT_LED_SPEED    50  // 5.0 (x10表記)

// デフォルトキーコード (init順: KP0, KP., Enter, 3, 2, 1, 6, 5, 4, +, 9, 8, 7, -, *, /, NumLock)
static const uint8_t DEFAULT_KEYCODES[17] = {
    0x62, 0x63, 0x58, 0x5B, 0x5A, 0x59, 0x5E, 0x5D, 0x5C,
    0x57, 0x61, 0x60, 0x5F, 0x56, 0x55, 0x54, 0x53
};

RapidTriggerKeyboard::RapidTriggerKeyboard() {
    flash_status = -1;
    init();
}

void RapidTriggerKeyboard::init() {
    // LED設定初期化
    ledConfig.mode = DEFAULT_LED_MODE;
    ledConfig.brightness = DEFAULT_LED_BRIGHT;
    ledConfig.speed = DEFAULT_LED_SPEED;

    // 1. マッピング配列の初期化
    for (int src = 0; src < 2; src++) {
        for (int i = 0; i < MUX_CH_COUNT; i++) {
            keyMapping[src][i] = -1;
        }
    }

    // 2. キー状態の初期化
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

        keyCounter++;
    };

    // --- MUX1 (ADC1) ---
    registerKey(0, 0,  0x62); // Keypad 0
    registerKey(0, 15, 0x63); // Keypad .
    registerKey(0, 14, 0x58); // Keypad Enter
    registerKey(0, 13, 0x5B); // Keypad 3
    registerKey(0, 12, 0x5A); // Keypad 2
    registerKey(0, 11, 0x59); // Keypad 1
    registerKey(0, 10, 0x5E); // Keypad 6
    registerKey(0, 9,  0x5D); // Keypad 5
    registerKey(0, 8,  0x5C); // Keypad 4
    
    // --- MUX2 (ADC2) ---
    registerKey(1, 15, 0x57); // Keypad +
    registerKey(1, 14, 0x61); // Keypad 9
    registerKey(1, 13, 0x60); // Keypad 8
    registerKey(1, 12, 0x5F); // Keypad 7
    registerKey(1, 11, 0x56); // Keypad -
    registerKey(1, 10, 0x55); // Keypad *
    registerKey(1, 9,  0x54); // Keypad /
    registerKey(1, 8,  0x53); // Keypad NumLock
}

void RapidTriggerKeyboard::updateKeyByMux(int muxIndex, int source, uint32_t adcValue) {
    if (source < 0 || source > 1 || muxIndex < 0 || muxIndex >= MUX_CH_COUNT) return;
    
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

// ===== Flash 保存 / 読み込み =====
// レイアウト v2:
// [0]  Magic (8B)
// [1]  LED Config: mode|brightness|speed|0 packed into uint64 (8B)
// [2..18] KeyData x17: each = 3 doublewords (24B)
//   DW0: sensitivity
//   DW1: dead_zone
//   DW2: keycode (lower 8 bits)
// Total: 8 + 8 + 17*24 = 424B (fits in 2KB page)

void RapidTriggerKeyboard::saveToFlash() {
    HAL_FLASH_Unlock();
    
    flash_status = 0; 
    flash_error_code = 0;

    // 1. Erase Page
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = FLASH_PAGE_INDEX;
    EraseInitStruct.NbPages = 1;

    uint32_t PageError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        flash_status = 1;
        flash_error_code = PageError;
        HAL_FLASH_Lock();
        return; 
    }

    uint32_t address = FLASH_USER_START_ADDR;

    // 2. Write Magic Number
    uint64_t magic_data = FLASH_MAGIC_NUMBER;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, magic_data) != HAL_OK) {
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

    // 3. Write LED Config (packed into uint64)
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

    // 4. Write Key Data (3 doublewords per key)
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        // DW0: sensitivity
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, 
                              (uint64_t)keyStates[i].sensitivity) != HAL_OK) {
            flash_status = 3;
            flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i;
            HAL_FLASH_Lock();
            return;
        }
        address += 8;

        // DW1: dead_zone
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, 
                              (uint64_t)keyStates[i].dead_zone) != HAL_OK) {
            flash_status = 3;
            flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i;
            HAL_FLASH_Lock();
            return;
        }
        address += 8;

        // DW2: keycode
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, 
                              (uint64_t)keyStates[i].keycode) != HAL_OK) {
            flash_status = 3;
            flash_error_code = HAL_FLASH_GetError();
            flash_debug_val = (uint32_t)i;
            HAL_FLASH_Lock();
            return;
        }
        address += 8;
    }

    HAL_FLASH_Lock();
}

void RapidTriggerKeyboard::loadFromFlash() {
    uint32_t address = FLASH_USER_START_ADDR;
    
    // Check Magic Number
    uint32_t magic = *(__IO uint32_t*)address;
    if (magic != FLASH_MAGIC_NUMBER) {
        return; // No valid v2 data
    }
    address += 8;

    // Read LED Config
    uint32_t led_raw = *(__IO uint32_t*)address;
    uint8_t m = led_raw & 0xFF;
    if (m < LED_MODE_COUNT) ledConfig.mode = (LedMode)m;
    uint8_t b = (led_raw >> 8) & 0xFF;
    ledConfig.brightness = b;
    uint8_t s = (led_raw >> 16) & 0xFF;
    if (s <= 100) ledConfig.speed = s;
    address += 8;

    // Read Key Data
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        uint32_t sens = *(__IO uint32_t*)address;
        if (sens >= 1 && sens <= 1000) {
            keyStates[i].sensitivity = sens;
        }
        address += 8;

        uint32_t dz = *(__IO uint32_t*)address;
        if (dz <= 500) {
            keyStates[i].dead_zone = dz;
        }
        address += 8;

        uint32_t kc = *(__IO uint32_t*)address;
        if (kc > 0 && kc < 120) {
            keyStates[i].keycode = (uint8_t)kc;
        }
        address += 8;
    }
}

void RapidTriggerKeyboard::resetDefaults() {
    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        keyStates[i].sensitivity = DEFAULT_SENSITIVITY;
        keyStates[i].dead_zone = DEFAULT_DEAD_ZONE;
        keyStates[i].keycode = DEFAULT_KEYCODES[i];
    }
    ledConfig.mode = DEFAULT_LED_MODE;
    ledConfig.brightness = DEFAULT_LED_BRIGHT;
    ledConfig.speed = DEFAULT_LED_SPEED;
    flash_status = 0;
}

void RapidTriggerKeyboard::updateRapidTriggerState(RapidTriggerState& state, uint32_t currentVal) {
    // 0. 初期化 (Calibration)
    if (!state.calibrated) {
        state.baseline = currentVal;
        state.low_peak = currentVal;
        state.high_peak = currentVal;
        state.calibrated = true;
        return;
    }

    // 磁気センサー: Idle(~2300) -> Pressed(~3200) (値が増加)
    
    // 1. ピークの更新
    if (currentVal > state.high_peak) {
        state.high_peak = currentVal;
    }
    if (currentVal < state.low_peak) {
        state.low_peak = currentVal;
    }
    
    // 2. ベースラインドリフト対策
    if (currentVal < state.baseline) {
        state.baseline = currentVal;
    }

    // デッドゾーン: baseline付近の不感帯 (設定可能、旧noise_margin)
    uint32_t activation_floor = state.baseline + state.dead_zone;

    if (state.is_active) {
        // ON → OFF判定: high_peakからsensitivity分戻ったらOFF
        if (currentVal < (state.high_peak - state.sensitivity)) {
            state.is_active = false;
            state.low_peak = currentVal; 
        }
    } else {
        // OFF → ON判定:
        // 1. low_peakからsensitivity分押したらON
        // 2. activation_floorを超えていること
        bool movedEnough = (currentVal > (state.low_peak + state.sensitivity));
        bool aboveBaseline = (currentVal > activation_floor); 

        if (movedEnough && aboveBaseline) {
            state.is_active = true;
            state.high_peak = currentVal; 
        }
        
        // ベースライン付近に戻ったら強制リセット
        if (currentVal <= activation_floor) {
             state.is_active = false;
             state.high_peak = currentVal; 
             state.low_peak = currentVal;
        }
    }
}

KeyboardReport* RapidTriggerKeyboard::getReport() {
    memset(&report, 0, sizeof(KeyboardReport));

    for (int i = 0; i < TOTAL_KEY_COUNT; i++) {
        if (keyStates[i].is_active) {
            uint8_t code = keyStates[i].keycode;
            
            if (code < 120) {
                int byteIndex = code / 8;
                int bitIndex  = code % 8;
                report.KEYS[byteIndex] |= (1 << bitIndex);
            }
        }
    }
    
    return &report;
}
