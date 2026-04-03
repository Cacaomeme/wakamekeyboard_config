#include "main.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "usb_device.h"
#include "usbd_customhid.h"
#include "tim.h"
#include "nkro.hpp"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
RapidTriggerKeyboard keyboard;

volatile uint32_t led_brightness = 0;
volatile bool usb_suspended = false;

volatile bool config_update_request = false;
volatile uint8_t config_target = 0;
volatile uint32_t config_val = 0;

volatile uint8_t last_received_cmd = 0;
volatile uint16_t last_received_len = 0;
volatile uint32_t usb_rx_cnt = 0;
volatile uint8_t usb_rx_err = 0;
volatile uint8_t last_payload[4] = {0};

// ===== Feature Report コマンドID =====
#define CMD_SET_SENSITIVITY  0x01
#define CMD_GET_SENSITIVITY  0x02
#define CMD_SET_KEYCODE      0x03
#define CMD_GET_KEYCODE      0x04
#define CMD_SET_DEADZONE     0x05
#define CMD_GET_DEADZONE     0x06
#define CMD_SAVE_TO_FLASH    0x10
#define CMD_RESET_DEFAULTS   0x11
#define CMD_GET_KEY_COUNT    0x20
#define CMD_SET_LED_MODE     0x30
#define CMD_GET_LED_MODE     0x31
#define CMD_SET_MACRO        0x40
#define CMD_GET_MACRO        0x41

#define RESP_OK              0x00
#define RESP_ERROR           0x01
#define RESP_INVALID_PARAM   0x02

// ===== ADCソース定義 =====
// 6つのADCソース (各MUXの出力先)
struct ADCSourceDef {
    ADC_HandleTypeDef* hadc;
    uint32_t channel;
};

static const ADCSourceDef ADC_SOURCES[6] = {
    { &hadc1, ADC_CHANNEL_15 },  // Source 0: ADC1_IN15 (PB0)
    { &hadc2, ADC_CHANNEL_4  },  // Source 1: ADC2_IN4  (PA7)
    { &hadc2, ADC_CHANNEL_3  },  // Source 2: ADC2_IN3  (PA6)
    { &hadc1, ADC_CHANNEL_2  },  // Source 3: ADC1_IN2  (PA1)
    { &hadc1, ADC_CHANNEL_1  },  // Source 4: ADC1_IN1  (PA0)
    { &hadc2, ADC_CHANNEL_10 },  // Source 5: ADC2_IN10 (PF1)
};

static const char* hidCodeToName(uint8_t code) {
    static const char* LETTERS[26] = {
        "A","B","C","D","E","F","G","H","I","J","K","L","M",
        "N","O","P","Q","R","S","T","U","V","W","X","Y","Z"
    };
    static const char* DIGITS[10] = {"1","2","3","4","5","6","7","8","9","0"};

    if (code >= 0x04 && code <= 0x1D) return LETTERS[code - 0x04];
    if (code >= 0x1E && code <= 0x27) return DIGITS[code - 0x1E];

    switch (code) {
    case 0x00: return "Unassigned";
    case 0x28: return "Enter";
    case 0x29: return "Esc";
    case 0x2A: return "Backspace";
    case 0x2B: return "Tab";
    case 0x2C: return "Space";
    case 0x2D: return "-";
    case 0x2E: return "=";
    case 0x2F: return "[";
    case 0x30: return "]";
    case 0x31: return "\\";
    case 0x33: return ";";
    case 0x34: return "'";
    case 0x35: return "`";
    case 0x36: return ",";
    case 0x37: return ".";
    case 0x38: return "/";
    case 0x39: return "CapsLock";
    case 0x3A: return "F1";
    case 0x3B: return "F2";
    case 0x3C: return "F3";
    case 0x3D: return "F4";
    case 0x3E: return "F5";
    case 0x3F: return "F6";
    case 0x40: return "F7";
    case 0x41: return "F8";
    case 0x42: return "F9";
    case 0x43: return "F10";
    case 0x44: return "F11";
    case 0x45: return "F12";
    case 0x46: return "PrintScr";
    case 0x47: return "ScrollLock";
    case 0x48: return "Pause";
    case 0x49: return "Insert";
    case 0x4A: return "Home";
    case 0x4B: return "PageUp";
    case 0x4C: return "Delete";
    case 0x4D: return "End";
    case 0x4E: return "PageDn";
    case 0x4F: return "Right";
    case 0x50: return "Left";
    case 0x51: return "Down";
    case 0x52: return "Up";
    case 0x53: return "NumLock";
    case 0x54: return "KP/";
    case 0x55: return "KP*";
    case 0x56: return "KP-";
    case 0x57: return "KP+";
    case 0x58: return "KPEnter";
    case 0x59: return "KP1";
    case 0x5A: return "KP2";
    case 0x5B: return "KP3";
    case 0x5C: return "KP4";
    case 0x5D: return "KP5";
    case 0x5E: return "KP6";
    case 0x5F: return "KP7";
    case 0x60: return "KP8";
    case 0x61: return "KP9";
    case 0x62: return "KP0";
    case 0x63: return "KP.";
    case 0x65: return "Menu";
    case 0xE0: return "L-Ctrl";
    case 0xE1: return "L-Shift";
    case 0xE2: return "L-Alt";
    case 0xE3: return "L-GUI";
    case 0xE4: return "R-Ctrl";
    case 0xE5: return "R-Shift";
    case 0xE6: return "R-Alt";
    case 0xE7: return "R-GUI";
    // JIS International keys
    case 0x32: return "]";
    case 0x87: return "Ro(\\)";
    case 0x88: return "Kana";
    case 0x89: return "Yen";
    case 0x8A: return "Henkan";
    case 0x8B: return "Muhenkan";
    default: return "Unknown";
    }
}

// ADCチャンネル設定 (チャンネル切替時に1回だけ呼ぶ)
static bool configureADCChannel(ADC_HandleTypeDef* hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    return (HAL_ADC_ConfigChannel(hadc, &sConfig) == HAL_OK);
}

// ADC 1回変換 (チャンネルは設定済み前提)
static uint32_t readADCOnce(ADC_HandleTypeDef* hadc) {
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0xFFFF;
    }
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0xFFFF;
    }
    uint32_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

// 後方互換 (setupダミースキャン用)
static uint32_t readADCChannel(ADC_HandleTypeDef* hadc, uint32_t channel) {
    if (!configureADCChannel(hadc, channel)) return 0xFFFF;
    readADCOnce(hadc); // discard
    return readADCOnce(hadc);
}

extern "C" void ProcessFeatureReport(uint8_t* data, uint16_t len, uint8_t* response_out) {
    usb_rx_cnt++;
    if (len < 2) return; // Need Report ID + at least 1 command byte

    // Report ID is the first byte of data; skip it for command processing
    uint8_t reportId = data[0];

    uint8_t local[32];
    memset(local, 0, 32);
    uint16_t copyLen = (len - 1) < 32 ? (len - 1) : 31;
    memcpy(local, data + 1, copyLen); // Skip Report ID

    uint8_t cmd = local[0];
    last_received_cmd = cmd;

    // response_out[0] = Report ID (for GET_REPORT response)
    // response_out[1..32] = actual response data
    memset(response_out, 0, 33);
    response_out[0] = reportId;

    // 'response' points to response_out+1 so existing code indices are unchanged
    uint8_t* response = response_out + 1;
    response[0] = cmd;

    switch (cmd) {
    case CMD_SET_SENSITIVITY: {
        uint8_t keyIdx = local[1];
        uint16_t value = (uint16_t)local[2] | ((uint16_t)local[3] << 8);
        if (value < 1 || value > 4095) {
            response[1] = RESP_INVALID_PARAM; break;
        }
        keyboard.setSensitivity((int)keyIdx, (uint32_t)value);
        response[1] = RESP_OK;
        break;
    }
    case CMD_GET_SENSITIVITY: {
        uint8_t keyIdx = local[1];
        if (keyIdx >= RapidTriggerKeyboard::TOTAL_KEY_COUNT && keyIdx != 0xFF) {
            response[1] = RESP_INVALID_PARAM; break;
        }
        response[1] = RESP_OK;
        if (keyIdx == 0xFF) {
            int count = RapidTriggerKeyboard::TOTAL_KEY_COUNT;
            if (count > 15) count = 15;
            for (int i = 0; i < count; i++) {
                uint16_t s = (uint16_t)keyboard.getSensitivity(i);
                response[2 + i * 2] = (uint8_t)(s & 0xFF);
                response[2 + i * 2 + 1] = (uint8_t)(s >> 8);
            }
        } else {
            uint16_t s = (uint16_t)keyboard.getSensitivity(keyIdx);
            response[2] = (uint8_t)(s & 0xFF);
            response[3] = (uint8_t)(s >> 8);
        }
        break;
    }
    case CMD_SET_KEYCODE: {
        uint8_t keyIdx = local[1];
        uint16_t code = (uint16_t)local[2] | ((uint16_t)local[3] << 8);
        if (keyIdx >= RapidTriggerKeyboard::TOTAL_KEY_COUNT) {
            response[1] = RESP_INVALID_PARAM; break;
        }
        keyboard.setKeycode(keyIdx, code);
        response[1] = RESP_OK;
        break;
    }
    case CMD_GET_KEYCODE: {
        uint8_t keyIdx = local[1];
        if (keyIdx == 0xFF) {
            response[1] = RESP_OK;
            int count = RapidTriggerKeyboard::TOTAL_KEY_COUNT;
            if (count > 15) count = 15;  // 16-bit: 2 bytes per key, max 15 keys
            for (int i = 0; i < count; i++) {
                uint16_t kc = keyboard.getKeycode(i);
                response[2 + i * 2] = (uint8_t)(kc & 0xFF);
                response[2 + i * 2 + 1] = (uint8_t)(kc >> 8);
            }
        } else if (keyIdx < RapidTriggerKeyboard::TOTAL_KEY_COUNT) {
            response[1] = RESP_OK;
            uint16_t kc = keyboard.getKeycode(keyIdx);
            response[2] = (uint8_t)(kc & 0xFF);
            response[3] = (uint8_t)(kc >> 8);
        } else {
            response[1] = RESP_INVALID_PARAM;
        }
        break;
    }
    case CMD_SET_DEADZONE: {
        uint8_t keyIdx = local[1];
        uint16_t value = (uint16_t)local[2] | ((uint16_t)local[3] << 8);
        if (value > 500) { response[1] = RESP_INVALID_PARAM; break; }
        keyboard.setDeadZone((int)keyIdx, (uint32_t)value);
        response[1] = RESP_OK;
        break;
    }
    case CMD_GET_DEADZONE: {
        uint8_t keyIdx = local[1];
        if (keyIdx == 0xFF) {
            response[1] = RESP_OK;
            int count = RapidTriggerKeyboard::TOTAL_KEY_COUNT;
            if (count > 15) count = 15;
            for (int i = 0; i < count; i++) {
                uint16_t dz = (uint16_t)keyboard.getDeadZone(i);
                response[2 + i * 2] = (uint8_t)(dz & 0xFF);
                response[2 + i * 2 + 1] = (uint8_t)(dz >> 8);
            }
        } else if (keyIdx < RapidTriggerKeyboard::TOTAL_KEY_COUNT) {
            response[1] = RESP_OK;
            uint16_t dz = (uint16_t)keyboard.getDeadZone(keyIdx);
            response[2] = (uint8_t)(dz & 0xFF);
            response[3] = (uint8_t)(dz >> 8);
        } else {
            response[1] = RESP_INVALID_PARAM;
        }
        break;
    }
    case CMD_SET_LED_MODE: {
        uint8_t mode = local[1];
        uint8_t bright = local[2];
        uint8_t spd = local[3];
        if (mode >= LED_MODE_COUNT) { response[1] = RESP_INVALID_PARAM; break; }
        keyboard.ledConfig.mode = (LedMode)mode;
        keyboard.ledConfig.brightness = bright;
        if (spd <= 100) keyboard.ledConfig.speed = spd;
        response[1] = RESP_OK;
        break;
    }
    case CMD_GET_LED_MODE: {
        response[1] = RESP_OK;
        response[2] = (uint8_t)keyboard.ledConfig.mode;
        response[3] = keyboard.ledConfig.brightness;
        response[4] = keyboard.ledConfig.speed;
        break;
    }
    case CMD_SET_MACRO: {
        uint8_t idx = local[1];
        uint8_t sc  = local[2];
        if (idx < keyboard.TOTAL_KEY_COUNT && sc <= MAX_MACRO_STEPS) {
            MacroStep steps[MAX_MACRO_STEPS] = {};
            for (int s = 0; s < sc && (3 + s*2 + 1) < 32; s++) {
                steps[s].modifiers = local[3 + s*2];
                steps[s].keycode   = local[4 + s*2];
            }
            keyboard.setMacro(idx, sc, steps);
            response[1] = RESP_OK;
        } else {
            response[1] = RESP_INVALID_PARAM;
        }
        break;
    }
    case CMD_GET_MACRO: {
        uint8_t idx = local[1];
        if (idx < keyboard.TOTAL_KEY_COUNT) {
            uint8_t sc = keyboard.getMacroStepCount(idx);
            const MacroStep* steps = keyboard.getMacroSteps(idx);
            response[1] = RESP_OK;
            response[2] = sc;
            for (int s = 0; s < sc && (3 + s*2 + 1) < 32; s++) {
                response[3 + s*2] = steps[s].modifiers;
                response[4 + s*2] = steps[s].keycode;
            }
        } else {
            response[1] = RESP_INVALID_PARAM;
        }
        break;
    }
    case CMD_SAVE_TO_FLASH: {
        keyboard.saveToFlash();
        response[1] = (keyboard.flash_status == 0) ? RESP_OK : RESP_ERROR;
        response[2] = (uint8_t)keyboard.flash_status;
        break;
    }
    case CMD_RESET_DEFAULTS: {
        keyboard.resetDefaults();
        response[1] = RESP_OK;
        break;
    }
    case CMD_GET_KEY_COUNT: {
        response[1] = RESP_OK;
        response[2] = (uint8_t)RapidTriggerKeyboard::TOTAL_KEY_COUNT;
        break;
    }
    default:
        response[1] = RESP_ERROR;
        break;
    }
}

extern "C" void ProcessConfigPacket(uint8_t* data, uint16_t len) {
    last_received_len = len;
    usb_rx_cnt++;
    if (len > 0) last_received_cmd = data[0];
    if (len >= 4) {
        last_payload[0] = data[0];
        last_payload[1] = data[1];
        last_payload[2] = data[2];
        last_payload[3] = data[3];
    }

    if (len < 1) return;
    uint8_t cmd = data[0];
    uint8_t idx_target = 1;
    uint8_t idx_val = 2;

    if (cmd == 0x00 && len > 1) {
        cmd = data[1]; idx_target = 2; idx_val = 3;
    }

    if (cmd == 0x01) {
        config_target = data[idx_target];
        config_val = data[idx_val];
        config_update_request = true;
    }
}

// 前方宣言
static void DWT_Init(void);
static inline void delay_us(uint32_t us);

extern "C" void setup()
{
    keyboard.init();
    keyboard.loadFromFlash();

    printf("=== Full-size Rapid Trigger Keyboard ===\r\n");
    printf("Keys:%d Sources:%d\r\n",
           RapidTriggerKeyboard::TOTAL_KEY_COUNT,
           RapidTriggerKeyboard::SOURCE_COUNT);

    // DWTサイクルカウンタ初期化 (us遅延に使用)
    DWT_Init();

    // ADCキャリブレーション
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        printf("[ERR] ADC1 Calibration Failed\r\n");
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        printf("[ERR] ADC2 Calibration Failed\r\n");
    }

    // センサーの電源安定化待ち
    HAL_Delay(500);

    // ADCダミースキャン (内部のS&HキャパシタとMUXのゴミを読む)
    for (int dummy = 0; dummy < 10; dummy++) {
        for (int ch = 0; ch < RapidTriggerKeyboard::MUX_CH_COUNT; ch++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            for (volatile int w = 0; w < 200; w++);
            for (int src = 0; src < RapidTriggerKeyboard::SOURCE_COUNT; src++) {
                readADCChannel(ADC_SOURCES[src].hadc, ADC_SOURCES[src].channel);
            }
        }
    }

    // LED PWM開始
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("Setup complete.\r\n");
    printf("------------------------------\r\n");
}

// us単位ディレイ (DWTサイクルカウンタ使用)
static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

// DWT初期化
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// MUX切り替え (PB4-PB7: S0-S3) — BSRR一括アトミック書込み
void selectMuxChannel(int channel) {
    // PB4-PB7 のセット/リセットをBSRRで同時に反映
    uint32_t bsrr = 0;
    // セットビット (bits 0-15): 該当ピンを HIGH にする
    // リセットビット (bits 16-31): 該当ピンを LOW にする
    const uint16_t pins[] = { GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7 };
    for (int i = 0; i < 4; i++) {
        if (channel & (1 << i)) {
            bsrr |= pins[i];            // Set
        } else {
            bsrr |= (uint32_t)pins[i] << 16; // Reset
        }
    }
    GPIOB->BSRR = bsrr;
}

// 3サンプル中央値 (スパイク除去)
static uint32_t readADCMedian(ADC_HandleTypeDef* hadc) {
    uint32_t a = readADCOnce(hadc);
    uint32_t b = readADCOnce(hadc);
    uint32_t c = readADCOnce(hadc);
    // ソート後の中央値
    if (a > b) { uint32_t t = a; a = b; b = t; }
    if (b > c) { uint32_t t = b; b = c; c = t; }
    if (a > b) { uint32_t t = a; a = b; b = t; }
    return b;
}

// 高速ADC読み取り (レジスタ直接アクセス, HALオーバーヘッド排除)
static inline uint32_t readADCFast(ADC_HandleTypeDef* hadc) {
    ADC_TypeDef* inst = hadc->Instance;
    // ADCが無効なら有効化
    if (!(inst->CR & ADC_CR_ADEN)) {
        inst->ISR = ADC_ISR_ADRDY;
        inst->CR |= ADC_CR_ADEN;
        while (!(inst->ISR & ADC_ISR_ADRDY));
    }
    // フラグクリア → 変換開始 → 完了待ち → 結果読み取り
    inst->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    inst->CR |= ADC_CR_ADSTART;
    while (!(inst->ISR & ADC_ISR_EOC));
    return inst->DR;
}

extern "C" void loop()
{
    static uint32_t mux_adc[RapidTriggerKeyboard::MUX_CH_COUNT][RapidTriggerKeyboard::SOURCE_COUNT] = {{0}};
    static uint32_t scan_us = 0;

    if (config_update_request) {
        config_update_request = false;
        keyboard.setSensitivity((int)config_target, config_val);
    }

    // キー押下 → LED制御
    {
        KeyboardReport* rpt = keyboard.getReport();
        bool any_key = (rpt->MODIFIER != 0);
        if (!any_key) {
            for (size_t i = 0; i < sizeof(rpt->KEYS); i++) {
                if (rpt->KEYS[i] != 0) { any_key = true; break; }
            }
        }

        LedMode mode = keyboard.ledConfig.mode;
        uint8_t maxBright = keyboard.ledConfig.brightness;

        if (mode == LED_MODE_FADE) {
            if (any_key) led_brightness = maxBright;
        } else if (mode == LED_MODE_SOLID) {
            led_brightness = any_key ? maxBright : 0;
        } else if (mode == LED_MODE_OFF) {
            led_brightness = 0;
        }
    }

    // ===== ADCスキャン (全7ソース · 1kHz高速) =====
    uint32_t scan_start = DWT->CYCCNT;

    for (int src = 0; src < RapidTriggerKeyboard::SOURCE_COUNT; src++) {
        const ADCSourceDef& adc = ADC_SOURCES[src];

        // ADCチャネル設定 (ソースごとに1回)
        if (!configureADCChannel(adc.hadc, adc.channel)) continue;

        // ADC内部S&Hセトリング用ダミー
        readADCFast(adc.hadc);

        for (int ch = 0; ch < RapidTriggerKeyboard::MUX_CH_COUNT; ch++) {
            selectMuxChannel(ch);
            delay_us(5);  // MUXセトリング (5μs)

            // 2サンプル平均 (ノイズ低減 + 高速)
            uint32_t v1 = readADCFast(adc.hadc);
            uint32_t v2 = readADCFast(adc.hadc);
            uint32_t val = (v1 + v2) / 2;
            keyboard.updateKeyByMux(ch, src, val);
            mux_adc[ch][src] = val;
        }
    }

    scan_us = (DWT->CYCCNT - scan_start) / (SystemCoreClock / 1000000);

    // USBレポート送信 (Report ID付き, 1ms間隔)
    static uint32_t last_usb_send = 0;
    static uint16_t last_consumer = 0;
    uint32_t now = HAL_GetTick();
    if (now != last_usb_send) {
        last_usb_send = now;
        KeyboardReport* report = keyboard.getReport();
        uint16_t consumer = keyboard.getConsumerKey();

        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            // Consumerキーの状態が変わったら優先送信
            if (consumer != last_consumer) {
                uint8_t cc_buf[3];
                cc_buf[0] = 0x02;  // Report ID 2: Consumer
                cc_buf[1] = (uint8_t)(consumer & 0xFF);
                cc_buf[2] = (uint8_t)(consumer >> 8);
                if (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, cc_buf, 3) == USBD_OK) {
                    last_consumer = consumer;
                }
            } else {
                // Keyboardレポート (Report ID 1)
                uint8_t kb_buf[1 + sizeof(KeyboardReport)]; // 21 bytes
                kb_buf[0] = 0x01;  // Report ID 1: Keyboard
                memcpy(&kb_buf[1], report, sizeof(KeyboardReport));
                USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, kb_buf, sizeof(kb_buf));
            }
        }
    }

    // デバッグ出力 (500msに1回, USB未接続でも出力)
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print > 500) {
        last_print = HAL_GetTick();

        printf("Sens:%lu LED:%lu Scan:%luus | USB:%s Keys:%d\r\n",
               keyboard.getSensitivity(0), (uint32_t)led_brightness,
               (unsigned long)scan_us,
               (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? "ON" : "OFF",
               RapidTriggerKeyboard::TOTAL_KEY_COUNT);

        // ON状態のキーのみ表示
        bool any_active = false;
        for (int k = 0; k < RapidTriggerKeyboard::TOTAL_KEY_COUNT; k++) {
            if (keyboard.isKeyActive(k)) {
                uint16_t code = keyboard.getKeycode(k);
                if (IS_CONSUMER_KEY(code)) {
                    printf("  [ON] Media(%04X)\r\n", code);
                } else {
                    printf("  [ON] %s(%02X)\r\n", hidCodeToName((uint8_t)code), (uint8_t)code);
                }
                any_active = true;
            }
        }
        if (!any_active) {
            printf("  All keys OFF\r\n");
        }
    }
}

// TIM6 割り込みコールバック
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        static uint32_t sub_counter = 0;
        static uint32_t effect_counter = 0;
        static bool blink_on = false;

        LedMode mode = keyboard.ledConfig.mode;
        uint8_t maxBright = keyboard.ledConfig.brightness;
        uint8_t speed = keyboard.ledConfig.speed;

        uint32_t threshold = 110 - speed;
        sub_counter += 3;

        if (sub_counter >= threshold) {
            sub_counter -= threshold;
            effect_counter++;

            switch (mode) {
            case LED_MODE_FADE:
                if (led_brightness > 0) led_brightness--;
                break;

            case LED_MODE_BLINK:
                if (effect_counter >= 128) {
                    effect_counter = 0;
                    blink_on = !blink_on;
                }
                led_brightness = blink_on ? maxBright : 0;
                break;

            case LED_MODE_BREATHING: {
                uint32_t phase = effect_counter % 512;
                uint32_t level;
                if (phase < 256) level = phase;
                else level = 511 - phase;
                led_brightness = (level * maxBright) / 255;
                break;
            }

            case LED_MODE_SOLID:
            case LED_MODE_OFF:
            default:
                break;
            }
        }

        // USB Suspendedの場合(PC sleep)、LEDを消灯
        uint32_t brightness = usb_suspended ? 0 : (uint32_t)led_brightness;
        uint32_t pwm_val = brightness * brightness;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_val);
    }
}