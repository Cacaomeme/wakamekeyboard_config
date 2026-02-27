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

// LED明るさ制御 (TIM6割り込みで減光)
// 知覚的な明るさレベル (0-255)。PWMへはガンマ補正(²)で変換する
volatile uint32_t led_brightness = 0;

// USB経由の設定変更リクエスト (Output Report互換)
volatile bool config_update_request = false;
volatile uint8_t config_target = 0;
volatile uint32_t config_val = 0;

// USBデバッグ変数
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

// 応答ステータス
#define RESP_OK              0x00
#define RESP_ERROR           0x01
#define RESP_INVALID_PARAM   0x02

// Feature Report処理 (SET_REPORT経由で呼ばれる)
// data: 受信した32バイト, response: 応答を書き込む32バイト (同じバッファ)
extern "C" void ProcessFeatureReport(uint8_t* data, uint16_t len, uint8_t* response) {
    usb_rx_cnt++;
    if (len < 1) return;

    // data と response は同じバッファ(Feature_buf)を指すため、
    // 応答書き込み前に受信データをローカルにコピーする
    uint8_t local[32];
    memcpy(local, data, 32);

    uint8_t cmd = local[0];
    last_received_cmd = cmd;

    // 応答バッファをクリア
    memset(response, 0, 32);
    response[0] = cmd;  // エコーバック

    switch (cmd) {
    case CMD_SET_SENSITIVITY: {
        // local[1]=keyIndex, local[2..3]=value (uint16_t LE)
        uint8_t keyIdx = local[1];
        uint16_t value = (uint16_t)local[2] | ((uint16_t)local[3] << 8);

        if (value < 1 || value > 4095) {
            response[1] = RESP_INVALID_PARAM;
            break;
        }

        keyboard.setSensitivity((int)keyIdx, (uint32_t)value);
        response[1] = RESP_OK;
        printf("[CFG] SetSens Key:%d Val:%d\r\n", keyIdx, value);
        break;
    }
    case CMD_GET_SENSITIVITY: {
        // local[1]=keyIndex
        uint8_t keyIdx = local[1];
        if (keyIdx >= RapidTriggerKeyboard::TOTAL_KEY_COUNT && keyIdx != 0xFF) {
            response[1] = RESP_INVALID_PARAM;
            break;
        }

        response[1] = RESP_OK;

        if (keyIdx == 0xFF) {
            // 全キーの感度を返す (最大17キー × 2バイト = 34 → 32バイトに収まるよう15キー分)
            // response[2..3]=key0, [4..5]=key1, ...
            int count = RapidTriggerKeyboard::TOTAL_KEY_COUNT;
            if (count > 15) count = 15; // 32バイト制限
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
        // local[1]=keyIndex, local[2]=keycode
        uint8_t keyIdx = local[1];
        uint8_t code = local[2];
        if (keyIdx >= RapidTriggerKeyboard::TOTAL_KEY_COUNT || code == 0 || code >= 120) {
            response[1] = RESP_INVALID_PARAM;
            break;
        }
        keyboard.setKeycode(keyIdx, code);
        response[1] = RESP_OK;
        printf("[CFG] SetKeycode Key:%d Code:0x%02X\r\n", keyIdx, code);
        break;
    }
    case CMD_GET_KEYCODE: {
        uint8_t keyIdx = local[1];
        if (keyIdx == 0xFF) {
            // 全キーのキーコードを返す
            response[1] = RESP_OK;
            int count = RapidTriggerKeyboard::TOTAL_KEY_COUNT;
            if (count > 30) count = 30; // 32バイト制限
            for (int i = 0; i < count; i++) {
                response[2 + i] = keyboard.getKeycode(i);
            }
        } else if (keyIdx < RapidTriggerKeyboard::TOTAL_KEY_COUNT) {
            response[1] = RESP_OK;
            response[2] = keyboard.getKeycode(keyIdx);
        } else {
            response[1] = RESP_INVALID_PARAM;
        }
        break;
    }
    case CMD_SET_DEADZONE: {
        // local[1]=keyIndex, local[2..3]=value (uint16_t LE)
        uint8_t keyIdx = local[1];
        uint16_t value = (uint16_t)local[2] | ((uint16_t)local[3] << 8);
        if (value > 500) {
            response[1] = RESP_INVALID_PARAM;
            break;
        }
        keyboard.setDeadZone((int)keyIdx, (uint32_t)value);
        response[1] = RESP_OK;
        printf("[CFG] SetDZ Key:%d Val:%d\r\n", keyIdx, value);
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
        // local[1]=mode, local[2]=brightness, local[3]=speed
        uint8_t mode = local[1];
        uint8_t bright = local[2];
        uint8_t spd = local[3];
        if (mode >= LED_MODE_COUNT) {
            response[1] = RESP_INVALID_PARAM;
            break;
        }
        keyboard.ledConfig.mode = (LedMode)mode;
        keyboard.ledConfig.brightness = bright;
        if (spd <= 100) keyboard.ledConfig.speed = spd;
        response[1] = RESP_OK;
        printf("[CFG] SetLED mode:%d bright:%d speed:%d\r\n", mode, bright, spd);
        break;
    }
    case CMD_GET_LED_MODE: {
        response[1] = RESP_OK;
        response[2] = (uint8_t)keyboard.ledConfig.mode;
        response[3] = keyboard.ledConfig.brightness;
        response[4] = keyboard.ledConfig.speed;
        break;
    }
    case CMD_SAVE_TO_FLASH: {
        keyboard.saveToFlash();
        response[1] = (keyboard.flash_status == 0) ? RESP_OK : RESP_ERROR;
        response[2] = (uint8_t)keyboard.flash_status;
        printf("[CFG] SaveFlash status:%d\r\n", keyboard.flash_status);
        break;
    }
    case CMD_RESET_DEFAULTS: {
        keyboard.resetDefaults();
        response[1] = RESP_OK;
        printf("[CFG] ResetDefaults\r\n");
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

// 旧Output Report互換 (既存のInterrupt OUT経由)
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
        cmd = data[1];
        idx_target = 2;
        idx_val = 3;
    }

    if (cmd == 0x01) {
        config_target = data[idx_target];
        config_val = data[idx_val];
        config_update_request = true;
    }
}

extern "C" void setup()
{
    keyboard.init();
    keyboard.loadFromFlash();

    printf("=== Rapid Trigger Keyboard ===\r\n");
    printf("Keys:%d Sensitivity(K0):%lu\r\n",
           RapidTriggerKeyboard::TOTAL_KEY_COUNT,
           keyboard.getSensitivity(0));

    // ADCキャリブレーション
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        printf("[ERR] ADC1 Calibration Failed\r\n");
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        printf("[ERR] ADC2 Calibration Failed\r\n");
    }

    // LED PWM開始 (TIM3 CH2)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // TIM6割り込み開始 (LED減光用)
    HAL_TIM_Base_Start_IT(&htim6);

    printf("Setup complete.\r\n");
    printf("------------------------------\r\n");
}

// MUX切り替え (PB4-PB7: S0-S3)
void selectMuxChannel(int channel) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (channel & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t debug_adc1[16];
uint32_t debug_adc2[16];

extern "C" void loop()
{
    // USB経由の設定変更 (旧Output Report互換)
    if (config_update_request) {
        config_update_request = false;
        keyboard.setSensitivity((int)config_target, config_val);
        printf("[CFG] Key:%d Sens:%d\r\n", config_target, (int)config_val);
    }

    // キー押下検出 → LEDモードに応じた制御
    {
        KeyboardReport* rpt = keyboard.getReport();
        bool any_key = (rpt->MODIFIER != 0);
        if (!any_key) {
            for (int i = 0; i < 15; i++) {
                if (rpt->KEYS[i] != 0) { any_key = true; break; }
            }
        }

        LedMode mode = keyboard.ledConfig.mode;
        uint8_t maxBright = keyboard.ledConfig.brightness;

        if (mode == LED_MODE_FADE) {
            // Fade: キー押下→最大輝度、TIM6で減光
            if (any_key) led_brightness = maxBright;
        } else if (mode == LED_MODE_SOLID) {
            // Solid: キー押下中=点灯、離すと即消灯
            led_brightness = any_key ? maxBright : 0;
        } else if (mode == LED_MODE_BLINK || mode == LED_MODE_BREATHING) {
            // Blink/Breathing: TIM6側で制御
            // any_keyフラグをグローバルで渡す
        } else if (mode == LED_MODE_OFF) {
            led_brightness = 0;
        }
    }

    // ADCスキャン (全MUXチャンネル)
    for (int ch = 0; ch < RapidTriggerKeyboard::MUX_CH_COUNT; ch++) {
        selectMuxChannel(ch);

        // 信号安定待ち
        for (volatile int w = 0; w < 200; w++);

        // ADC1 (MUX1)
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
        uint32_t ret1 = HAL_ADC_Start(&hadc1);
        if (ret1 == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                uint32_t val1 = HAL_ADC_GetValue(&hadc1);
                keyboard.updateKeyByMux(ch, 0, val1);
                debug_adc1[ch] = val1;
            } else {
                HAL_ADC_Stop(&hadc1);
                debug_adc1[ch] = 8888;
            }
        } else {
            HAL_ADC_Stop(&hadc1);
            debug_adc1[ch] = 9000 + ret1;
        }

        // ADC2 (MUX2)
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
        uint32_t ret2 = HAL_ADC_Start(&hadc2);
        if (ret2 == HAL_OK) {
            if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
                uint32_t val2 = HAL_ADC_GetValue(&hadc2);
                keyboard.updateKeyByMux(ch, 1, val2);
                debug_adc2[ch] = val2;
            } else {
                HAL_ADC_Stop(&hadc2);
                debug_adc2[ch] = 8888;
            }
        } else {
            HAL_ADC_Stop(&hadc2);
            debug_adc2[ch] = 9000 + ret2;
        }
    }

    // USBレポート送信
    KeyboardReport* report = keyboard.getReport();
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)report, sizeof(KeyboardReport));

        // デバッグ出力 (200msに1回)
        static uint32_t last_print = 0;
        if (HAL_GetTick() - last_print > 200) {
            last_print = HAL_GetTick();

            // ステータス行
            printf("Sens:%lu LED:%lu | USB[Cmd:%02X Cnt:%lu]\r\n",
                   keyboard.getSensitivity(0), (uint32_t)led_brightness,
                   last_received_cmd, usb_rx_cnt);

            // MUX1(ADC1): キー名とADC値
            printf(" KP0:%4lu KP.:%4lu Ent:%4lu KP3:%4lu KP2:%4lu KP1:%4lu KP6:%4lu KP5:%4lu KP4:%4lu\r\n",
                   debug_adc1[0],  debug_adc1[15], debug_adc1[14],
                   debug_adc1[13], debug_adc1[12], debug_adc1[11],
                   debug_adc1[10], debug_adc1[9],  debug_adc1[8]);

            // MUX2(ADC2): キー名とADC値
            printf(" KP+:%4lu KP9:%4lu KP8:%4lu KP7:%4lu KP-:%4lu KP*:%4lu KP/:%4lu Num:%4lu\r\n",
                   debug_adc2[15], debug_adc2[14], debug_adc2[13],
                   debug_adc2[12], debug_adc2[11], debug_adc2[10],
                   debug_adc2[9],  debug_adc2[8]);
        }
    }
}

// TIM6 割り込みコールバック: LEDエフェクト処理
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        static uint32_t sub_counter = 0;
        static uint32_t effect_counter = 0;
        static bool blink_on = false;

        LedMode mode = keyboard.ledConfig.mode;
        uint8_t maxBright = keyboard.ledConfig.brightness;
        uint8_t speed = keyboard.ledConfig.speed; // 0-100 (0.0-10.0)

        // speed: 0=最遅, 100=最速。アキュムレータ方式で0.1刻み対応
        // divider = 110 - speed (10〜110), カウンタ += 3 (低速寄り)
        uint32_t threshold = 110 - speed;
        sub_counter += 3;

        if (sub_counter >= threshold) {
            sub_counter -= threshold;
            effect_counter++;

            switch (mode) {
            case LED_MODE_FADE:
                // 徐々に減光 (キー押下でled_brightnessがmaxBrightにセットされる)
                if (led_brightness > 0) led_brightness--;
                break;

            case LED_MODE_BLINK:
                // 約256カウントで1周期の点滅
                if (effect_counter >= 128) {
                    effect_counter = 0;
                    blink_on = !blink_on;
                }
                led_brightness = blink_on ? maxBright : 0;
                break;

            case LED_MODE_BREATHING: {
                // サイン波近似: 三角波で呼吸エフェクト
                uint32_t phase = effect_counter % 512;
                uint32_t level;
                if (phase < 256) {
                    level = phase; // 0→255
                } else {
                    level = 511 - phase; // 255→0
                }
                led_brightness = (level * maxBright) / 255;
                break;
            }

            case LED_MODE_SOLID:
                // loop()側で制御 (キー押下=ON、離す=OFF)
                break;

            case LED_MODE_OFF:
                led_brightness = 0;
                break;

            default:
                break;
            }
        }

        // ガンマ2.0補正: PWM = brightness² (0-65025)
        uint32_t pwm_val = (uint32_t)led_brightness * led_brightness;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_val);
    }
}