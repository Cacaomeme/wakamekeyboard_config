#ifndef NKRO_HPP
#define NKRO_HPP

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <vector> // C++機能を利用

// NKRO対応のレポート構造体 (17バイト)
typedef struct {
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYS[15]; // 120 keys / 8 = 15 bytes
} KeyboardReport;

// LEDモード
enum LedMode : uint8_t {
    LED_MODE_FADE      = 0,  // キー押下→フェードアウト (デフォルト)
    LED_MODE_BLINK     = 1,  // キー押下中に点滅
    LED_MODE_BREATHING = 2,  // 常時呼吸エフェクト
    LED_MODE_SOLID     = 3,  // キー押下中は常時点灯
    LED_MODE_OFF       = 4,  // LED消灯
    LED_MODE_COUNT     = 5
};

// LED設定構造体
struct LedConfig {
    LedMode mode;
    uint8_t brightness;  // 最大輝度 (0-255)
    uint8_t speed;       // エフェクト速度 (0-100, 0.0-10.0の10倍値)
};

// Rapid Trigger State for a single key
struct RapidTriggerState {
    uint32_t high_peak;      // 押し込みの最深点
    uint32_t low_peak;       // 戻りの最浅点
    bool is_active;          // 現在のキー状態
    
    uint32_t baseline;       // 初期値 (アイドル値)
    bool calibrated;         // 初期値取得済みフラグ
    uint32_t sensitivity;    // 感度 (Rapid Trigger閾値)
    uint8_t keycode;         // 対応するUSBキーコード
    uint32_t dead_zone;      // デッドゾーン (baseline付近の不感帯)
};

class RapidTriggerKeyboard {
public:
    static const int MUX_CH_COUNT = 16;
    
    // 現在使用している有効なキーの数（MUX1: 9個, MUX2: 8個 = 17個）
    static const int TOTAL_KEY_COUNT = 17;

    RapidTriggerKeyboard();
    void init();
    
    // MUXインデックスとADCソースを指定して更新
    void updateKeyByMux(int muxIndex, int source, uint32_t adcValue);
    
    // 感度
    void setSensitivity(int keyIndex, uint32_t value);
    uint32_t getSensitivity(int keyIndex);

    // キーコード
    void setKeycode(int keyIndex, uint8_t code);
    uint8_t getKeycode(int keyIndex);

    // デッドゾーン
    void setDeadZone(int keyIndex, uint32_t value);
    uint32_t getDeadZone(int keyIndex);

    // LED設定
    LedConfig ledConfig;

    // Flash保存・読み込み
    void saveToFlash();
    void loadFromFlash();
    void resetDefaults();
    
    // デバッグ用: Flash操作の結果コード
    int flash_status;
    uint32_t flash_error_code;
    uint32_t flash_debug_val;

    // 現在の状態からレポートを生成して返す
    KeyboardReport* getReport();

private:
    // マッピング管理用: [Source][MuxIndex] -> StateIndex (-1 if unused)
    int keyMapping[2][MUX_CH_COUNT];
    
    // 全キーの状態配列
    RapidTriggerState keyStates[TOTAL_KEY_COUNT];
    
    KeyboardReport report;
    
    void updateRapidTriggerState(RapidTriggerState& state, uint32_t currentVal);
};

#endif // NKRO_HPP
