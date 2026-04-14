#ifndef NKRO_HPP
#define NKRO_HPP

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Consumer Key フラグ (keycode の bit15)
#define CONSUMER_KEY_FLAG  0x8000
#define IS_CONSUMER_KEY(code) (((code) & CONSUMER_KEY_FLAG) != 0)
#define CONSUMER_USAGE(code)  ((code) & 0x7FFF)

// NKRO対応のレポート構造体 (20バイト)
typedef struct {
    uint8_t MODIFIER;
    uint8_t RESERVED;
    uint8_t KEYS[18]; // 140 keys / 8 = 18 bytes (0x00-0x8B, JIS International含む)
} KeyboardReport;

// LEDモード
enum LedMode : uint8_t {
    LED_MODE_FADE      = 0,
    LED_MODE_BLINK     = 1,
    LED_MODE_BREATHING = 2,
    LED_MODE_SOLID     = 3,
    LED_MODE_OFF       = 4,
    LED_MODE_COUNT     = 5
};

// LED設定構造体
struct LedConfig {
    LedMode mode;
    uint8_t brightness;
    uint8_t speed;       // 0-100
};

// マクロステップ
#define MAX_MACRO_STEPS 8
#define MACRO_ACTION_PRESS   0
#define MACRO_ACTION_RELEASE 1

struct MacroStep {
    uint8_t action;    // 0=PRESS, 1=RELEASE
    uint8_t modifiers;
    uint8_t keycode;
};

// Rapid Trigger State for a single key
struct RapidTriggerState {
    uint32_t high_peak;
    uint32_t low_peak;
    bool is_active;

    uint32_t baseline;
    bool calibrated;
    uint32_t sensitivity_on;
    uint32_t sensitivity_off;
    uint16_t keycode;        // HIDキーコード (0x00-0xE7: キーボード, 0x8xxx: Consumer)
    uint32_t dead_zone;

    // 初期キャリブレーション用 (ランタイムのみ)
    uint16_t calibration_samples;
    uint32_t calibration_sum;
    uint8_t on_debounce_count;  // OFF->ON判定の連続成立カウント

    // マクロシーケンス
    uint8_t macro_step_count;
    MacroStep macro_steps[MAX_MACRO_STEPS];

    // マクロ実行状態 (ランタイムのみ)
    bool was_active;
    uint8_t macro_exec_step;
    bool macro_exec_pressing;
    uint32_t macro_exec_tick;
    bool macro_completed;        // 全ステップ完了済み
    uint8_t macro_held_modifiers; // 現在保持中の修飾キー
    uint8_t macro_held_keys[6];  // 現在保持中のキー (最大6個)
    uint8_t macro_held_count;    // 保持中キー数
};

// コンボマクロ
#define MAX_COMBOS 8
#define MAX_COMBO_STEPS 8

struct ComboMacro {
    // 設定 (保存対象)
    uint8_t trigger_modifiers;  // トリガー修飾キービットマスク
    uint8_t trigger_keycode;    // トリガー通常キー (0=修飾キーのみ)
    uint8_t step_count;
    MacroStep steps[MAX_COMBO_STEPS];

    // ランタイム状態
    bool is_active;
    bool was_active;
    uint8_t exec_step;
    bool exec_pressing;
    uint32_t exec_tick;
    bool completed;
    uint8_t held_modifiers;
    uint8_t held_keys[6];
    uint8_t held_count;
};

class RapidTriggerKeyboard {
public:
    static const int MUX_CH_COUNT = 16;
    static const int SOURCE_COUNT = 6;     // ADCソース数
    static const int TOTAL_KEY_COUNT = 82;  // 70% JISキーボード

    RapidTriggerKeyboard();
    void init();

    // MUXインデックスとADCソースを指定して更新
    void updateKeyByMux(int muxIndex, int source, uint32_t adcValue);

    // 感度
    void setSensitivity(int keyIndex, uint32_t value);
    uint32_t getSensitivity(int keyIndex);

    // しきい値 (OFF->ON / ON->OFF)
    void setOnThreshold(int keyIndex, uint32_t value);
    uint32_t getOnThreshold(int keyIndex);
    void setOffThreshold(int keyIndex, uint32_t value);
    uint32_t getOffThreshold(int keyIndex);

    // キーコード (16bit: 0x0000-0x00E7=キーボード, 0x8xxx=Consumer)
    void setKeycode(int keyIndex, uint16_t code);
    uint16_t getKeycode(int keyIndex);

    // デッドゾーン
    void setDeadZone(int keyIndex, uint32_t value);
    uint32_t getDeadZone(int keyIndex);

    // マクロ
    void setMacro(int keyIndex, uint8_t stepCount, const MacroStep* steps);
    uint8_t getMacroStepCount(int keyIndex);
    const MacroStep* getMacroSteps(int keyIndex);

    // コンボマクロ
    void setCombo(int index, uint8_t trigMod, uint8_t trigKey, uint8_t stepCount, const MacroStep* steps);
    uint8_t getComboTriggerMod(int index);
    uint8_t getComboTriggerKey(int index);
    uint8_t getComboStepCount(int index);
    const MacroStep* getComboSteps(int index);

    // デバッグ参照
    int getMappedKeyIndex(int source, int muxIndex);
    bool isKeyActive(int keyIndex);
    uint32_t getCalibBase(int keyIndex);
    uint32_t getLowPeak(int keyIndex);
    uint32_t getHighPeak(int keyIndex);

    // LED設定
    LedConfig ledConfig;

    // Flash保存・読み込み
    void saveToFlash();
    void loadFromFlash();
    void resetDefaults();

    int flash_status;
    uint32_t flash_error_code;
    uint32_t flash_debug_val;

    // レポート生成
    KeyboardReport* getReport();
    uint16_t getConsumerKey();

private:
    int keyMapping[SOURCE_COUNT][MUX_CH_COUNT];

    RapidTriggerState keyStates[TOTAL_KEY_COUNT];
    ComboMacro combos[MAX_COMBOS];
    KeyboardReport report;
    uint16_t activeConsumerKey;

    void updateRapidTriggerState(RapidTriggerState& state, uint32_t currentVal);
};

#endif // NKRO_HPP
