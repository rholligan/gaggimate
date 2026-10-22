#pragma once
#ifndef GAGGIMATE_HEADLESS

#include <Arduino.h>
#include <display/drivers/Driver.h>

enum class DisplayPowerState {
    ACTIVE,    // 190mA @ 240MHz, full brightness, 60Hz UI
    DIMMED,    // 45mA @ 80MHz, 15% backlight PWM
    DEEP_SLEEP // <1.2mA, panel off, FT3267 wake-on-touch
};

class PowerManager {
  public:
    static constexpr uint32_t INACTIVITY_DIM_MS = 45000;    // 45 seconds
    static constexpr uint32_t INACTIVITY_SLEEP_MS = 300000; // 5 minutes

    explicit PowerManager(Driver *driver);
    void setup();
    void loop(bool isProcessActive, int systemMode);
    void recordUserActivity();
    void enterState(DisplayPowerState newState);
    void enterDeepSleep();
    DisplayPowerState getState() const { return currentState_; }
    uint16_t getBatteryMillivolts();
    uint8_t getBatteryPercent();

    void setActiveBrightness(uint8_t brightness) { activeBrightness_ = brightness; }
    uint8_t getActiveBrightness() const { return activeBrightness_; }

    void onStandbyMessage() { enterDeepSleep(); }

  private:
    Driver *driver_{nullptr};
    DisplayPowerState currentState_{DisplayPowerState::ACTIVE};
    uint32_t lastActivityTime_{0};
    uint8_t activeBrightness_{16};
};

#endif // GAGGIMATE_HEADLESS
