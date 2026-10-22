#ifndef GAGGIMATE_HEADLESS

#include "PowerManager.h"
#include <esp_sleep.h>
#if !defined(GAGGIMATE_SIM)
#include <lvgl.h>
#endif

PowerManager::PowerManager(Driver *driver) : driver_(driver) {}

void PowerManager::setup() {
    lastActivityTime_ = millis();
    currentState_ = DisplayPowerState::ACTIVE;
    if (driver_ != nullptr) {
        driver_->setLowPowerFrequency(false);
        driver_->setBrightness(activeBrightness_);
    }
}

void PowerManager::recordUserActivity() {
    lastActivityTime_ = millis();
    if (currentState_ == DisplayPowerState::DIMMED) {
        enterState(DisplayPowerState::ACTIVE);
    }
}

void PowerManager::enterState(DisplayPowerState newState) {
    if (currentState_ == newState) {
        return;
    }

    switch (newState) {
    case DisplayPowerState::ACTIVE:
        if (driver_ != nullptr) {
            driver_->setLowPowerFrequency(false); // 240MHz
            driver_->setBrightness(activeBrightness_);
            driver_->wakeup();
        }
        currentState_ = DisplayPowerState::ACTIVE;
        break;

    case DisplayPowerState::DIMMED:
        if (driver_ != nullptr) {
            // Set brightness to 15% (e.g. 2 out of 16)
            uint8_t dimmedBrightness = std::max(1, static_cast<int>(activeBrightness_ * 0.15f + 0.5f));
            driver_->setBrightness(dimmedBrightness);
            driver_->setLowPowerFrequency(true); // 80MHz
        }
        currentState_ = DisplayPowerState::DIMMED;
        break;

    case DisplayPowerState::DEEP_SLEEP:
        enterDeepSleep();
        break;
    }
}

void PowerManager::enterDeepSleep() {
    currentState_ = DisplayPowerState::DEEP_SLEEP;
    if (driver_ != nullptr) {
        driver_->sleep();
    }
#if !defined(GAGGIMATE_SIM)
    // Direct hardware fallback if driver sleep returns without entering sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_1, 0);
    esp_deep_sleep_start();
#endif
}

void PowerManager::loop(bool isProcessActive, int systemMode) {
    uint32_t now = millis();

    // Active brewing or processes keep the remote awake
    if (isProcessActive) {
        lastActivityTime_ = now;
        if (currentState_ == DisplayPowerState::DIMMED) {
            enterState(DisplayPowerState::ACTIVE);
        }
        return;
    }

    // Check LVGL display input activity if LVGL is running
#if !defined(GAGGIMATE_SIM)
    uint32_t lvglInactive = lv_disp_get_inactive_time(NULL);
    if (lvglInactive < 200) {
        recordUserActivity();
    }
#endif

    uint32_t idleMs = now - lastActivityTime_;

    if (currentState_ == DisplayPowerState::ACTIVE) {
        if (idleMs >= INACTIVITY_SLEEP_MS) {
            enterDeepSleep();
        } else if (idleMs >= INACTIVITY_DIM_MS) {
            enterState(DisplayPowerState::DIMMED);
        }
    } else if (currentState_ == DisplayPowerState::DIMMED) {
        if (idleMs >= INACTIVITY_SLEEP_MS) {
            enterDeepSleep();
        }
    }
}

uint16_t PowerManager::getBatteryMillivolts() {
    if (driver_ != nullptr) {
        return driver_->getBatteryVoltage();
    }
    return 0;
}

uint8_t PowerManager::getBatteryPercent() {
    uint16_t mv = getBatteryMillivolts();
    if (mv == 0) {
        return 100;
    }
    if (mv <= 3200) {
        return 0;
    }
    if (mv >= 4200) {
        return 100;
    }
    return static_cast<uint8_t>((mv - 3200) * 100 / 1000);
}

#endif // GAGGIMATE_HEADLESS
