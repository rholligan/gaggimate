#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#if __has_include(<control_physics/AutoCalibrator.hpp>)
#include <control_physics/AutoCalibrator.hpp>
#include <control_physics/ComplianceModel.hpp>
#include <control_physics/FlashSteamEngine.hpp>
#include <control_physics/PumpSurface.hpp>
#include <control_physics/ThermalObserver.hpp>
#else
#include "../../../control_physics/AutoCalibrator.hpp"
#include "../../../control_physics/ComplianceModel.hpp"
#include "../../../control_physics/FlashSteamEngine.hpp"
#include "../../../control_physics/PumpSurface.hpp"
#include "../../../control_physics/ThermalObserver.hpp"
#endif

#include <peripherals/DimmedPump.h>
#include <peripherals/PressureSensor.h>
#include <peripherals/SimpleRelay.h>
#include <peripherals/TemperatureSensor.h>

#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

class PhysicsControlCoordinator {
  public:
    enum class ControlMode : uint8_t { FLOW = 0, PRESSURE = 1, POWER = 2 };

    PhysicsControlCoordinator(TemperatureSensor *tempSensor, PressureSensor *pressureSensor, DimmedPump *pump, SimpleRelay *valve,
                              uint8_t ssrPin);
    ~PhysicsControlCoordinator();

    void setup();
    void tick50Hz(); // Invoked by 50Hz Real-Time Task

    // Mode Selection & Setpoint Routing
    void setBrewTarget(float targetTemp, float targetPressure = 9.0f, float targetFlow = 2.5f);
    void setFlowTarget(float targetFlow, float pressureLimit = 9.0f);
    void setPressureTarget(float targetPressure, float flowLimit = 2.5f);
    void setManualPumpPower(float powerPercent);
    void setSteamMode(bool enabled);
    void startHydraulicCalibration();
    void startThermalAutotune(float T_ref = 88.0f);
    void stop();
    void setValveState(bool open);

    // Telemetry getters for BLE/protobuf sensor data streaming
    [[nodiscard]] float getBoilerTempEst() const noexcept;
    [[nodiscard]] float getGroupTempEst() const noexcept;
    [[nodiscard]] float getPuckTempEst() const noexcept;
    [[nodiscard]] float getPuckFlow() const noexcept;
    [[nodiscard]] float getPumpFlow() const noexcept;
    [[nodiscard]] float getPuckResistance() const noexcept;
    [[nodiscard]] float getPressure() const noexcept;
    [[nodiscard]] float getSsrDuty() const noexcept;
    [[nodiscard]] float getPumpDuty() const noexcept;
    [[nodiscard]] bool isChannelingAutoSaveActive() const noexcept;
    [[nodiscard]] bool isEmergencyTripped() const noexcept;
    [[nodiscard]] bool isSteaming() const noexcept;
    [[nodiscard]] ControlMode getControlMode() const noexcept { return _controlMode; }

    // Direct access to underlying domain physics models
    control_physics::ThermalObserver &getThermalObserver() noexcept { return _thermalObserver; }
    const control_physics::ThermalObserver &getThermalObserver() const noexcept { return _thermalObserver; }
    control_physics::PumpSurface &getPumpSurface() noexcept { return _pumpSurface; }
    const control_physics::PumpSurface &getPumpSurface() const noexcept { return _pumpSurface; }
    control_physics::ComplianceModel &getComplianceModel() noexcept { return _complianceModel; }
    const control_physics::ComplianceModel &getComplianceModel() const noexcept { return _complianceModel; }
    control_physics::FlashSteamEngine &getSteamEngine() noexcept { return _steamEngine; }
    const control_physics::FlashSteamEngine &getSteamEngine() const noexcept { return _steamEngine; }
    control_physics::AutoCalibrator &getAutoCalibrator() noexcept { return _autoCalibrator; }
    const control_physics::AutoCalibrator &getAutoCalibrator() const noexcept { return _autoCalibrator; }

  private:
    TemperatureSensor *_tempSensor;
    PressureSensor *_pressureSensor;
    DimmedPump *_pump;
    SimpleRelay *_valve;
    uint8_t _ssrPin;

    control_physics::ThermalObserver _thermalObserver{0.02f};
    control_physics::PumpSurface _pumpSurface{0.02f};
    control_physics::ComplianceModel _complianceModel{0.02f};
    control_physics::FlashSteamEngine _steamEngine{0.02f};
    control_physics::AutoCalibrator _autoCalibrator{0.02f};

    ControlMode _controlMode = ControlMode::FLOW;
    float _targetTemp = 93.0f;
    float _targetPressure = 9.0f;
    float _targetFlow = 2.5f;
    float _manualPumpPower = 0.0f;
    bool _isSteaming = false;

    float _lastPumpDuty = 0.0f;
    float _lastPumpFlow = 0.0f;
    float _lastPuckFlow = 0.0f;
    float _lastSsrDuty = 0.0f;
    float _ssrAccumulator = 0.0f;
    bool _emergencyTripped = false;

#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    TaskHandle_t _taskHandle = nullptr;
    static void loopTask(void *arg);
#endif

    void executeFailOpenGuard(float T_sens);
    void applySsrDuty(float duty);
};
