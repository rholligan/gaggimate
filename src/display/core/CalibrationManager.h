#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include "PressureController/PumpSurfaceModel.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>

// 2-Step Automated Pump Calibration Wizard:
// Step 1: Open flow sweep across phase-cut duty percentages (20%, 40%, 60%, 80%, 100%)
//         measuring Qmax(Duty) via BLE scale dm/dt.
// Step 2: Blind basket stall sweep across discrete duty percentages polling the 0-5V transducer
//         to determine Pstall(Duty).
class CalibrationManager {
  public:
    enum class State { IDLE, SWEEP_OPEN_FLOW, AWAIT_BLIND_BASKET, SWEEP_STALL, COMPUTING_FIT, SAVED, ERROR_ABORT };

    static constexpr size_t NUM_POINTS = PumpCalibrationData::NUM_POINTS;
    static constexpr uint32_t STEP_DWELL_MS = 5000;      // 5 seconds dwell per duty node
    static constexpr uint32_t SETTLE_DWELL_MS = 1000;    // 1 second initial stabilization
    static constexpr uint32_t SAFETY_TIMEOUT_MS = 60000; // 60 seconds safety abort per phase
    static constexpr float MAX_STALL_PRESSURE = 12.0f;   // 12.0 bar maximum pressure limit

    using StateChangeCallback = std::function<void(State oldState, State newState)>;
    using CalibrationSavedCallback = std::function<void(const PumpCalibrationData &cal)>;

    CalibrationManager();
    explicit CalibrationManager(const PumpCalibrationData &initialCal);

    // Lifecycle triggers
    bool startCalibration();
    bool confirmBlindBasket();
    void abortCalibration(const char *reason = "User aborted");
    void resetToFactoryDefaults();

    // Periodic state machine update
    void update(uint32_t nowMs, float currentPressure, float scaleMass, bool scaleConnected);

    // State & status queries
    State getState() const { return _state; }
    const char *getStateString() const;
    const char *getLastError() const { return _lastError; }
    size_t getCurrentPointIndex() const { return _currentPointIndex; }
    float getActiveDutyCommand() const { return _activeDutyCommand; }
    bool isValveOpenCommand() const { return _valveOpenCommand; }
    float getStepProgressPercent(uint32_t nowMs) const;
    bool isCalibrated() const { return _isCalibrated; }
    bool isActive() const { return _state != State::IDLE && _state != State::SAVED && _state != State::ERROR_ABORT; }

    const PumpCalibrationData &getCalibratedData() const { return _cal; }
    const PumpSurfaceModel &getModel() const { return _model; }
    PumpSurfaceModel &getModel() { return _model; }

    // Direct configuration
    bool setCalibrationData(const PumpCalibrationData &cal);

    // Callbacks
    void setStateChangeCallback(StateChangeCallback cb) { _onStateChange = std::move(cb); }
    void setCalibrationSavedCallback(CalibrationSavedCallback cb) { _onSaved = std::move(cb); }

    // NVS Persistence
    bool loadFromNVS();
    bool saveToNVS();

  private:
    void transitionTo(State newState, uint32_t nowMs);
    void finishOpenFlowStep(uint32_t nowMs, float scaleMass);
    void finishStallStep(uint32_t nowMs);

    State _state = State::IDLE;
    char _lastError[96] = {0};
    bool _isCalibrated = false;

    PumpCalibrationData _cal;
    PumpSurfaceModel _model;

    // Measurement buffers
    size_t _currentPointIndex = 0;
    float _measuredQmax[NUM_POINTS] = {0.0f};
    float _measuredPstall[NUM_POINTS] = {0.0f};

    // Controller outputs
    float _activeDutyCommand = 0.0f;
    bool _valveOpenCommand = false;

    // Timing tracking
    uint32_t _phaseStartTimeMs = 0;
    uint32_t _stepStartTimeMs = 0;
    uint32_t _settleStartTimeMs = 0;
    float _stepStartMass = 0.0f;
    float _settleStartMass = 0.0f;
    bool _stepStarted = false;
    bool _phaseStarted = false;
    bool _settleStarted = false;
    bool _stepSettled = false;

    // Stall pressure averaging
    float _pressureAccum = 0.0f;
    uint32_t _pressureSampleCount = 0;

    StateChangeCallback _onStateChange;
    CalibrationSavedCallback _onSaved;
};

#endif // CALIBRATION_MANAGER_H
