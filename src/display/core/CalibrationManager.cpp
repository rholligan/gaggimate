#include "CalibrationManager.h"

#if defined(ARDUINO) || defined(ESP32)
#include <Preferences.h>
#elif defined(GAGGIMATE_SIM)
#include "Preferences.h"
#endif

CalibrationManager::CalibrationManager() : _cal(), _model() {}

CalibrationManager::CalibrationManager(const PumpCalibrationData &initialCal)
    : _cal(initialCal.isValid() ? initialCal : PumpCalibrationData()),
      _model(_cal),
      _isCalibrated(initialCal.isValid()) {}

const char *CalibrationManager::getStateString() const {
    switch (_state) {
    case State::IDLE:
        return "IDLE";
    case State::SWEEP_OPEN_FLOW:
        return "SWEEP_OPEN_FLOW";
    case State::AWAIT_BLIND_BASKET:
        return "AWAIT_BLIND_BASKET";
    case State::SWEEP_STALL:
        return "SWEEP_STALL";
    case State::COMPUTING_FIT:
        return "COMPUTING_FIT";
    case State::SAVED:
        return "SAVED";
    case State::ERROR_ABORT:
        return "ERROR_ABORT";
    default:
        return "UNKNOWN";
    }
}

float CalibrationManager::getStepProgressPercent(uint32_t nowMs) const {
    if (_state != State::SWEEP_OPEN_FLOW && _state != State::SWEEP_STALL) {
        return (_state == State::SAVED) ? 100.0f : 0.0f;
    }
    if (!_stepStarted) {
        return (static_cast<float>(_currentPointIndex) / static_cast<float>(NUM_POINTS)) * 100.0f;
    }
    const uint32_t elapsed = nowMs - _stepStartTimeMs;
    const float stepFrac = std::clamp(static_cast<float>(elapsed) / static_cast<float>(STEP_DWELL_MS), 0.0f, 1.0f);
    const float overallFrac = (static_cast<float>(_currentPointIndex) + stepFrac) / static_cast<float>(NUM_POINTS);
    return overallFrac * 100.0f;
}

void CalibrationManager::transitionTo(State newState, uint32_t nowMs) {
    const State oldState = _state;
    _state = newState;
    _phaseStartTimeMs = nowMs;
    _stepStartTimeMs = nowMs;
    _settleStartTimeMs = nowMs;
    _stepSettled = false;
    _stepStarted = false;
    _phaseStarted = false;
    _settleStarted = false;
    if (_onStateChange && oldState != newState) {
        _onStateChange(oldState, newState);
    }
}

bool CalibrationManager::startCalibration() {
    if (_state != State::IDLE && _state != State::SAVED && _state != State::ERROR_ABORT) {
        return false;
    }
    _lastError[0] = '\0';
    _currentPointIndex = 0;
    _activeDutyCommand = _cal.dutyPoints[0];
    _valveOpenCommand = true;
    _pressureAccum = 0.0f;
    _pressureSampleCount = 0;
    _stepStarted = false;
    _phaseStarted = false;
    _settleStarted = false;
    _stepSettled = false;
    std::fill(std::begin(_measuredQmax), std::end(_measuredQmax), 0.0f);
    std::fill(std::begin(_measuredPstall), std::end(_measuredPstall), 0.0f);
    transitionTo(State::SWEEP_OPEN_FLOW, 0);
    return true;
}

bool CalibrationManager::confirmBlindBasket() {
    if (_state != State::AWAIT_BLIND_BASKET) {
        return false;
    }
    _currentPointIndex = 0;
    _activeDutyCommand = _cal.dutyPoints[0];
    _valveOpenCommand = true;
    _pressureAccum = 0.0f;
    _pressureSampleCount = 0;
    _stepStarted = false;
    _phaseStarted = false;
    _settleStarted = false;
    _stepSettled = false;
    transitionTo(State::SWEEP_STALL, 0);
    return true;
}

void CalibrationManager::abortCalibration(const char *reason) {
    _activeDutyCommand = 0.0f;
    _valveOpenCommand = false;
    _stepStarted = false;
    _phaseStarted = false;
    _settleStarted = false;
    _stepSettled = false;
    if (reason != nullptr) {
        std::strncpy(_lastError, reason, sizeof(_lastError) - 1);
        _lastError[sizeof(_lastError) - 1] = '\0';
    }
    transitionTo(State::ERROR_ABORT, 0);
}

void CalibrationManager::resetToFactoryDefaults() {
    _cal = PumpCalibrationData();
    _model.resetToFactoryDefaults();
    _isCalibrated = false;
    _activeDutyCommand = 0.0f;
    _valveOpenCommand = false;
    _lastError[0] = '\0';
    _stepStarted = false;
    _phaseStarted = false;
    _settleStarted = false;
    _stepSettled = false;
    transitionTo(State::IDLE, 0);
    saveToNVS();
}

bool CalibrationManager::setCalibrationData(const PumpCalibrationData &cal) {
    if (!cal.isValid()) {
        return false;
    }
    _cal = cal;
    _model.setCalibration(_cal);
    _isCalibrated = true;
    saveToNVS();
    if (_onSaved) {
        _onSaved(_cal);
    }
    return true;
}

void CalibrationManager::update(uint32_t nowMs, float currentPressure, float scaleMass, bool scaleConnected) {
    if (_state == State::SWEEP_OPEN_FLOW) {
        if (!scaleConnected) {
            abortCalibration("BLE scale disconnected during open flow sweep");
            return;
        }
        if (_phaseStarted && (nowMs - _phaseStartTimeMs) > SAFETY_TIMEOUT_MS) {
            abortCalibration("Open flow sweep safety timeout exceeded");
            return;
        }
        if (!_stepStarted) {
            _phaseStartTimeMs = nowMs;
            _stepStartTimeMs = nowMs;
            _stepStartMass = scaleMass;
            _settleStartMass = scaleMass;
            _stepStarted = true;
            _phaseStarted = true;
            _stepSettled = false;
            _settleStarted = false;
            return;
        }
        const uint32_t stepElapsed = nowMs - _stepStartTimeMs;
        if (!_stepSettled && stepElapsed >= SETTLE_DWELL_MS) {
            _stepSettled = true;
            _settleStartTimeMs = nowMs;
            _settleStarted = true;
            _settleStartMass = scaleMass;
        }
        if (stepElapsed >= STEP_DWELL_MS) {
            finishOpenFlowStep(nowMs, scaleMass);
        }
    } else if (_state == State::SWEEP_STALL) {
        if (_phaseStarted && (nowMs - _phaseStartTimeMs) > SAFETY_TIMEOUT_MS) {
            abortCalibration("Stall pressure sweep safety timeout exceeded");
            return;
        }
        if (currentPressure > MAX_STALL_PRESSURE) {
            abortCalibration("Pressure exceeded safety ceiling limit (12 bar)");
            return;
        }
        if (!_stepStarted) {
            _phaseStartTimeMs = nowMs;
            _stepStartTimeMs = nowMs;
            _pressureAccum = 0.0f;
            _pressureSampleCount = 0;
            _stepStarted = true;
            _phaseStarted = true;
            _stepSettled = false;
            _settleStarted = false;
            return;
        }
        const uint32_t stepElapsed = nowMs - _stepStartTimeMs;
        if (!_stepSettled && stepElapsed >= SETTLE_DWELL_MS) {
            _stepSettled = true;
            _settleStartTimeMs = nowMs;
            _settleStarted = true;
            _pressureAccum = 0.0f;
            _pressureSampleCount = 0;
        }
        if (_stepSettled) {
            _pressureAccum += currentPressure;
            _pressureSampleCount++;
        }
        if (stepElapsed >= STEP_DWELL_MS) {
            finishStallStep(nowMs);
        }
    }
}

void CalibrationManager::finishOpenFlowStep(uint32_t nowMs, float scaleMass) {
    const float deltaM = scaleMass - _settleStartMass;
    const uint32_t settleElapsedMs = nowMs - _settleStartTimeMs;
    const float deltaT = (_settleStarted && settleElapsedMs <= SAFETY_TIMEOUT_MS)
                             ? (static_cast<float>(settleElapsedMs) / 1000.0f)
                             : 4.0f;
    const float flow = (deltaT > 0.05f) ? (deltaM / deltaT) : 0.0f;

    // Safety check: if duty >= 40% and mass delta < 1.0g over 5s, flag low flow
    if (_cal.dutyPoints[_currentPointIndex] >= 40.0f && deltaM < 1.0f) {
        abortCalibration("Low open flow detected (<1g over 5s): water tank empty or scale error");
        return;
    }

    _measuredQmax[_currentPointIndex] = std::max(0.1f, flow);
    _currentPointIndex++;

    if (_currentPointIndex < NUM_POINTS) {
        _stepStartTimeMs = nowMs;
        _stepStarted = true;
        _stepSettled = false;
        _settleStarted = false;
        _stepStartMass = scaleMass;
        _settleStartMass = scaleMass;
        _activeDutyCommand = _cal.dutyPoints[_currentPointIndex];
    } else {
        // Completed Open Flow sweep
        _activeDutyCommand = 0.0f;
        _valveOpenCommand = false;
        transitionTo(State::AWAIT_BLIND_BASKET, nowMs);
    }
}

void CalibrationManager::finishStallStep(uint32_t nowMs) {
    const float avgP = (_pressureSampleCount > 0) ? (_pressureAccum / static_cast<float>(_pressureSampleCount)) : 0.0f;
    _measuredPstall[_currentPointIndex] = std::max(0.2f, avgP);
    _currentPointIndex++;

    if (_currentPointIndex < NUM_POINTS) {
        _stepStartTimeMs = nowMs;
        _stepStarted = true;
        _stepSettled = false;
        _settleStarted = false;
        _pressureAccum = 0.0f;
        _pressureSampleCount = 0;
        _activeDutyCommand = _cal.dutyPoints[_currentPointIndex];
    } else {
        // Completed Blind Stall sweep
        _activeDutyCommand = 0.0f;
        _valveOpenCommand = false;
        transitionTo(State::COMPUTING_FIT, nowMs);

        PumpCalibrationData candidate = _cal;
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            candidate.qMaxPoints[i] = _measuredQmax[i];
            candidate.pStallPoints[i] = _measuredPstall[i];
        }

        if (candidate.isValid()) {
            _cal = candidate;
            _model.setCalibration(_cal);
            _isCalibrated = true;
            saveToNVS();
            if (_onSaved) {
                _onSaved(_cal);
            }
            transitionTo(State::SAVED, nowMs);
        } else {
            _model.resetToFactoryDefaults();
            _isCalibrated = false;
            abortCalibration("Surface fit validation failed (non-monotonic or out of bounds)");
        }
    }
}

bool CalibrationManager::saveToNVS() {
#if (defined(ARDUINO) || defined(ESP32) || defined(GAGGIMATE_SIM))
    Preferences prefs;
    if (!prefs.begin("pump_cal", false)) {
        return false;
    }
    prefs.putBool("valid", _isCalibrated);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        char qKey[16], pKey[16];
        std::snprintf(qKey, sizeof(qKey), "q_%u", static_cast<unsigned>(i));
        std::snprintf(pKey, sizeof(pKey), "p_%u", static_cast<unsigned>(i));
        prefs.putFloat(qKey, _cal.qMaxPoints[i]);
        prefs.putFloat(pKey, _cal.pStallPoints[i]);
    }
    prefs.putFloat("comp", _cal.systemCompliance);
    prefs.putFloat("fade", _cal.thermalFadeFactor);
    prefs.end();
    return true;
#else
    return true;
#endif
}

bool CalibrationManager::loadFromNVS() {
#if (defined(ARDUINO) || defined(ESP32) || defined(GAGGIMATE_SIM))
    Preferences prefs;
    if (!prefs.begin("pump_cal", true)) {
        resetToFactoryDefaults();
        return false;
    }
    const bool valid = prefs.getBool("valid", false);
    if (!valid) {
        prefs.end();
        resetToFactoryDefaults();
        return false;
    }
    PumpCalibrationData loaded;
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        char qKey[16], pKey[16];
        std::snprintf(qKey, sizeof(qKey), "q_%u", static_cast<unsigned>(i));
        std::snprintf(pKey, sizeof(pKey), "p_%u", static_cast<unsigned>(i));
        loaded.qMaxPoints[i] = prefs.getFloat(qKey, loaded.qMaxPoints[i]);
        loaded.pStallPoints[i] = prefs.getFloat(pKey, loaded.pStallPoints[i]);
    }
    loaded.systemCompliance = prefs.getFloat("comp", loaded.systemCompliance);
    loaded.thermalFadeFactor = prefs.getFloat("fade", loaded.thermalFadeFactor);
    prefs.end();

    if (loaded.isValid()) {
        _cal = loaded;
        _model.setCalibration(_cal);
        _isCalibrated = true;
        return true;
    }
    resetToFactoryDefaults();
    return false;
#else
    return true;
#endif
}
