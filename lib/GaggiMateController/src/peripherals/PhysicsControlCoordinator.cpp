#include "PhysicsControlCoordinator.h"

PhysicsControlCoordinator::PhysicsControlCoordinator(TemperatureSensor *tempSensor, PressureSensor *pressureSensor,
                                                     DimmedPump *pump, SimpleRelay *valve, uint8_t ssrPin)
    : _tempSensor(tempSensor), _pressureSensor(pressureSensor), _pump(pump), _valve(valve), _ssrPin(ssrPin) {}

PhysicsControlCoordinator::~PhysicsControlCoordinator() {
    stop();
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    if (_taskHandle != nullptr) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
#endif
}

void PhysicsControlCoordinator::setup() {
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    pinMode(_ssrPin, OUTPUT);
    digitalWrite(_ssrPin, LOW);
#endif

    _thermalObserver.reset();
    _pumpSurface.resetCoil();
    _complianceModel.reset();
    _steamEngine.reset();
    _autoCalibrator.reset();

    _lastPumpDuty = 0.0f;
    _lastPumpFlow = 0.0f;
    _lastPuckFlow = 0.0f;
    _lastSsrDuty = 0.0f;
    _ssrAccumulator = 0.0f;
    _emergencyTripped = false;

#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    if (_taskHandle == nullptr) {
        xTaskCreate(loopTask, "PhysicsCoordinator::task", configMINIMAL_STACK_SIZE * 4, this, 10, &_taskHandle);
    }
#endif
}

#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
void PhysicsControlCoordinator::loopTask(void *arg) {
    auto *coord = static_cast<PhysicsControlCoordinator *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        coord->tick50Hz();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20)); // Synchronous 50Hz (20ms) interval
    }
}
#endif

void PhysicsControlCoordinator::setBrewTarget(float targetTemp, float targetPressure, float targetFlow) {
    _targetTemp = targetTemp;
    _targetPressure = targetPressure;
    _targetFlow = targetFlow;
}

void PhysicsControlCoordinator::setFlowTarget(float targetFlow, float pressureLimit) {
    _controlMode = ControlMode::FLOW;
    _targetFlow = targetFlow;
    _targetPressure = pressureLimit;
}

void PhysicsControlCoordinator::setPressureTarget(float targetPressure, float flowLimit) {
    _controlMode = ControlMode::PRESSURE;
    _targetPressure = targetPressure;
    _targetFlow = flowLimit;
}

void PhysicsControlCoordinator::setManualPumpPower(float powerPercent) {
    _controlMode = ControlMode::POWER;
    _manualPumpPower = std::clamp(powerPercent, 0.0f, 100.0f);
}

void PhysicsControlCoordinator::setSteamMode(bool enabled) {
    _isSteaming = enabled;
    if (enabled) {
        _steamEngine.startSteamMode();
    } else {
        _steamEngine.stopSteamMode();
    }
}

void PhysicsControlCoordinator::startHydraulicCalibration() { _autoCalibrator.startHydraulicCalibration(); }

void PhysicsControlCoordinator::startThermalAutotune(float T_ref) { _autoCalibrator.startThermalAutotune(T_ref); }

void PhysicsControlCoordinator::stop() {
    _targetFlow = 0.0f;
    _lastPumpDuty = 0.0f;
    _lastSsrDuty = 0.0f;
    applySsrDuty(0.0f);
    if (_pump) {
        _pump->stop();
    }
    if (_valve) {
        _valve->set(false);
    }
}

void PhysicsControlCoordinator::setValveState(bool open) {
    if (_valve) {
        _valve->set(open);
    }
}

void PhysicsControlCoordinator::executeFailOpenGuard(float T_sens) {
    _emergencyTripped = true;
    _lastSsrDuty = 0.0f;
    _lastPumpDuty = 0.0f;
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
    digitalWrite(_ssrPin, LOW);
#endif
    if (_pump) {
        _pump->stop();
    }
    if (_valve) {
        _valve->set(false);
    }
    _thermalObserver.reset();
}

void PhysicsControlCoordinator::applySsrDuty(float duty) {
    if (_emergencyTripped || duty <= 0.0f) {
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
        digitalWrite(_ssrPin, LOW);
#endif
        _ssrAccumulator = 0.0f;
        return;
    }
    if (duty >= 1.0f) {
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
        digitalWrite(_ssrPin, HIGH);
#endif
        _ssrAccumulator = 0.0f;
        return;
    }

    _ssrAccumulator += duty;
    if (_ssrAccumulator >= 1.0f) {
        _ssrAccumulator -= 1.0f;
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
        digitalWrite(_ssrPin, HIGH);
#endif
    } else {
#if defined(ARDUINO) || defined(ESP32) || defined(ESP_PLATFORM)
        digitalWrite(_ssrPin, LOW);
#endif
    }
}

void PhysicsControlCoordinator::tick50Hz() {
    // a. Sample pressure transducer and temperature sensor
    float P = _pressureSensor ? _pressureSensor->getPressure() : 0.0f;
    float T_sens = _tempSensor ? _tempSensor->read() : 0.0f;
    bool sensorFault = _tempSensor ? _tempSensor->isErrorState() : false;

    // b. Fail-open sensor guard: if T_sens > 155.0°C or isnan/isinf or thermocouple fault bit set,
    // immediately force SSR GPIO LOW, stop pump, close valve, and trip safety.
    if (std::isnan(T_sens) || std::isinf(T_sens) || T_sens > 155.0f || sensorFault) {
        executeFailOpenGuard(T_sens);
        return;
    }

    // c. Update solenoid coil thermal fade
    _pumpSurface.updateThermalCoil(_lastPumpDuty);

    // d. Calculate raw pump displacement flow Q_raw = predictFlow(P, duty)
    float Q_raw = _pumpSurface.predictFlow(P, _lastPumpDuty);
    _lastPumpFlow = Q_raw;

    // e. Update dynamic compliance and real-time Darcy puck resistance Q_puck = update(P, Q_raw)
    float Q_puck = _complianceModel.update(P, Q_raw);
    _lastPuckFlow = Q_puck;

    // f. Auto-Save channeling guard: if channeling auto-save trips (d ln R/dt < -0.18 s^-1),
    // clamp commanded flow to 1.8 ml/s
    float activeTargetFlow = _targetFlow;
    if (_complianceModel.isAutoSaveActive()) {
        activeTargetFlow = std::min(activeTargetFlow, control_physics::ComplianceModel::kAutoSaveFlowTarget);
    }

    bool isValveOpen = _valve ? _valve->getState() : false;
    float pumpDuty = 0.0f;
    float ssrDuty = 0.0f;

    if (_isSteaming) {
        float steamHeaterDuty = 0.0f;
        float steamTargetFlow = 0.0f;
        float dP_dt = _complianceModel.getPressureDerivative();
        _steamEngine.update(_thermalObserver.getBoilerTempEstimate(), P, dP_dt, steamHeaterDuty, steamTargetFlow);
        ssrDuty = steamHeaterDuty;
        if (steamTargetFlow > 0.0f) {
            pumpDuty = _pumpSurface.invertDuty(steamTargetFlow, P);
        } else {
            pumpDuty = 0.0f;
        }
    } else if (_autoCalibrator.getHydraulicState() != control_physics::AutoCalibrator::HydraulicState::IDLE &&
               _autoCalibrator.getHydraulicState() != control_physics::AutoCalibrator::HydraulicState::COMPLETE &&
               _autoCalibrator.getHydraulicState() != control_physics::AutoCalibrator::HydraulicState::ABORTED) {
        float calPumpDuty = 0.0f;
        _autoCalibrator.updateHydraulic(P, Q_raw, calPumpDuty);
        pumpDuty = calPumpDuty;
        ssrDuty = _thermalObserver.update(T_sens, _targetTemp, Q_raw, isValveOpen);
        if (_autoCalibrator.getHydraulicState() == control_physics::AutoCalibrator::HydraulicState::COMPLETE) {
            const auto &res = _autoCalibrator.getHydraulicResults();
            if (res.valid) {
                _pumpSurface.setQMaxPoly(res.qMax_c2, res.qMax_c1, res.qMax_c0);
                _pumpSurface.setPStallPoly(res.pStall_c2, res.pStall_c1, res.pStall_c0);
                _complianceModel.setComplianceParams(res.V_air0, res.c_hose);
            }
        }
    } else if (_autoCalibrator.getThermalState() != control_physics::AutoCalibrator::ThermalState::IDLE &&
               _autoCalibrator.getThermalState() != control_physics::AutoCalibrator::ThermalState::COMPLETE &&
               _autoCalibrator.getThermalState() != control_physics::AutoCalibrator::ThermalState::ABORTED) {
        float calSsrDuty = 0.0f;
        _autoCalibrator.updateThermal(T_sens, calSsrDuty);
        ssrDuty = calSsrDuty;
        pumpDuty = 0.0f;
        if (_autoCalibrator.getThermalState() == control_physics::AutoCalibrator::ThermalState::COMPLETE) {
            const auto &res = _autoCalibrator.getThermalResults();
            if (res.valid) {
                _thermalObserver.setGains(res.Kp, res.Ti, res.Td);
            }
        }
    } else {
        // Normal extraction / idle
        if (isValveOpen) {
            if (_controlMode == ControlMode::FLOW) {
                pumpDuty = _pumpSurface.invertDuty(activeTargetFlow, P);
            } else if (_controlMode == ControlMode::PRESSURE) {
                float flowTarget = (P < _targetPressure) ? activeTargetFlow : 0.0f;
                pumpDuty = _pumpSurface.invertDuty(flowTarget, P);
            } else {
                pumpDuty = _manualPumpPower / 100.0f;
            }

            // Low-pressure Bresenham pre-infusion shaping for P < 2.5 bar
            if (P < 2.5f) {
                pumpDuty = DimmedPump::shapeBresenhamPreinfusion(pumpDuty, P);
            }
        } else {
            pumpDuty = 0.0f;
        }

        // g. Zero-lag feed-forward: synchronously update thermal observer within the same 50Hz tick (0 ms latency!)
        ssrDuty = _thermalObserver.update(T_sens, _targetTemp, Q_raw, isValveOpen);
    }

    _lastPumpDuty = pumpDuty;
    if (_pump) {
        _pump->setPower(pumpDuty * 100.0f);
    }

    // h. Apply SSR duty to heater GPIO
    _lastSsrDuty = ssrDuty;
    applySsrDuty(ssrDuty);
}

float PhysicsControlCoordinator::getBoilerTempEst() const noexcept { return _thermalObserver.getBoilerTempEstimate(); }

float PhysicsControlCoordinator::getGroupTempEst() const noexcept { return _thermalObserver.getGroupTempEstimate(); }

float PhysicsControlCoordinator::getPuckTempEst() const noexcept { return _thermalObserver.estimatePuckTemperature(); }

float PhysicsControlCoordinator::getPuckFlow() const noexcept { return _lastPuckFlow; }

float PhysicsControlCoordinator::getPumpFlow() const noexcept { return _lastPumpFlow; }

float PhysicsControlCoordinator::getPuckResistance() const noexcept { return _complianceModel.getPuckResistance(); }

float PhysicsControlCoordinator::getPressure() const noexcept { return _pressureSensor ? _pressureSensor->getPressure() : 0.0f; }

float PhysicsControlCoordinator::getSsrDuty() const noexcept { return _lastSsrDuty; }

float PhysicsControlCoordinator::getPumpDuty() const noexcept { return _lastPumpDuty; }

bool PhysicsControlCoordinator::isChannelingAutoSaveActive() const noexcept { return _complianceModel.isAutoSaveActive(); }

bool PhysicsControlCoordinator::isEmergencyTripped() const noexcept { return _emergencyTripped; }

bool PhysicsControlCoordinator::isSteaming() const noexcept { return _isSteaming; }
