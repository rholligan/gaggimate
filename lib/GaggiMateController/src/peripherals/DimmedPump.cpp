#include "DimmedPump.h"

#include <ExtensionIOXL9555.hpp>
#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor, PressureControlRate controlRate)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), _psm(_sense_pin, _ssr_pin, 100, FALLING, 1, 4), _pressureSensor(pressure_sensor),
      _controlRate(controlRate), _pressureController(pressureControlPeriodS(controlRate), &_ctrlPressure, &_ctrlFlow,
                                                     &_currentPressure, &_controllerPower, &_valveStatus) {
    _psm.set(0);
}

void DimmedPump::setup() {
    _cps = _psm.cps();
    ESP_LOGI("DimmedPump", "Detected ZC frequency: %dHz", _cps);
    if (_cps > 70) {
        _cps = _cps / 2;
        _psm.setDivider(2);
        ESP_LOGI("DimmedPump", "Detected 2 signals per wave, dividing down to %dHz", _cps);
    }
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void DimmedPump::loop() {
    const int64_t now = esp_timer_get_time();
    const float elapsed = _lastUpdateUs ? (now - _lastUpdateUs) / 1000000.0f : pressureControlPeriodS(_controlRate);
    _lastUpdateUs = now;
    const bool fresh = _pressureSensor->consumeAverage();
    if (fresh)
        _currentPressure = _pressureSensor->getAveragedPressure();
    updatePower(fresh, _pressureSensor->getSampleTime(), elapsed);
    _currentFlow = _pressureController.getPumpFlowRate();
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _ctrlPressure = setpoint > 0 ? 20.0f : 0.0f;
    _mode = ControlMode::POWER;
    _power = std::clamp(setpoint, 0.0f, 100.0f);
    _controllerPower = _power; // Feed manual control back into pressure controller
    _ctrlFlow = 0.0f;
    _ctrlPressure = 0.0f;
    if (_power == 0.0f) {
        _currentFlow = 0.0f;
    }
    const float shapedDuty = shapeBresenhamPreinfusion(_power / 100.0f, _currentPressure);
    const float effPower = shapedDuty * 100.0f;
    _psm.set(static_cast<int>(_binaryMode ? (_power > 0 ? 100.0f : 0.0f) : std::round(effPower)));
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getCoffeeOutputEstimate(); }

float DimmedPump::getPumpFlow() { return _currentFlow; }

float DimmedPump::getPuckFlow() { return _pressureController.getCoffeeFlowRate(); }

float DimmedPump::getPuckResistance() { return _pressureController.getPuckResistance(); }

void DimmedPump::tare() {
    _pumpedWater = 0.0f;
    _pressureController.tare();
    _pressureController.reset();
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    uint8_t intervalIndex = 0;
    while (true) {
        pump->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(pressureControlIntervalMs(pump->_controlRate, intervalIndex)));
        intervalIndex = (intervalIndex + 1) % 3;
    }
}

void DimmedPump::updatePower(bool freshPressure, float sampleTime, float elapsed) {
    _pumpedWater += _currentFlow * elapsed;
    _pressureController.update(static_cast<PressureController::ControlMode>(_mode), freshPressure, sampleTime, elapsed);
    if (_mode != ControlMode::POWER) {
        _power = _controllerPower;
    }
    const float shapedDuty = shapeBresenhamPreinfusion(_power / 100.0f, _currentPressure);
    const float effPower = shapedDuty * 100.0f;
    if (_binaryMode) {
        int binaryPower = (_controllerPower > 0 || _ctrlPressure > 0 || _ctrlFlow > 0) ? 100 : 0;
        _psm.set(binaryPower);
    } else {
        _psm.set(static_cast<int>(std::round(effPower)));
    }
}

float DimmedPump::shapeBresenhamPreinfusion(float duty, float pressure) {
    if (duty <= 0.0f) {
        return 0.0f;
    }
    // Bresenham sub-harmonic half-wave skipping for pre-infusion when P < 2.5 bar:
    // divider N_skip = 4 -> 12.5 Hz on 50 Hz mains with minimum 2 idle zero-crossings between pulses.
    if (pressure < PREINFUSION_PRESSURE_BAR) {
        return duty / PREINFUSION_DIVIDER; // Scales duty to [0.0, 0.25] -> max 1 pulse per 4 zero-crossings
    } else if (pressure < (PREINFUSION_PRESSURE_BAR + PREINFUSION_TRANSITION_WINDOW)) {
        // Smooth linear transition between 2.5 bar and 3.0 bar into continuous closed-loop modulation
        const float blend = (pressure - PREINFUSION_PRESSURE_BAR) / PREINFUSION_TRANSITION_WINDOW;
        const float preinfusionDuty = duty / PREINFUSION_DIVIDER;
        return preinfusionDuty + blend * (duty - preinfusionDuty);
    } else {
        // Continuous closed-loop duty modulation
        return duty;
    }
}

void DimmedPump::stop() { setPower(0.0f); }

void DimmedPump::fullPower() { setPower(100.0f); }

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlFlow = targetFlow;
    _ctrlPressure = pressureLimit;
    _pressureController.setPressureLimit(pressureLimit);
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlFlow = flowLimit;
    _ctrlPressure = targetPressure;
    _pressureController.setFlowLimit(flowLimit);
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

void DimmedPump::setBinaryMode(bool binaryMode) { _binaryMode = binaryMode; }

void DimmedPump::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    _pressureController.setPumpFlowCoeff(oneBarFlow, nineBarFlow);
}

void DimmedPump::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpFlowPolyCoeffs(a, b, c, d);
}

void DimmedPump::setPumpSlipPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpSlipPolyCoeffs(a, b, c, d);
}

void DimmedPump::setGains(float commutationGain, float convergenceGain, float integralGain) {
    _pressureController.setGains(commutationGain, convergenceGain, integralGain);
}
