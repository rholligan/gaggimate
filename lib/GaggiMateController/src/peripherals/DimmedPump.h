#ifndef DIMMEDPUMP_H
#define DIMMEDPUMP_H
#if __has_include("PSM.h")
#include "PSM.h"
#else
class PSM {
  public:
    PSM(unsigned char, unsigned char, unsigned int, int = 0, unsigned char = 1, unsigned char = 0) {}
    void set(unsigned int) {}
    unsigned int cps() { return 50; }
    void setDivider(unsigned char) {}
};
#endif
#include "PressureController/PressureController.h"
#include "PressureSensor.h"
#include "Pump.h"
#include <Arduino.h>

class DimmedPump : public Pump {
  public:
    enum class ControlMode { POWER, PRESSURE, FLOW };

    static constexpr float PREINFUSION_PRESSURE_BAR = 2.5f;
    static constexpr float PREINFUSION_TRANSITION_WINDOW = 0.5f; // 2.5 to 3.0 bar
    static constexpr float PREINFUSION_DIVIDER = 4.0f;           // N_skip = 4 -> 12.5 Hz on 50 Hz mains
    static constexpr uint8_t PREINFUSION_MIN_IDLE_CROSSINGS = 2;

    static float shapeBresenhamPreinfusion(float duty, float pressure);

    DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressureSensor, PressureControlRate controlRate);
    ~DimmedPump() = default;

    void setup() override;
    void loop() override;
    void setPower(float setpoint) override;

    float getCoffeeVolume();
    float getPumpFlow();
    float getPuckFlow();
    float getPuckResistance();
    float getPressureTarget() { return _ctrlPressure; }
    float getFlowTarget() { return _ctrlFlow; }
    float *getPumpPowerPtr() override { return &_power; } // For external pump control
    float getPowerTarget() const override { return _power; }
    float *getPumpFlowPtr() { return &_currentFlow; } // For thermal feedforward
    float getPumpedWater() { return _pumpedWater; }
    int *getValveStatusPtr() { return &_valveStatus; } // For thermal feedforward valve state
    bool isPreinfusionActive() const { return _currentPressure < PREINFUSION_PRESSURE_BAR && _power > 0.0f; }
    void tare();

    void setFlowTarget(float targetFlow, float pressureLimit);
    void setPressureTarget(float targetPressure, float flowLimit);
    void setPumpFlowCoeff(float oneBarFlow, float nineBarFlow);
    void setPumpFlowPolyCoeffs(float a, float b, float c, float d);
    void setPumpSlipPolyCoeffs(float a, float b, float c, float d);
    void setGains(float commutationGain, float convergenceGain, float integralGain);
    void stop();
    void fullPower();
    void setValveState(bool open);
    void setBinaryMode(bool binaryMode);

  private:
    uint8_t _ssr_pin;
    uint8_t _sense_pin;
    PSM _psm;
    PressureSensor *_pressureSensor;
    const PressureControlRate _controlRate;
    PressureController _pressureController;

    xTaskHandle taskHandle;

    ControlMode _mode = ControlMode::POWER;
    float _power = 0.0f;
    float _controllerPower = 0.0f;
    float _ctrlPressure = 0.0f;
    float _ctrlFlow = 0.0f;
    float _currentPressure = 0.0f;
    float _currentFlow = 0.0f;
    float _lastPressure = 0.0f;
    float _pumpedWater = 0.0f;
    int _valveStatus = 0;
    int _cps = MAX_FREQ;
    bool _binaryMode = false;

    float _opvPressure = 0.0f;

    static constexpr float BASE_FLOW_RATE = 0.25f;
    static constexpr float MAX_PRESSURE = 15.0f;
    static constexpr float MAX_FREQ = 60.0f;

    void updatePower(bool freshPressure, float sampleTime, float elapsed);
    int64_t _lastUpdateUs = 0;
    void onPressureUpdate(float pressure);

    const char *LOG_TAG = "DimmedPump";
    static void loopTask(void *arg);
};

#endif // DIMMEDPUMP_H
