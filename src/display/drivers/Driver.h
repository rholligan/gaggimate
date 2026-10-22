
#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <cstdint>

class Driver {
  public:
    virtual ~Driver() = default;

    virtual bool isCompatible();
    virtual void init();
    virtual void setBrightness(int brightness);
    virtual bool supportsSDCard();
    virtual bool installSDCard();

    virtual void sleep() {}
    virtual void wakeup() {}
    virtual uint16_t getBatteryVoltage() { return 0; }
    virtual void setLowPowerFrequency(bool lowPower) {
#if !defined(GAGGIMATE_SIM)
        setCpuFrequencyMhz(lowPower ? 80 : 240);
#endif
    }
};

#endif // DRIVER_H
