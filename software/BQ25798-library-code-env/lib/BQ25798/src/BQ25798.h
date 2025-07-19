#ifndef BQ25798_h
#define BQ25798_h

#include <Arduino.h>
#include <Wire.h>

class BQ25798
{
private:
    uint8_t _i2cAddress;
    TwoWire *_wire;

    struct RegisterEntry
    {
        uint8_t address;
        const char *name;
    };

    static const RegisterEntry registers[58];

public:
    BQ25798(uint8_t i2cAddress = 0x6B);

    void begin(TwoWire &wirePort = Wire);

    uint8_t readRegister(uint8_t reg);
    void readAllRegisters();
};

#endif