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
        bool is16bit;
    };

    static const RegisterEntry registers[];
    static const uint8_t registerCount;
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegister8(uint8_t reg);
    uint16_t readRegister16(uint8_t reg);
    void decodeRegister(uint8_t reg, uint16_t value);

    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegister16(uint8_t reg, uint16_t value);

public:
    BQ25798(uint8_t i2cAddress = 0x6B);

    void begin(TwoWire &wirePort = Wire, uint32_t clockSpeed = 100000);

    void scanI2C(TwoWire &wirePort = Wire);

    uint16_t readSingleRegister(uint8_t reg, bool outputBinary = false);
    void printAllRegisters(bool outputBinary = false);

    void setSingleRegister(uint8_t reg, uint16_t bits);
};

#endif