#include <Arduino.h>
#include "BQ25798.h"

BQ25798::BQ25798(uint8_t i2cAddress)
{
    this->_i2cAddress = i2cAddress;
}

/*
Sets the library internal values for the I2C port and clock speed
i2C port, speed in Hz standard value is 100kHz, arduino max. 400kHz
- Standard-Mode (Sm), with a bit rate up to 100 kbit/s
- Fast-Mode (Fm), with a bit rate up to 400 kbit/s
- Fast-Mode Plus (Fm+), with a bit rate up to 1 Mbit/s
- High-speed Mode (Hs-mode), with a bit rate up to 3.4 Mbit/s
*/
void BQ25798::begin(TwoWire &wirePort, uint32_t clockSpeed)
{
    _wire = &wirePort;
    _wire->begin();
    _wire->setClock(clockSpeed);
}

/*
Scans all I2C addresses on the port, no begin needed.
Prints all found devices out on Serial (Serial begin needed in code).
*/
void BQ25798::scanI2C(TwoWire &wirePort)
{
    _wire = &wirePort;
    _wire->begin();

    Serial.println(F("Scanning I2C bus..."));

    for (uint8_t address = 1; address < 127; address++)
    {
        _wire->beginTransmission(address);
        uint8_t error = _wire->endTransmission();

        if (error == 0)
        {
            Serial.print(F("Device found at 0x"));
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }

    Serial.println(F("Scan complete."));
}

// reads a single register in the BQ25798 and returns its value as a Uint with 8 Bit
uint8_t BQ25798::readRegister(uint8_t reg)
{
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);

    _wire->requestFrom(_i2cAddress, (uint8_t)1);
    if (_wire->available())
    {
        return _wire->read();
    }
    return 0xFF; // Error value
}

// reads all registers of the BQ25798 and prints them to Serial
void BQ25798::readAllRegisters()
{
    Serial.println(F("Reading BQ25798 Registers:"));
    for (uint8_t i = 0; i < 58; i++)
    {
        uint8_t reg = registers[i].address;
        const char *name = registers[i].name;
        uint8_t value = readRegister(reg);

        Serial.print(F("0x"));
        if (reg < 0x10)
            Serial.print('0');
        Serial.print(reg, HEX);
        Serial.print(" : ");
        Serial.print(name);
        Serial.print(" : 0x");
        if (value < 0x10)
            Serial.print('0');
        Serial.println(value, HEX);
    }
}

/*
Library internal struct for all registers and their functions
*/
const BQ25798::RegisterEntry BQ25798::registers[58] = {
    {0x00, "REG00_Minimal_System_Voltage"},
    {0x01, "REG01_Charge_Voltage_Limit"},
    {0x03, "REG03_Charge_Current_Limit"},
    {0x05, "REG05_Input_Voltage_Limit"},
    {0x06, "REG06_Input_Current_Limit"},
    {0x08, "REG08_Precharge_Control"},
    {0x09, "REG09_Termination_Control"},
    {0x0A, "REG0A_Re-charge_Control"},
    {0x0B, "REG0B_VOTG_regulation"},
    {0x0D, "REG0D_IOTG_regulation"},
    {0x0E, "REG0E_Timer_Control"},
    {0x0F, "REG0F_Charger_Control_0"},
    {0x10, "REG10_Charger_Control_1"},
    {0x11, "REG11_Charger_Control_2"},
    {0x12, "REG12_Charger_Control_3"},
    {0x13, "REG13_Charger_Control_4"},
    {0x14, "REG14_Charger_Control_5"},
    {0x15, "REG15_MPPT_Control"},
    {0x16, "REG16_Temperature_Control"},
    {0x17, "REG17_NTC_Control_0"},
    {0x18, "REG18_NTC_Control_1"},
    {0x19, "REG19_ICO_Current_Limit"},
    {0x1B, "REG1B_Charger_Status_0"},
    {0x1C, "REG1C_Charger_Status_1"},
    {0x1D, "REG1D_Charger_Status_2"},
    {0x1E, "REG1E_Charger_Status_3"},
    {0x1F, "REG1F_Charger_Status_4"},
    {0x20, "REG20_FAULT_Status_0"},
    {0x21, "REG21_FAULT_Status_1"},
    {0x22, "REG22_Charger_Flag_0"},
    {0x23, "REG23_Charger_Flag_1"},
    {0x24, "REG24_Charger_Flag_2"},
    {0x25, "REG25_Charger_Flag_3"},
    {0x26, "REG26_FAULT_Flag_0"},
    {0x27, "REG27_FAULT_Flag_1"},
    {0x28, "REG28_Charger_Mask_0"},
    {0x29, "REG29_Charger_Mask_1"},
    {0x2A, "REG2A_Charger_Mask_2"},
    {0x2B, "REG2B_Charger_Mask_3"},
    {0x2C, "REG2C_FAULT_Mask_0"},
    {0x2D, "REG2D_FAULT_Mask_1"},
    {0x2E, "REG2E_ADC_Control"},
    {0x2F, "REG2F_ADC_Function_Disable_0"},
    {0x30, "REG30_ADC_Function_Disable_1"},
    {0x31, "REG31_IBUS_ADC"},
    {0x33, "REG33_IBAT_ADC"},
    {0x35, "REG35_VBUS_ADC"},
    {0x37, "REG37_VAC1_ADC"},
    {0x39, "REG39_VAC2_ADC"},
    {0x3B, "REG3B_VBAT_ADC"},
    {0x3D, "REG3D_VSYS_ADC"},
    {0x3F, "REG3F_TS_ADC"},
    {0x41, "REG41_TDIE_ADC"},
    {0x43, "REG43_D+_ADC"},
    {0x45, "REG45_D-_ADC"},
    {0x47, "REG47_DPDM_Driver"},
    {0x48, "REG48_Part_Information"}};