#include <Arduino.h>
#include "BQ25798.h"

BQ25798 charger; // standard i2C address 0x6B

void setup()
{
    Serial.begin(115200);
    Serial.println("\nBQ25798 library test");
    charger.begin();
    charger.scanI2C();
    Serial.println("=========================================================================================");
    delay(10000);
}

void loop()
{
    // Serial.println("HEX output:");
    // charger.printAllRegisters(); // default HEX
    charger.readSingleRegister(0x1C);
    (int16_t)charger.readSingleRegister(0x33);

    charger.setWatchdogTimer(0); //disables Watchdog timer
    
    //charger.setSingleRegister(0x2E, 0b10110000); //enable adc in register 0x2E => default: 0b00110000

    /*Serial.println("\nBINARY output:");
    charger.printAllRegisters(true); // binary*/
    Serial.println("=========================================================================================");
    delay(10000);
}
