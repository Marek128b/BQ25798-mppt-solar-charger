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
    Serial.println("HEX output:");
    charger.readAllRegisters(); // default HEX

    /*Serial.println("\nBINARY output:");
    charger.readAllRegisters(true); // binary*/
    Serial.println("=========================================================================================");
    delay(10000);
}
