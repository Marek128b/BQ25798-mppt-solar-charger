#include <Arduino.h>
#include "../lib\BQ25798\src\BQ25798.h"

BQ25798 charger; //standard i2C address 0x6B

void setup()
{
    Serial.begin(115200);
    Serial.println("BQ25798 library test");
    charger.begin();
    charger.scanI2C();
}

void loop()
{
    charger.readAllRegisters();
    delay(1000);
}
