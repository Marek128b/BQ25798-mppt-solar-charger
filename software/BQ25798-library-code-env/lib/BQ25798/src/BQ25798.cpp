#include <Arduino.h>
#include "BQ25798.h"

BQ25798::BQ25798(uint8_t i2cAddress)
{
    this->_i2cAddress = i2cAddress;
}

/*i2C port, speed in Hz standard value is 100kHz
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

// code to scan for all devices on the i2C port
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

// reads a single register in the BQ25798
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

uint8_t BQ25798::readRegister8(uint8_t reg)
{
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);

    _wire->requestFrom(_i2cAddress, (uint8_t)1);
    if (_wire->available())
    {
        return _wire->read();
    }
    return 0xFF; // error
}

uint16_t BQ25798::readRegister16(uint8_t reg)
{
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);

    _wire->requestFrom(_i2cAddress, (uint8_t)2);
    uint16_t msb = _wire->read();
    uint16_t lsb = _wire->read();
    return (msb << 8) | lsb; // MSB first
}

void BQ25798::decodeRegister(uint8_t reg, uint16_t value)
{
    switch (reg)
    {
    // -------------------
    // ADC Registers
    // -------------------

    // REG00_Minimal_System_Voltage Register (Offset = 0h) [reset = X]
    case 0x00:
    {                                   // Minimal System Voltage
        uint8_t vsysmin = value & 0x3F; // bits 5-0
        int vmin_mV = (float)vsysmin * 250 + 2500;
        Serial.print(" -> MinSysV = ");
        Serial.print(vmin_mV, 0);
        Serial.print(" mV");

        // Map to cell count for POR defaults
        switch (vmin_mV)
        {
        case 3500:
            Serial.print(" (1s)");
            break;
        case 7000:
            Serial.print(" (2s)");
            break;
        case 9000:
            Serial.print(" (3s)");
            break;
        case 12000:
            Serial.print(" (4s)");
            break;
        default:
            break; // other voltages, no POR mapping
        }
        break;
    }

    // REG01_Charge_Voltage_Limit Register (Offset = 1h) [reset = X]
    case 0x01:
    {                                   // Charge Voltage Limit
        uint16_t vreg = value & 0x07FF; // bits 10-0
        float vchg_mV = (float)vreg * 10;
        Serial.print(" -> ChargeV = ");
        Serial.print(vchg_mV, 0);
        Serial.print(" mV");

        // Map to cell count for POR defaults
        switch ((int)vchg_mV)
        {
        case 4200:
            Serial.print(" (1s)");
            break;
        case 8400:
            Serial.print(" (2s)");
            break;
        case 12600:
            Serial.print(" (3s)");
            break;
        case 16800:
            Serial.print(" (4s)");
            break;
        default:
            break; // other voltages, no mapping
        }
        break;
    }

    // REG03_Charge_Current_Limit Register (Offset = 3h) [reset = X]
    case 0x03:
    {                                   // Charge Current Limit
        uint16_t ichg = value & 0x01FF; // bits 8-0
        float ichg_mA = (float)ichg * 10;
        Serial.print(" -> ChargeI = ");
        Serial.print(ichg_mA, 0);
        Serial.print(" mA");

        // Optional: indicate POR default
        if ((int)ichg_mA == 1000)
            Serial.print(" (POR default 1A)");
        break;
    }

    // REG05_Input_Voltage_Limit Register (Offset = 5h) [reset = 24h]
    case 0x05:
    {                                  // Input Voltage Limit
        uint8_t vindpm = value & 0xFF; // bits 7-0
        float vin_mV = (float)vindpm * 100;
        Serial.print(" -> InputV = ");
        Serial.print(vin_mV, 0);
        Serial.print(" mV");

        // Optional: indicate POR default
        if ((int)vin_mV == 3600)
            Serial.print(" (POR default)");
        break;
    }

    // REG06_Input_Current_Limit Register (Offset = 6h) [reset = 12Ch]
    case 0x06:
    {                                     // Input Current Limit
        uint16_t iindpm = value & 0x01FF; // bits 8-0
        float iin_mA = (float)iindpm * 10;
        Serial.print(" -> InputI = ");
        Serial.print(iin_mA, 0);
        Serial.print(" mA");

        // Optional: indicate POR default
        if ((int)iin_mA == 3000)
            Serial.print(" (POR default)");
        break;
    }

    // REG08_Precharge_Control Register (Offset = 8h) [reset = C3h]
    case 0x08:
    {                                      // Precharge Control
        uint8_t vth = (value >> 6) & 0x03; // bits 7-6
        uint8_t ipre = value & 0x3F;       // bits 5-0

        Serial.print(" -> Precharge Vth = ");
        switch (vth)
        {
        case 0:
            Serial.print("15% of VREG");
            break;
        case 1:
            Serial.print("62.2% of VREG");
            break;
        case 2:
            Serial.print("66.7% of VREG");
            break;
        case 3:
            Serial.print("71.4% of VREG");
            break;
        }

        float ipre_mA = ipre * 40;
        Serial.print(", Iprecharge = ");
        Serial.print(ipre_mA, 0);
        Serial.print(" mA");

        break;
    }

    // REG09_Termination_Control Register (Offset = 9h) [reset = 5h]
    case 0x09:
    {                                       // Termination Control
        bool reg_rst = (value >> 6) & 0x01; // bit 6
        bool stop_wd = (value >> 5) & 0x01; // bit 5
        uint8_t iterm = value & 0x1F;       // bits 4-0

        Serial.print(" -> REG_RST = ");
        Serial.print(reg_rst ? "Reset" : "Not reset");

        Serial.print(", STOP_WD_CHG = ");
        Serial.print(stop_wd ? "EN_CHG=0 on WD expiry" : "Keep EN_CHG");

        float iterm_mA = iterm * 40;
        Serial.print(", Iterm = ");
        Serial.print(iterm_mA, 0);
        Serial.print(" mA");

        break;
    }

    // REG0A_Re-charge_Control Register (Offset = Ah) [reset = X]
    case 0x0A:
    {                                         // Re-charge Control
        uint8_t cell = (value >> 6) & 0x03;   // bits 7-6
        uint8_t trechg = (value >> 4) & 0x03; // bits 5-4
        uint8_t vrechg = value & 0x0F;        // bits 3-0

        Serial.print(" -> CELL = ");
        switch (cell)
        {
        case 0:
            Serial.print("1s");
            break;
        case 1:
            Serial.print("2s");
            break;
        case 2:
            Serial.print("3s");
            break;
        case 3:
            Serial.print("4s");
            break;
        }

        Serial.print(", Trechg = ");
        switch (trechg)
        {
        case 0:
            Serial.print("64 ms");
            break;
        case 1:
            Serial.print("256 ms");
            break;
        case 2:
            Serial.print("1024 ms");
            break;
        case 3:
            Serial.print("2048 ms");
            break;
        }

        float vrechg_mV = 50 + vrechg * 50;
        Serial.print(", Vrechg = ");
        Serial.print(vrechg_mV, 0);
        Serial.print(" mV");

        break;
    }

    // REG0B_VOTG_regulation Register (Offset = Bh) [reset = DCh]
    case 0x0B:
    {                                   // VOTG Regulation
        uint16_t votg = value & 0x07FF; // bits 10-0
        float votg_mV = votg * 10 + 2800;
        Serial.print(" -> VOTG = ");
        Serial.print(votg_mV, 0);
        Serial.print(" mV");

        // Optional: indicate POR default
        if ((int)votg_mV == 5000)
            Serial.print(" (POR default)");
        break;
    }

    // REG0D_IOTG_regulation Register (Offset = Dh) [reset = 4Bh]
    case 0x0D:
    {                                          // IOTG Regulation
        bool prechg_tmr = (value >> 7) & 0x01; // bit 7
        uint8_t iotg = value & 0x7F;           // bits 6-0

        Serial.print(" -> PRECHG_TMR = ");
        Serial.print(prechg_tmr ? "0.5 hrs" : "2 hrs");

        float iotg_mA = iotg * 40;
        Serial.print(", IOTG = ");
        Serial.print(iotg_mA, 0);
        Serial.print(" mA");

        // Optional: indicate POR default
        if ((int)iotg_mA == 3040)
            Serial.print(" (POR default)");
        break;
    }

        // REG0E_Timer_Control Register (Offset = Eh) [reset = 3Dh]
    case 0x0E:
    {                                            // Timer Control
        uint8_t topoff = (value >> 6) & 0x03;    // bits 7-6
        bool en_trichg = (value >> 5) & 0x01;    // bit 5
        bool en_prechg = (value >> 4) & 0x01;    // bit 4
        bool en_chg = (value >> 3) & 0x01;       // bit 3
        uint8_t chg_timer = (value >> 1) & 0x03; // bits 2-1
        bool tmr2x = value & 0x01;               // bit 0

        Serial.print(" -> TOPOFF_TMR = ");
        switch (topoff)
        {
        case 0:
            Serial.print("Disabled");
            break;
        case 1:
            Serial.print("15 min");
            break;
        case 2:
            Serial.print("30 min");
            break;
        case 3:
            Serial.print("45 min");
            break;
        }

        Serial.print(", EN_TRICHG_TMR = ");
        Serial.print(en_trichg ? "ENABLED" : "DISABLED");
        Serial.print(", EN_PRECHG_TMR = ");
        Serial.print(en_prechg ? "ENABLED" : "DISABLED");
        Serial.print(", EN_CHG_TMR = ");
        Serial.print(en_chg ? "ENABLED" : "DISABLED");

        Serial.print(", CHG_TMR = ");
        switch (chg_timer)
        {
        case 0:
            Serial.print("5 h");
            break;
        case 1:
            Serial.print("8 h");
            break;
        case 2:
            Serial.print("12 h");
            break;
        case 3:
            Serial.print("24 h");
            break;
        }

        Serial.print(", TMR2X_EN = ");
        Serial.print(tmr2x ? "SLOWED 2x" : "NORMAL");

        break;
    }

    // REG0F_Charger_Control_0 Register (Offset = Fh) [reset = A2h]
    case 0x0F:
    { // Charger Control 0
        bool en_auto_dis = (value >> 7) & 0x01;
        bool force_dis = (value >> 6) & 0x01;
        bool en_chg = (value >> 5) & 0x01;
        bool en_ico = (value >> 4) & 0x01;
        bool force_ico = (value >> 3) & 0x01;
        bool en_hiz = (value >> 2) & 0x01;
        bool en_term = (value >> 1) & 0x01;
        bool en_backup = value & 0x01; // bit 0

        Serial.print(" -> EN_AUTO_IBATDIS = ");
        Serial.print(en_auto_dis ? "ON" : "OFF");
        Serial.print(", FORCE_IBATDIS = ");
        Serial.print(force_dis ? "ON" : "OFF");
        Serial.print(", EN_CHG = ");
        Serial.print(en_chg ? "ENABLED" : "DISABLED");
        Serial.print(", EN_ICO = ");
        Serial.print(en_ico ? "ENABLED" : "DISABLED");
        Serial.print(", FORCE_ICO = ");
        Serial.print(force_ico ? "FORCE" : "NO");
        Serial.print(", EN_HIZ = ");
        Serial.print(en_hiz ? "ENABLED" : "DISABLED");
        Serial.print(", EN_TERM = ");
        Serial.print(en_term ? "ENABLED" : "DISABLED");
        Serial.print(", EN_BACKUP = ");
        Serial.print(en_backup ? "ENABLED" : "DISABLED");

        break;
    }

    // REG10_Charger_Control_1 Register (Offset = 10h) [reset = 85h]
    case 0x10:
    {                                              // Charger Control 1
        uint8_t vbus_backup = (value >> 6) & 0x03; // bits 7-6
        uint8_t vac_ovp = (value >> 4) & 0x03;     // bits 5-4
        bool wd_rst = (value >> 3) & 0x01;         // bit 3
        uint8_t watchdog = value & 0x07;           // bits 2-0

        Serial.print(" -> VBUS_BACKUP = ");
        switch (vbus_backup)
        {
        case 0:
            Serial.print("40% VINDPM");
            break;
        case 1:
            Serial.print("60% VINDPM");
            break;
        case 2:
            Serial.print("80% VINDPM");
            break;
        case 3:
            Serial.print("100% VINDPM");
            break;
        }

        Serial.print(", VAC_OVP = ");
        switch (vac_ovp)
        {
        case 0:
            Serial.print("26 V");
            break;
        case 1:
            Serial.print("22 V");
            break;
        case 2:
            Serial.print("12 V");
            break;
        case 3:
            Serial.print("7 V");
            break;
        }

        Serial.print(", WD_RST = ");
        Serial.print(wd_rst ? "RESET" : "NORMAL");

        Serial.print(", WATCHDOG = ");
        switch (watchdog)
        {
        case 0:
            Serial.print("Disabled");
            break;
        case 1:
            Serial.print("0.5 s");
            break;
        case 2:
            Serial.print("1 s");
            break;
        case 3:
            Serial.print("2 s");
            break;
        case 4:
            Serial.print("20 s");
            break;
        case 5:
            Serial.print("40 s");
            break;
        case 6:
            Serial.print("80 s");
            break;
        case 7:
            Serial.print("160 s");
            break;
        }

        break;
    }

    // REG11_Charger_Control_2 Register (Offset = 11h) [reset = 40h]
    case 0x11:
    { // Charger Control 2
        bool force_indet = (value >> 7) & 0x01;
        bool auto_indet = (value >> 6) & 0x01;
        bool en_12v = (value >> 5) & 0x01;
        bool en_9v = (value >> 4) & 0x01;
        bool hvdcp_en = (value >> 3) & 0x01;
        uint8_t sdrv_ctrl = (value >> 1) & 0x03;
        bool sdrv_dly = value & 0x01;

        Serial.print(" -> FORCE_INDET = ");
        Serial.print(force_indet ? "FORCE" : "NO");
        Serial.print(", AUTO_INDET_EN = ");
        Serial.print(auto_indet ? "ENABLED" : "DISABLED");
        Serial.print(", EN_12V = ");
        Serial.print(en_12v ? "ENABLED" : "DISABLED");
        Serial.print(", EN_9V = ");
        Serial.print(en_9v ? "ENABLED" : "DISABLED");
        Serial.print(", HVDCP_EN = ");
        Serial.print(hvdcp_en ? "ENABLED" : "DISABLED");

        Serial.print(", SDRV_CTRL = ");
        switch (sdrv_ctrl)
        {
        case 0:
            Serial.print("IDLE");
            break;
        case 1:
            Serial.print("Shutdown");
            break;
        case 2:
            Serial.print("Ship");
            break;
        case 3:
            Serial.print("System Power Reset");
            break;
        }

        Serial.print(", SDRV_DLY = ");
        Serial.print(sdrv_dly ? "NO delay" : "10s delay");

        break;
    }

    // REG12_Charger_Control_3 Register (Offset = 12h) [reset = 0h]
    case 0x12:
    { // Charger Control 3
        bool dis_acdrv = (value >> 7) & 0x01;
        bool en_otg = (value >> 6) & 0x01;
        bool pfm_otg_dis = (value >> 5) & 0x01;
        bool pfm_fwd_dis = (value >> 4) & 0x01;
        bool wkup_dly = (value >> 3) & 0x01;
        bool dis_ldo = (value >> 2) & 0x01;
        bool dis_otg_ooa = (value >> 1) & 0x01;
        bool dis_fwd_ooa = value & 0x01;

        Serial.print(" -> DIS_ACDRV = ");
        Serial.print(dis_acdrv ? "FORCE OFF" : "Normal");
        Serial.print(", EN_OTG = ");
        Serial.print(en_otg ? "ENABLED" : "DISABLED");
        Serial.print(", PFM_OTG_DIS = ");
        Serial.print(pfm_otg_dis ? "DISABLED" : "ENABLED");
        Serial.print(", PFM_FWD_DIS = ");
        Serial.print(pfm_fwd_dis ? "DISABLED" : "ENABLED");
        Serial.print(", WKUP_DLY = ");
        Serial.print(wkup_dly ? "15 ms" : "1 s");
        Serial.print(", DIS_LDO = ");
        Serial.print(dis_ldo ? "DISABLED" : "ENABLED");
        Serial.print(", DIS_OTG_OOA = ");
        Serial.print(dis_otg_ooa ? "DISABLED" : "ENABLED");
        Serial.print(", DIS_FWD_OOA = ");
        Serial.print(dis_fwd_ooa ? "DISABLED" : "ENABLED");

        break;
    }

    // REG13_Charger_Control_4 Register (Offset = 13h) [reset = X]
    case 0x13:
    { // Charger Control 4
        bool en_acdrv2 = (value >> 7) & 0x01;
        bool en_acdrv1 = (value >> 6) & 0x01;
        bool pwm_freq = (value >> 5) & 0x01;
        bool dis_stat = (value >> 4) & 0x01;
        bool dis_vsys_short = (value >> 3) & 0x01;
        bool dis_votg_uvp = (value >> 2) & 0x01;
        bool force_vindpm = (value >> 1) & 0x01;
        bool en_ibus_ocp = value & 0x01;

        Serial.print(" -> EN_ACDRV2 = ");
        Serial.print(en_acdrv2 ? "ON" : "OFF");
        Serial.print(", EN_ACDRV1 = ");
        Serial.print(en_acdrv1 ? "ON" : "OFF");
        Serial.print(", PWM_FREQ = ");
        Serial.print(pwm_freq ? "750 kHz" : "1.5 MHz");
        Serial.print(", DIS_STAT = ");
        Serial.print(dis_stat ? "DISABLED" : "ENABLED");
        Serial.print(", DIS_VSYS_SHORT = ");
        Serial.print(dis_vsys_short ? "DISABLED" : "ENABLED");
        Serial.print(", DIS_VOTG_UVP = ");
        Serial.print(dis_votg_uvp ? "DISABLED" : "ENABLED");
        Serial.print(", FORCE_VINDPM_DET = ");
        Serial.print(force_vindpm ? "FORCED" : "NORMAL");
        Serial.print(", EN_IBUS_OCP = ");
        Serial.print(en_ibus_ocp ? "ENABLED" : "DISABLED");

        break;
    }

    // REG14_Charger_Control_5 Register (Offset = 14h) [reset = 16h]
    case 0x14:
    { // Charger Control 5
        bool sfet_present = (value >> 7) & 0x01;
        bool en_ibat = (value >> 5) & 0x01;
        uint8_t ibat_reg = (value >> 3) & 0x03;
        bool en_iindpm = (value >> 2) & 0x01;
        bool en_extilim = (value >> 1) & 0x01;
        bool en_batoc = value & 0x01;

        Serial.print(" -> SFET_PRESENT = ");
        Serial.print(sfet_present ? "YES" : "NO");
        Serial.print(", EN_IBAT = ");
        Serial.print(en_ibat ? "ENABLED" : "DISABLED");
        Serial.print(", IBAT_REG = ");
        switch (ibat_reg)
        {
        case 0:
            Serial.print("3A");
            break;
        case 1:
            Serial.print("4A");
            break;
        case 2:
            Serial.print("5A");
            break;
        case 3:
            Serial.print("DISABLED");
            break;
        }
        Serial.print(", EN_IINDPM = ");
        Serial.print(en_iindpm ? "ENABLED" : "DISABLED");
        Serial.print(", EN_EXTILIM = ");
        Serial.print(en_extilim ? "ENABLED" : "DISABLED");
        Serial.print(", EN_BATOC = ");
        Serial.print(en_batoc ? "ENABLED" : "DISABLED");

        break;
    }

    // REG15_MPPT_Control Register (Offset = 15h) [reset = AAh]
    case 0x15:
    { // MPPT Control
        uint8_t voc_pct = (value >> 5) & 0x07;
        uint8_t voc_dly = (value >> 3) & 0x03;
        uint8_t voc_rate = (value >> 1) & 0x03;
        bool en_mppt = value & 0x01;

        Serial.print(" -> VOC_PCT = ");
        switch (voc_pct)
        {
        case 0:
            Serial.print("0.5625");
            break;
        case 1:
            Serial.print("0.625");
            break;
        case 2:
            Serial.print("0.6875");
            break;
        case 3:
            Serial.print("0.75");
            break;
        case 4:
            Serial.print("0.8125");
            break;
        case 5:
            Serial.print("0.875");
            break;
        case 6:
            Serial.print("0.9375");
            break;
        case 7:
            Serial.print("1");
            break;
        }

        Serial.print(", VOC_DLY = ");
        switch (voc_dly)
        {
        case 0:
            Serial.print("50ms");
            break;
        case 1:
            Serial.print("300ms");
            break;
        case 2:
            Serial.print("2s");
            break;
        case 3:
            Serial.print("5s");
            break;
        }

        Serial.print(", VOC_RATE = ");
        switch (voc_rate)
        {
        case 0:
            Serial.print("30s");
            break;
        case 1:
            Serial.print("2mins");
            break;
        case 2:
            Serial.print("10mins");
            break;
        case 3:
            Serial.print("30mins");
            break;
        }

        Serial.print(", EN_MPPT = ");
        Serial.print(en_mppt ? "ENABLED" : "DISABLED");
        break;
    }

    // REG16_Temperature_Control Register (Offset = 16h) [reset = C0h]
    case 0x16:
    { // Temperature Control
        uint8_t treg = (value >> 6) & 0x03;
        uint8_t tshut = (value >> 4) & 0x03;
        bool vbus_pd = (value >> 3) & 0x01;
        bool vac1_pd = (value >> 2) & 0x01;
        bool vac2_pd = (value >> 1) & 0x01;
        bool bkup_acfet1 = value & 0x01;

        Serial.print(" -> TREG = ");
        switch (treg)
        {
        case 0:
            Serial.print("60C");
            break;
        case 1:
            Serial.print("80C");
            break;
        case 2:
            Serial.print("100C");
            break;
        case 3:
            Serial.print("120C");
            break;
        }

        Serial.print(", TSHUT = ");
        switch (tshut)
        {
        case 0:
            Serial.print("150C");
            break;
        case 1:
            Serial.print("130C");
            break;
        case 2:
            Serial.print("120C");
            break;
        case 3:
            Serial.print("85C");
            break;
        }

        Serial.print(", VBUS_PD_EN = ");
        Serial.print(vbus_pd ? "Enable" : "Disable");
        Serial.print(", VAC1_PD_EN = ");
        Serial.print(vac1_pd ? "Enable" : "Disable");
        Serial.print(", VAC2_PD_EN = ");
        Serial.print(vac2_pd ? "Enable" : "Disable");
        Serial.print(", BKUP_ACFET1_ON = ");
        Serial.print(bkup_acfet1 ? "Enable" : "IDLE");

        break;
    }

    // REG17_NTC_Control_0 Register (Offset = 17h) [reset = 7Ah]
    case 0x17:
    { // NTC Control 0
        uint8_t jeita_vset = (value >> 5) & 0x07;
        uint8_t jeita_iseth = (value >> 3) & 0x03;
        uint8_t jeita_isetc = (value >> 1) & 0x03;

        Serial.print(" -> JEITA_VSET = ");
        switch (jeita_vset)
        {
        case 0:
            Serial.print("Charge Suspend");
            break;
        case 1:
            Serial.print("VREG-800mV");
            break;
        case 2:
            Serial.print("VREG-600mV");
            break;
        case 3:
            Serial.print("VREG-400mV");
            break;
        case 4:
            Serial.print("VREG-300mV");
            break;
        case 5:
            Serial.print("VREG-200mV");
            break;
        case 6:
            Serial.print("VREG-100mV");
            break;
        case 7:
            Serial.print("VREG unchanged");
            break;
        }

        Serial.print(", JEITA_ISETH = ");
        switch (jeita_iseth)
        {
        case 0:
            Serial.print("Charge Suspend");
            break;
        case 1:
            Serial.print("20% ICHG");
            break;
        case 2:
            Serial.print("40% ICHG");
            break;
        case 3:
            Serial.print("unchanged");
            break;
        }

        Serial.print(", JEITA_ISETC = ");
        switch (jeita_isetc)
        {
        case 0:
            Serial.print("Charge Suspend");
            break;
        case 1:
            Serial.print("20% ICHG");
            break;
        case 2:
            Serial.print("40% ICHG");
            break;
        case 3:
            Serial.print("unchanged");
            break;
        }

        break;
    }

    // REG18_NTC_Control_1 Register (Offset = 18h) [reset = 54h]
    case 0x18:
    { // NTC Control 1
        uint8_t ts_cool = (value >> 6) & 0x03;
        uint8_t ts_warm = (value >> 4) & 0x03;
        uint8_t bhot = (value >> 2) & 0x03;
        bool bcold = (value >> 1) & 0x01;
        bool ts_ignore = value & 0x01;

        Serial.print(" -> TS_COOL = ");
        switch (ts_cool)
        {
        case 0:
            Serial.print("5C");
            break;
        case 1:
            Serial.print("10C");
            break;
        case 2:
            Serial.print("15C");
            break;
        case 3:
            Serial.print("20C");
            break;
        }

        Serial.print(", TS_WARM = ");
        switch (ts_warm)
        {
        case 0:
            Serial.print("40C");
            break;
        case 1:
            Serial.print("45C");
            break;
        case 2:
            Serial.print("50C");
            break;
        case 3:
            Serial.print("55C");
            break;
        }

        Serial.print(", BHOT = ");
        switch (bhot)
        {
        case 0:
            Serial.print("55C");
            break;
        case 1:
            Serial.print("60C");
            break;
        case 2:
            Serial.print("65C");
            break;
        case 3:
            Serial.print("Disabled");
            break;
        }

        Serial.print(", BCOLD = ");
        Serial.print(bcold ? "-20C" : "-10C");

        Serial.print(", TS_IGNORE = ");
        Serial.print(ts_ignore ? "Ignore" : "Normal");

        break;
    }

    // REG19_ICO_Current_Limit Register (Offset = 19h) [reset = 0h]
    case 0x19:
    {                                        // ICO Current Limit
        uint16_t ico_ilim = value & 0x1FF;   // bits 8-0
        uint32_t current_mA = ico_ilim * 10; // step size 10mA
        Serial.print(" -> ICO Current Limit = ");
        Serial.print(current_mA);
        Serial.print(" mA");
        break;
    }

    // REG1B_Charger_Status_0 Register (Offset = 1Bh) [reset = 0h]
    case 0x1B:
    {
        Serial.print(" -> IINDPM_STAT = ");
        Serial.print((value >> 7) & 0x01);
        Serial.print(", VINDPM_STAT = ");
        Serial.print((value >> 6) & 0x01);
        Serial.print(", WD_STAT = ");
        Serial.print((value >> 5) & 0x01);
        Serial.print(", PG_STAT = ");
        Serial.print((value >> 3) & 0x01);
        Serial.print(", AC2_PRESENT_STAT = ");
        Serial.print((value >> 2) & 0x01);
        Serial.print(", AC1_PRESENT_STAT = ");
        Serial.print((value >> 1) & 0x01);
        Serial.print(", VBUS_PRESENT_STAT = ");
        Serial.print(value & 0x01);
        break;
    }

    // REG1C_Charger_Status_1 Register (Offset = 1Ch) [reset = 0h]
    case 0x1C:
    {
        uint8_t chg_stat = (value >> 5) & 0x07;  // bits 7-5
        uint8_t vbus_stat = (value >> 1) & 0x0F; // bits 4-1
        bool bc12_done = value & 0x01;           // bit 0

        Serial.println();
        Serial.print(" -> CHG_STAT = ");
        switch (chg_stat)
        {
        case 0:
            Serial.println("0: Not Charging");
            break;
        case 1:
            Serial.println("1: Trickle Charge");
            break;
        case 2:
            Serial.println("2: Pre-charge");
            break;
        case 3:
            Serial.println("3: Fast Charge (CC mode)");
            break;
        case 4:
            Serial.println("4: Taper Charge (CV mode)");
            break;
        case 5:
            Serial.println("5: Reserved");
            break;
        case 6:
            Serial.println("6: Top-off Timer Active Charging");
            break;
        case 7:
            Serial.println("7: Charge Termination Done");
            break;
        }

        Serial.print(" -> VBUS_STAT = ");
        switch (vbus_stat)
        {
        case 0x0:
            Serial.println("0x0: No Input or BHOT/BCOLD OTG");
            break;
        case 0x1:
            Serial.println("0x1: USB SDP (500mA)");
            break;
        case 0x2:
            Serial.println("0x2: USB CDP (1.5A)");
            break;
        case 0x3:
            Serial.println("0x3: USB DCP (3.25A)");
            break;
        case 0x4:
            Serial.println("0x4: Adjustable HV DCP (1.5A)");
            break;
        case 0x5:
            Serial.println("0x5: Unknown Adapter (3A)");
            break;
        case 0x6:
            Serial.println("0x6: Non-Standard Adapter (1-2.4A)");
            break;
        case 0x7:
            Serial.println("0x7: In OTG mode");
            break;
        case 0x8:
            Serial.println("0x8: Not qualified adapter");
            break;
        case 0x9:
            Serial.println("0x9: Reserved");
            break;
        case 0xA:
            Serial.println("0xA: Reserved");
            break;
        case 0xB:
            Serial.println("0xB: Device directly powered from VBUS");
            break;
        case 0xC:
            Serial.println("0xC: Backup Mode");
            break;
        default:
            Serial.println("Other/Reserved");
            break;
        }

        Serial.print(" -> BC1.2_DONE_STAT = ");
        Serial.println(bc12_done ? "1: Complete" : "0: Not complete");
        break;
    }

    // REG1D_Charger_Status_2 Register (Offset = 1Dh) [reset = 0h]
    case 0x1D:
    {
        uint8_t ico_stat = (value >> 6) & 0x03;
        bool treg_stat = (value >> 2) & 0x01;
        bool dpdm_stat = (value >> 1) & 0x01;
        bool vbat_present = value & 0x01;

        Serial.print(" -> ICO_STAT = ");
        Serial.print(ico_stat);
        Serial.print(", TREG_STAT = ");
        Serial.print(treg_stat);
        Serial.print(", DPDM_STAT = ");
        Serial.print(dpdm_stat);
        Serial.print(", VBAT_PRESENT = ");
        Serial.print(vbat_present);
        break;
    }

    // REG1E_Charger_Status_3 Register (Offset = 1Eh) [reset = 0h]
    case 0x1E:
    {
        bool acrb2 = (value >> 7) & 0x01;
        bool acrb1 = (value >> 6) & 0x01;
        bool adc_done = (value >> 5) & 0x01;
        bool vsys_stat = (value >> 4) & 0x01;
        bool chg_timer = (value >> 3) & 0x01;
        bool trichg_timer = (value >> 2) & 0x01;
        bool prechg_timer = (value >> 1) & 0x01;

        Serial.print(" -> ACRB2_STAT = ");
        Serial.print(acrb2);
        Serial.print(", ACRB1_STAT = ");
        Serial.print(acrb1);
        Serial.print(", ADC_DONE_STAT = ");
        Serial.print(adc_done);
        Serial.print(", VSYS_STAT = ");
        Serial.print(vsys_stat);
        Serial.print(", CHG_TMR_STAT = ");
        Serial.print(chg_timer);
        Serial.print(", TRICHG_TMR_STAT = ");
        Serial.print(trichg_timer);
        Serial.print(", PRECHG_TMR_STAT = ");
        Serial.print(prechg_timer);
        break;
    }

    // REG1F_Charger_Status_4 Register (Offset = 1Fh) [reset = 0h]
    case 0x1F:
    {
        bool vbat_otg_low = (value >> 4) & 0x01;
        bool ts_cold = (value >> 3) & 0x01;
        bool ts_cool = (value >> 2) & 0x01;
        bool ts_warm = (value >> 1) & 0x01;
        bool ts_hot = value & 0x01;

        Serial.print(" -> VBATOTG_LOW = ");
        Serial.print(vbat_otg_low);
        Serial.print(", TS_COLD = ");
        Serial.print(ts_cold);
        Serial.print(", TS_COOL = ");
        Serial.print(ts_cool);
        Serial.print(", TS_WARM = ");
        Serial.print(ts_warm);
        Serial.print(", TS_HOT = ");
        Serial.print(ts_hot);
        break;
    }

    // REG48_Part_Information Register (Offset = 48h) [reset = 0h]
    case 0x48:
    {                                     // Part Information
        uint8_t pn = (value >> 3) & 0x07; // bits 5-3
        uint8_t rev = value & 0x07;       // bits 2-0
        Serial.print(" -> Part = ");
        switch (pn)
        {
        case 0b011:
            Serial.print("BQ25798");
            break;
        default:
            Serial.print("Unknown");
            break;
        }
        Serial.print(", Rev = ");
        Serial.print(rev);
        break;
    }

    default:
        break; // no decoding available
    }
}

// reads all registers of the BQ25798 and prints them to Serial
void BQ25798::readAllRegisters(bool outputBinary)
{
    Serial.println(F("Reading BQ25798 Registers:"));

    for (uint8_t i = 0; i < registerCount; i++)
    {
        uint8_t reg = registers[i].address;
        const char *name = registers[i].name;
        bool wide = registers[i].is16bit;

        // read value
        uint16_t value = wide ? readRegister16(reg) : readRegister8(reg);

        // print address
        Serial.print(F("0x"));
        if (reg < 0x10)
            Serial.print('0');
        Serial.print(reg, HEX);
        Serial.print(" : ");
        Serial.print(name);
        Serial.print(" : ");

        // format value
        if (outputBinary)
        {
            Serial.print(F("0b"));
            if (wide)
            {
                for (int b = 15; b >= 0; b--)
                    Serial.print((value >> b) & 0x01);
            }
            else
            {
                for (int b = 7; b >= 0; b--)
                    Serial.print((value >> b) & 0x01);
            }
        }
        else
        {
            Serial.print(F("0x"));
            if (wide)
            {
                if (value < 0x1000)
                    Serial.print('0');
                if (value < 0x0100)
                    Serial.print('0');
                if (value < 0x0010)
                    Serial.print('0');
                Serial.print(value, HEX);
            }
            else
            {
                if (value < 0x10)
                    Serial.print('0');
                Serial.print(value & 0xFF, HEX);
            }
        }

        // decode if possible
        decodeRegister(reg, value);

        Serial.println();
        Serial.println();
    }
}

const BQ25798::RegisterEntry BQ25798::registers[] = {
    {0x00, "REG00_Minimal_System_Voltage", false},
    {0x01, "REG01_Charge_Voltage_Limit", true},
    {0x03, "REG03_Charge_Current_Limit", true},
    {0x05, "REG05_Input_Voltage_Limit", false},
    {0x06, "REG06_Input_Current_Limit", true},
    {0x08, "REG08_Precharge_Control", false},
    {0x09, "REG09_Termination_Control", false},
    {0x0A, "REG0A_Re-charge_Control", false},
    {0x0B, "REG0B_VOTG_regulation", true},
    {0x0D, "REG0D_IOTG_regulation", false},
    {0x0E, "REG0E_Timer_Control", false},
    {0x0F, "REG0F_Charger_Control_0", false},
    {0x10, "REG10_Charger_Control_1", false},
    {0x11, "REG11_Charger_Control_2", false},
    {0x12, "REG12_Charger_Control_3", false},
    {0x13, "REG13_Charger_Control_4", false},
    {0x14, "REG14_Charger_Control_5", false},
    {0x15, "REG15_MPPT_Control", false},
    {0x16, "REG16_Temperature_Control", false},
    {0x17, "REG17_NTC_Control_0", false},
    {0x18, "REG18_NTC_Control_1", false},
    {0x19, "REG19_ICO_Current_Limit", true},
    {0x1B, "REG1B_Charger_Status_0", false},
    {0x1C, "REG1C_Charger_Status_1", false},
    {0x1D, "REG1D_Charger_Status_2", false},
    {0x1E, "REG1E_Charger_Status_3", false},
    {0x1F, "REG1F_Charger_Status_4", false},
    {0x20, "REG20_FAULT_Status_0", false},
    {0x21, "REG21_FAULT_Status_1", false},
    {0x22, "REG22_Charger_Flag_0", false},
    {0x23, "REG23_Charger_Flag_1", false},
    {0x24, "REG24_Charger_Flag_2", false},
    {0x25, "REG25_Charger_Flag_3", false},
    {0x26, "REG26_FAULT_Flag_0", false},
    {0x27, "REG27_FAULT_Flag_1", false},
    {0x28, "REG28_Charger_Mask_0", false},
    {0x29, "REG29_Charger_Mask_1", false},
    {0x2A, "REG2A_Charger_Mask_2", false},
    {0x2B, "REG2B_Charger_Mask_3", false},
    {0x2C, "REG2C_FAULT_Mask_0", false},
    {0x2D, "REG2D_FAULT_Mask_1", false},
    {0x2E, "REG2E_ADC_Control", false},
    {0x2F, "REG2F_ADC_Function_Disable_0", false},
    {0x30, "REG30_ADC_Function_Disable_1", false},
    {0x31, "REG31_IBUS_ADC", true},
    {0x33, "REG33_IBAT_ADC", true},
    {0x35, "REG35_VBUS_ADC", true},
    {0x37, "REG37_VAC1_ADC", true},
    {0x39, "REG39_VAC2_ADC", true},
    {0x3B, "REG3B_VBAT_ADC", true},
    {0x3D, "REG3D_VSYS_ADC", true},
    {0x3F, "REG3F_TS_ADC", true},
    {0x41, "REG41_TDIE_ADC", true},
    {0x43, "REG43_D+_ADC", true},
    {0x45, "REG45_D-_ADC", true},
    {0x47, "REG47_DPDM_Driver", false},
    {0x48, "REG48_Part_Information", false}};

const uint8_t BQ25798::registerCount = sizeof(BQ25798::registers) / sizeof(BQ25798::registers[0]);
