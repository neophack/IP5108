#include "IP5108.h"

#define IP5108_DEFATLT_ADDRESS 0x75

IP5108::IP5108(TwoWire *i, int sdaPin, int sclPin, uint32_t frequency)
{
    i2c = i;
    i2c->begin(sdaPin, sclPin, frequency);
    Address = IP5108_DEFATLT_ADDRESS;
}

IP5108::~IP5108()
{
}

void IP5108::setup()
{
    writeRegBit(SYS_CTL0, SYS_CTL0_BIT_FlashLight, false);
    writeRegBit(SYS_CTL0, SYS_CTL0_BIT_Light, false);
    writeRegBit(SYS_CTL1, SYS_CTL1_BIT_LowLoadOff, false);
    writeRegBit(SYS_CTL1, SYS_CTL1_BIT_LoadInsertAutoStartUp, false);

    writeRegBit(SYS_CTL3, SYS_CTL3_BIT_DoubleShortPress, true);
    writeRegBits(SYS_CTL3, SYS_CTL3_BIT_LongPressTimeSet, SYS_CTL3_BIT_LongPressTimeSet_4S);

    writeRegBit(SYS_CTL4, SYS_CTL4_BIT_VIN_PullOutBoost, true); // VIN ?��??????? BOOST
    writeRegBit(SYS_CTL5, SYS_CTL5_BIT_KeyShutdownSet, false);

    writeRegBits(MFP_CTL0, MFP_CTL0_BIT_LIGHT_Sel, MFP_CTL0_BIT_LIGHT_Sel_WLED);
    writeRegBits(MFP_CTL0, MFP_CTL0_BIT_L3_Sel, MFP_CTL0_BIT_L3_Sel_L3);
    writeRegBits(MFP_CTL0, MFP_CTL0_BIT_L4_Sel, MFP_CTL0_BIT_L4_Sel_L4);
    writeRegBit(MFP_CTL1, MFP_CTL1_BIT_VSET_Sel, false);
    writeRegBit(MFP_CTL1, MFP_CTL1_BIT_RSET_Sel, false);

    //  ???????????? ??????????
    writeRegBit(CHG_DIG_CTL4, CHG_DIG_CTL4_BIT_BatteryTypeInternalSet, false);
    //  ???????????4.2V
    writeRegBits(Charger_CTL2, Charger_CTL2_BIT_BatteryTypeSet, Charger_CTL2_BIT_BatteryTypeSet_4V2);
    // 4.2V ???????? 28mV
    writeRegBits(Charger_CTL2, Charger_CTL2_BIT_ConstantVoltageForcingSet, Charger_CTL2_BIT_ConstantVoltageForcingSet_28MV);
}

uint8_t IP5108::readReg(REG_t reg)
{
    i2c->beginTransmission(Address);
    i2c->write(reg);
    i2c->endTransmission();

    i2c->requestFrom(Address, uint8_t(1));
    uint8_t value = i2c->read();
    i2c->endTransmission();

    return value;
}

bool IP5108::readRegBit(REG_t reg, REG_BIT_t bit)
{
    return (readReg(reg) & bit);
}

bool IP5108::readBtn()
{
    return (readReg(Reg_READ2) & Reg_READ2_BIT_KeyPressFlag);
}
bool IP5108::isLongPress()
{
    return (readReg(Reg_READ2) & Reg_READ2_BIT_KeyLongPressFlag);
}
bool IP5108::isClickPress()
{
    return (readReg(Reg_READ2) & Reg_READ2_BIT_KeyClickPressFlag);
}
void IP5108::writeReg(REG_t reg, uint8_t val)
{
    if (readReg(reg) == val)
        return;

    // else
    //     Serial.printf("IP5108 write %d\r\n", val);

    i2c->beginTransmission(Address);
    i2c->write(reg);
    i2c->write(val);
    i2c->endTransmission();
}

void IP5108::writeRegBit(REG_t reg, REG_BIT_t bit, bool val)
{
    uint8_t nval = readReg(reg);
    val ? nval |= bit : nval &= ~bit;
    writeReg(reg, nval);
}

void IP5108::writeRegBits(REG_t reg, REG_BIT_t bit, REG_BIT_t val)
{
    uint8_t nval = readReg(reg);

    for (int i = 0; i < 8; i++)
    {
        if (bit & (1 << i))
            (val & (1 << i)) ? nval |= (1 << i) : nval &= ~(1 << i);
    }

    writeReg(reg, nval);
}

float IP5108::getBattVoltage()
{
    float batVol;
    uint8_t BATVADC_VALUE_low = readReg(BATVADC_DAT0);  // low 8bit
    uint8_t BATVADC_VALUE_high = readReg(BATVADC_DAT1); // high 6bit
    if ((BATVADC_VALUE_high & 0x20) == 0x20)            //????
    {
        batVol = 2600 - ((~BATVADC_VALUE_low) + (~(BATVADC_VALUE_high & 0x1F)) * 256 + 1) * 0.26855;
    }
    else //???
    {
        batVol = 2600 + (BATVADC_VALUE_low + BATVADC_VALUE_high * 256) * 0.26855; // mv ???��
    }
    return batVol;
}

float IP5108::getBattCurrent()
{
    float batCur;
    uint8_t BATIADC_VALUE_low = readReg(BATIADC_DAT0);
    uint8_t BATIADC_VALUE_high = readReg(BATIADC_DAT1);
    if ((BATIADC_VALUE_high & 0x20) == 0x20) //???
    {
        char a = ~BATIADC_VALUE_low;
        char b = (~(BATIADC_VALUE_high & 0x1F) & 0x1f);
        int c = b * 256 + a + 1;
        batCur = -c * 0.745985;
        // BATCUR=-(int)(((~BATIADC_VALUE_low)+(~(BATIADC_VALUE_high & 0x1F))*256+1)*0.745985);
    }
    else //???
    {
        batCur = (BATIADC_VALUE_high * 256 + BATIADC_VALUE_low) * 0.745985; // mA ???��
    }
    return batCur;
}

// open-circuit voltage
float IP5108::getBattOcVoltage()
{
    float outVol;
    uint8_t BATOCVADC_VALUE_low = readReg(BATOCV_DAT0);  // low 8bit
    uint8_t BATOCVADC_VALUE_high = readReg(BATOCV_DAT1); // high 6bit
    if ((BATOCVADC_VALUE_high & 0x20) == 0x20)           //????
    {
        outVol = 2600 - ((~BATOCVADC_VALUE_low) + (~(BATOCVADC_VALUE_high & 0x1F)) * 256 + 1) * 0.26855;
    }
    else //???
    {
        outVol = 2600 + (BATOCVADC_VALUE_low + BATOCVADC_VALUE_high * 256) * 0.26855; // mv ???��
    }
    return outVol;
}

/*
 * @brief: get battery state
 */
void IP5108::getBattState()
{
    // 000: idle?????? 0
    // 001: ???????? 32
    // 010: ????????? 64
    // 011: ???????? 96
    // 100: ?????????? 128
    // 101: ???? 160
    // 110: ??�^??? 192

    uint8_t Reg_Byte = readReg(Reg_READ0) & Reg_READ0_BIT_ChargeStatusFlags;
    uint8_t last_ischarging = isCharging;

    switch (Reg_Byte)
    {
    case 32:
    case 64:
    case 96:
    case 128:
    case 160:
    case 192:
        isCharging = true;
        if (State != Reg_Byte)
        {
            State = Reg_Byte;
            if (!last_ischarging)
                // state code different from last and turn to charge now
                toggle = true;
        }
        break;
    default:
        isCharging = false;
        State = Reg_Byte;
        break;
    }
}

void IP5108::update()
{
    current = (int)round(getBattCurrent());
    voltage = (int)round(getBattVoltage());
    voltageOc = (int)round(getBattOcVoltage());

    getBattState();

    if (voltageOc >= 4200)
    {
        if (isCharging)
        {
            percent = State == 160 ? 100 : 99;
            return;
        }
        percent = 100;
    }
    else if (voltageOc <= 3000)
        percent = 0;
    else
        percent = (int)round((voltageOc - 3000) / 12);
}

void IP5108::scan_i2c()
{
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        i2c->beginTransmission(address);
        error = i2c->endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
    delay(5000);
}