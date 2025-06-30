/*
  * StratoLit-BQ25611D.cpp
  * https://github.com/LCerimeli/StratoLit-BQ25611D
  * 
  * The MIT License (MIT)
  *
  * Copyright (c) 2025 Lucas Cerimeli https://github.com/LCerimeli/StratoLit-BQ25611D
  * All rights reserved.
  *
  * You may copy, alter and reuse this code in any way you like, but please leave
  * reference to https://github.com/LCerimeli/StratoLit-BQ25611D in your comments if you redistribute this code.
  *
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  * THE SOFTWARE. 
*/

#include "StratoLit-BQ25611D.h"

BQ25611D::BQ25611D(uint8_t i2c_addr) : _addr(i2c_addr) {}

bool BQ25611D::begin() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nInitializing BQ25611D\n");
  return initChip();
}

bool BQ25611D::initChip() {
  uint8_t info = readRegs(reg_id);
  if (info == 0b01010100) {
    Serial.print("Chip ID: ");
    Serial.println(info, BIN);
    Serial.println();
    return true;
  }
  Serial.print("Chip not located, check connection. Value: ");
  Serial.println(info, BIN);
  Serial.println();
  return false;
}

uint8_t BQ25611D::readRegs(uint8_t reg) {
  uint8_t val = 0xFF;
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission() == 0) {
    Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
    val = Wire.read();
  }
  return val;
}

void BQ25611D::writeRegs(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}



void BQ25611D::getChargerStatus0() {
  uint8_t val = readRegs(reg_st0);
  uint8_t vbus_stat = (val >> 5) & 0x07;
  uint8_t chrg_stat = (val >> 3) & 0x03;
  bool therm_stat = (val >> 1) & 0x01;
  bool vsys_stat = val & 0x01;

  Serial.println("REG08 - Charger Status 0:");

  const char* vbus_status_str[] = {
    "Unknown or No Input", "USB SDP", "USB CDP", "USB DCP",
    "Adjustable High Voltage DCP", "Unknown Adapter", "Non-Standard Adapter", "OTG"
  };
  Serial.print("  VBUS_STAT: "); Serial.println(vbus_status_str[vbus_stat]);

  const char* chrg_status_str[] = {
    "Not Charging", "Pre-charge or trickle charge", "Fast Charging", "Charge Termination"
  };
  Serial.print("  CHRG_STAT: "); Serial.println(chrg_status_str[chrg_stat]);

  Serial.print("  THERM_STAT: ");
  Serial.println(therm_stat ? "In thermal regulation" : "Not in thermal regulation");

  Serial.print("  VSYS_STAT: ");
  Serial.println(vsys_stat ? "In SYS_MIN regulation" : "Not in SYS_MIN regulation");
}

void BQ25611D::getChargerStatus1() {
  uint8_t val = readRegs(reg_st1);
  bool watchdog_fault = (val >> 7) & 0x01;
  bool boost_fault = (val >> 6) & 0x01;
  uint8_t chrg_fault = (val >> 4) & 0x03;
  bool bat_fault = (val >> 3) & 0x01;
  uint8_t ntc_fault = val & 0x07;

  Serial.println("REG09 - Charger Status 1:");

  Serial.print("  WATCHDOG_FAULT: ");
  Serial.println(watchdog_fault ? "Watchdog timer expiration, device is in default mode." : "Normal, device is in host mode");

  Serial.print("  BOOST_FAULT: ");
  Serial.println(boost_fault ? "Boost fault detected" : "Normal");

  const char* chrg_fault_str[] = {
    "Normal", "Input fault", "Thermal shutdown", "Charge safety timer expiration"
  };
  Serial.print("  CHRG_FAULT: "); Serial.println(chrg_fault_str[chrg_fault]);

  Serial.print("  BAT_FAULT: ");
  Serial.println(bat_fault ? "Battery over-voltage" : "Normal");

  const char* ntc_fault_str[] = {
    "Normal", "Unknown", "Warm", "Cool", "Unknown", "Cold", "Hot", "Unknown"
  };
  Serial.print("  NTC_FAULT: "); Serial.println(ntc_fault_str[ntc_fault]);
}

void BQ25611D::getChargerStatus2() {
  uint8_t val = readRegs(reg_st2);

  bool vbus_gd = (val >> 7) & 0x01;
  bool vindpm_stat = (val >> 6) & 0x01;
  bool iindpm_stat = (val >> 5) & 0x01;
  bool batsns_stat = (val >> 4) & 0x01;
  bool topoff_active = (val >> 3) & 0x01;
  bool acov_stat = (val >> 2) & 0x01;
  bool vindpm_int_mask = (val >> 1) & 0x01;
  bool iindpm_int_mask = val & 0x01;

  Serial.println("REG0A - Charger Status 2:");
  Serial.print("  VBUS_GD: "); Serial.println(vbus_gd ? "VBUS passes poor source detection" : "VBUS does not pass poor source detection");
  Serial.print("  VINDPM_STAT: "); Serial.println(vindpm_stat ? "In VINDPM" : "Not in VINDPM");
  Serial.print("  IINDPM_STAT: "); Serial.println(iindpm_stat ? "In IINDPM" : "Not in IINDPM");
  Serial.print("  BATSNS_STAT: "); Serial.println(batsns_stat ? "BATSNS pin open/short" : "BATSNS connected");
  Serial.print("  TOPOFF_ACTIVE: "); Serial.println(topoff_active ? "Top-off timer active" : "Top-off timer not active");
  Serial.print("  ACOV_STAT: "); Serial.println(acov_stat ? "In ACOV (over-voltage)" : "Not in ACOV");
  Serial.print("  VINDPM_INT_MASK: "); Serial.println(vindpm_int_mask ? "INT masked during VINDPM" : "INT asserted during VINDPM");
  Serial.print("  IINDPM_INT_MASK: "); Serial.println(iindpm_int_mask ? "INT masked during IINDPM" : "INT asserted during IINDPM");
}

void BQ25611D::getPartInfo() {
  uint8_t val = readRegs(reg_id);
  bool reg_rst = (val >> 7) & 0x01;
  uint8_t pn = (val >> 3) & 0x0F;

  Serial.println("REG0B - Part Information:");
  Serial.print("  REG_RST: "); Serial.println(reg_rst ? "Triggering reset" : "Keep current settings");
  Serial.print("  Part Number (PN): "); Serial.print(pn, BIN);
  Serial.println(pn == 0x0A ? " (BQ25611D)" : " (Unknown/Unexpected)");
}

bool BQ25611D::resetPartRegisters() {
  uint8_t val = readRegs(reg_id);
  val |= 0x80;
  writeRegs(reg_id, val);
  return !(readRegs(reg_id) & 0x80);
}

bool BQ25611D::setInputCurrentLimit(uint8_t EN_HIZ, uint8_t TS_IGNORE, uint8_t BATSNS_DIS, uint8_t IINDPM) {
  uint8_t val = ((EN_HIZ & 0x01) << 7) |
                ((TS_IGNORE & 0x01) << 6) |
                ((BATSNS_DIS & 0x01) << 5) |
                (IINDPM & 0x1F);
  writeRegs(reg_Iin, val);
  return readRegs(reg_Iin) == val;
}

bool BQ25611D::setChgCtrl0(uint8_t PFM_DIS, uint8_t WD_RST, uint8_t BST_CONFIG, uint8_t CHG_CONFIG, uint8_t SYS_MIN, uint8_t MIN_VBAT_SEL) {
  uint8_t val = ((PFM_DIS & 0x01) << 7) | ((WD_RST & 0x01) << 6) | ((BST_CONFIG & 0x01) << 5) |
                ((CHG_CONFIG & 0x01) << 4) | ((SYS_MIN & 0x07) << 1) | (MIN_VBAT_SEL & 0x01);
  writeRegs(reg_CC0, val);
  return readRegs(reg_CC0) == val;
}

bool BQ25611D::setChgCurrentLimit(uint8_t BOOST_LIM, uint8_t Q1_FULLON, uint8_t ICHG) {
  uint8_t val = ((BOOST_LIM & 0x01) << 7) | ((Q1_FULLON & 0x01) << 6) | (ICHG & 0x3F);
  writeRegs(reg_Ichg, val);
  return readRegs(reg_Ichg) == val;
}

bool BQ25611D::setPreTermCurrent(uint8_t IPRECHG, uint8_t ITERM) {
  uint8_t val = ((IPRECHG & 0x0F) << 4) | (ITERM & 0x0F);
  writeRegs(reg_Iter, val);
  return readRegs(reg_Iter) == val;
}

bool BQ25611D::setBatteryVoltageLimit(uint8_t VBATREG, uint8_t TOPOFF_TIMER, uint8_t VRECHG) {
  uint8_t val = ((VBATREG & 0x1F) << 3) | ((TOPOFF_TIMER & 0x03) << 1) | (VRECHG & 0x01);
  writeRegs(reg_Vbat, val);
  return readRegs(reg_Vbat) == val;
}

bool BQ25611D::setChgCtrl1(uint8_t EN_TERM, uint8_t WATCHDOG, uint8_t EN_TIMER, uint8_t CHG_TIMER, uint8_t TREG, uint8_t JEITA_VSET) {
  uint8_t val = ((EN_TERM & 0x01) << 7) | ((WATCHDOG & 0x03) << 5) | ((EN_TIMER & 0x01) << 4) |
                ((CHG_TIMER & 0x01) << 3) | ((TREG & 0x01) << 2) | ((JEITA_VSET & 0x01) << 1);
  writeRegs(reg_CC1, val);
  return readRegs(reg_CC1) == val;
}

bool BQ25611D::setChgCtrl2(uint8_t OVP, uint8_t BOOSTV, uint8_t VINDPM) {
  uint8_t val = ((OVP & 0x03) << 6) | ((BOOSTV & 0x03) << 4) | (VINDPM & 0x0F);
  writeRegs(reg_CC2, val);
  return readRegs(reg_CC2) == val;
}

bool BQ25611D::setChgCtrl3(uint8_t IINDET_EN, uint8_t TMR2X_EN, uint8_t BATFET_DIS, uint8_t BATFET_RST_WVBUS,
                            uint8_t BATFET_DLY, uint8_t BATFET_RST_EN, uint8_t VINDPM_BAT_TRACK) {
  uint8_t val = ((IINDET_EN & 0x01) << 7) |
                ((TMR2X_EN & 0x01) << 6) |
                ((BATFET_DIS & 0x01) << 5) |
                ((BATFET_RST_WVBUS & 0x01) << 4) |
                ((BATFET_DLY & 0x01) << 3) |
                ((BATFET_RST_EN & 0x01) << 2) |
                (VINDPM_BAT_TRACK & 0x03);
  writeRegs(reg_CC3, val);
  return readRegs(reg_CC3) == val;
}

bool BQ25611D::setChargerControl4(uint8_t JEITA_COOL_ISET, uint8_t JEITA_WARM_ISET, uint8_t JEITA_VT2, uint8_t JEITA_VT3) {
  uint8_t val = ((JEITA_COOL_ISET & 0x03) << 6) |
                ((JEITA_WARM_ISET & 0x03) << 4) |
                ((JEITA_VT2 & 0x03) << 2) |
                (JEITA_VT3 & 0x03);
  writeRegs(reg_CC4, val);
  return readRegs(reg_CC4) == val;
}

void BQ25611D::initAllChargerRegisters() {
  if (!initChip()) {
    Serial.println("Charger chip initialization failed!");
    return;
  }
  getChargerStatus0();
  getChargerStatus1();
  getChargerStatus2();
  getPartInfo();

  Serial.println(setInputCurrentLimit() ? "Input Current Limit set" : "Input Current Limit failed to set");
  Serial.println(setChgCtrl0() ? "Charger Control 0 set" : "Charger Control 0 failed to set");
  Serial.println(setChgCurrentLimit() ? "Charge Current Limit set" : "Charge Current Limit failed to set");
  Serial.println(setPreTermCurrent() ? "Pre-charge / Termination Current set" : "Pre-charge / Termination Current failed to set");
  Serial.println(setBatteryVoltageLimit() ? "Battery Voltage Limit set" : "Battery Voltage Limit failed to set");
  Serial.println(setChgCtrl1() ? "Charger Control 1 set" : "Charger Control 1 failed to set");
  Serial.println(setChgCtrl2() ? "Charger Control 2 set" : "Charger Control 2 failed to set");
  Serial.println(setChgCtrl3() ? "Charger Control 3 set" : "Charger Control 3 failed to set");
  Serial.println(setChargerControl4() ? "Charger Control 4 set" : "Charger Control 4 failed to set");
}