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

#ifndef STRATOLIT_BQ25611D_H
#define STRATOLIT_BQ25611D_H

#include <Arduino.h>
#include <Wire.h>

class BQ25611D {
public:
  BQ25611D(uint8_t i2c_addr = 0x6B);

  // Initialization
  bool initChip();
  bool begin();

  // Setters (Set the registers to default values)
  /*
  Input Current Limit Register – REG00 (Address 0x00)

  EN_HIZ:
  Enable High Impedance (Hi-Z) mode in buck mode.
  0 – Disable (default)
  1 – Enable

  TS_IGNORE:
  Ignore TS pin condition during charge and boost.
  0 – Include TS pin in charge/boost enable condition (default)
  1 – Ignore TS pin. Always consider TS is good to allow charging
  and boost mode. NTC_FAULT bits are 000 to report normal
  status.

  BATSNS_DIS:
  Select either BATSNS pin or BAT pin to regulate battery voltage
  0 – Enable BATSNS in battery CV regulation. If the device fails
  BATSNS open/short detection (BATSNS_STAT = 1). Battery
  voltage is regulated through BAT pin. (default)
  1 – Disable BATSNS. Use BAT pin in battery CV regulation

  IINDPM:
  Input current limit setting (maximum limit, not
  typical)
  Offset: 100 mA, Step: 100 mA
  Range: 0–31 → 100 mA – 3200 mA
  Default: 2400 mA (IINDPM = 23)
  */

  bool setInputCurrentLimit(uint8_t EN_HIZ = 0, uint8_t TS_IGNORE = 0, uint8_t BATSNS_DIS = 0, uint8_t IINDPM = 23);
  
  /*
  Charger Control 0

  PFM_DIS:
  PFM disable in both buck and boost mode
    0 – Enable PFM mode (default)
    1 – Disable PFM mode

  WD_RST:
  2C Watchdog timer reset. Back to 0 after watchdog timer reset
    0 – Normal(default)
    1 – Reset watchdog timer

  BST_CONFIG:
  Boost mode enable. In charging case application, based on adapter
  plug-in or removal, the charger will automatically transit between
  charging mode and boost mode by setting BST_CONFIG bit and
  CHG_CONFIG bit both to 1
    0 – Boost mode disable (default)
    1 – Boost mode enable

  CHG_CONFIG:
  Battery charging buck mode enable. Charging is enabled when CE pin
  is pulled low, CHG_CONFIG bit is 1 and charge current is not zero
    0 – Disable charging
    1 – Enable charging (default)

  SYS_MIN:
  System minimum voltage setting
    0 – 3.0 V
    1 – 3.1 V
    2 – 3.2 V
    3 – 3.3 V
    4 – 3.4 V
    5 – 3.5 V (default)
    6 – 3.6 V

  MIN_VBAT_SEL:
  Minimum battery voltage when exiting boost mode. The rising threshold
  allows the device to start boost mode if other conditions are valid.
    0 – 2.8V VBAT falling, 3 V rising (default)
    1 – 2.5V VBAT falling, 2.8V rising
  */
  bool setChgCtrl0(uint8_t PFM_DIS = 0, uint8_t WD_RST = 0, uint8_t BST_CONFIG = 0, uint8_t CHG_CONFIG = 1, uint8_t SYS_MIN = 5, uint8_t MIN_VBAT_SEL = 0);

  /*
  Charge Current Limit

  BOOST_LIM:
  Boost mode current regulation limit (minimum current limit, not
  typical).
    0 – 0.5 A
    1 – 1.2 A (default)

  Q1_FULLON:
  In buck mode, charger will fully turn on Q1 RBFET according
  to this bit setting when IINDPM is below 700 mA. When
  IINDPM is over 700 mA, Q1 is always fully on. In boost mode ,
  Q1 is always fully on too, regardless of this bit setting.
    0 – Partially turn on Q1 for better regulation accuracy when
    IINDPM is below 700 mA. (default)
    1 – Fully turn on Q1 for better efficiency when IINDPM is below
    700 mA.

  ICHG:
  Fast charge current setting
  Default: 1020 mA (17)
  Range: 0 mA (0) –
  3000 mA (50)
  ICHG 0 mA disables charge.
  ICHG > 3000 mA is clamped to
  3000 mA (0)
    0–63 – Fast charge current limit (step = 60 mA)
          e.g., 21 = 1.26 A
  */
  bool setChgCurrentLimit(uint8_t BOOST_LIM = 1, uint8_t Q1_FULLON = 0, uint8_t ICHG = 17);

  /*
  Pre-charge and Termination Current

  IPRECHG:
  Pre-charge current setting
  Default: 180 mA (2)
  Range: 60 mA (0) – 780 mA
  (12)
  Offset: 60 mA
  Note: IPRECHG > 780 mA is
  clamped to 780 mA (12)
    0–15 – Pre-charge current limit (step = 60 mA)

  ITERM:
  Termination current setting
  Default: 180 mA (2)
  Range: 60 mA – 780 mA (12)
  Offset: 60 mA
    0–15 – Termination current limit (step = 60 mA)
  */

  bool setPreTermCurrent(uint8_t IPRECHG = 2, uint8_t ITERM = 2);

  /*
  Battery Voltage Limit

  VBATREG:
  Battery voltage setting, also called VREG.
  Default: 4.190 V (8)
  00000 – 3.494 V
  00001 – 3.590 V
  00010 – 3.686 V
  00011 – 3.790 V
  00100 – 3.894 V
  00101 – 3.990 V
  00110 – 4.090 V
  00111 – 4.140 V
  01000 – 4.190 V
  01001 - 11111 – 4.290 V - 4.510 V, 10 mV/step
  01110 4.340 V, 10011 4.390V, 11000 4.440 V, 11101 4.490 V

  TOPOFF_TIMER:Top-off timer setting.
  00 – Disabled (Default)
  01 – 15 minutes
  10 – 30 minutes
  11 – 45 minutes

  VRECHG:
  Battery recharge threshold setting.
    0 – 100 mV (default)
    1 – 200 mV
  */

  bool setBatteryVoltageLimit(uint8_t VBATREG = 8, uint8_t TOPOFF_TIMER = 0, uint8_t VRECHG = 0);

/*
Charger Control 1

EN_TERM:
  Battery charging termination enable.
  0 – Disable
  1 – Enable (default)

WATCHDOG:
  Watchdog timer setting.
  0 – Disabled
  1 – 40 s
  2 – 80 s
  3 – 160 s (default)

EN_TIMER:
  Battery charging safety timer enable, including both fast
  charge and pre-charge timers. Pre-charge timer is 2 hours.
  Fast charge timer is set by REG05[2]
  0 – Disable
  1 – Enable timer (default)

CHG_TIMER:
  Battery fast charging safety timer setting.
  0 – 20 hrs
  1 – 10 hrs (default)

TREG:
  Thermal Regulation Threshold:
  0 – 90°C
  1 – 110°C (default)

JEITA_VSET:
  Battery voltage setting during JEITA warm (T3 - T5,
  typically 45C - 60C)
  0 – Set Charge Voltage to 4.1 V (max) (default)
  1 – Set Charge Voltage to VREG
*/

  bool setChgCtrl1(uint8_t EN_TERM = 1, uint8_t WATCHDOG = 1, uint8_t EN_TIMER = 1, uint8_t CHG_TIMER = 1, uint8_t TREG = 1, uint8_t JEITA_VSET = 0);

  /*
  Charger Control 2

  OVP (Input OVP Threshold):
    VACOV threshold during buck mode and boost mode.
    00 – 5.85 V
    01 – 6.4 V (5-V input)
    10 – 11 V (9-V input)
    11 – 14.2 V (12-V input) (default

  BOOSTV (Boost Mode Output Voltage):
    Boost regulation voltage setting
    0 – 4.6 V
    1 – 4.75 V
    2 – 5.0 V (default)
    3 – 5.15 V

  VINDPM (Input Voltage DPM Threshold):
    VINDPM threshold setting
    0 – 3.9 V
    ...
    6 – 4.5 V (default)
    ...
    15 – 5.4 V
    (Offset: 3.9 V + 0.1 V * value)
  */

  bool setChgCtrl2(uint8_t OVP = 3, uint8_t BOOSTV = 2, uint8_t VINDPM = 6);

  /*
  Charger Control 3

  IINDET_EN:
    Force input source type detection. After the detection is complete, this
    bit returns to 0.
    0 – Not in input current limit detection. (default)
    1 – Force input current limit detection when adapter is present.

  TMR2X_EN:
    Safety timer is slowed by 2X during input DPM, JEITA cool/warm or
    thermal regulation.
    0 – Disable. Safety timer duration is set by REG05[2].
    1 – Safety timer slowed by 2X during input DPM (both V and I) or JEITA
    cool/warm (except ICHG=100%), or thermal regulation. (default)

  BATFET_DIS:
    BATFET Q4 ON/OFF control. Set this bit to 1 to enter ship mode. To
    reset the device with adapter present, the host shall set
    BATFET_RST_WVBUS to 1 and then BATFET_DIS to 1.
    0 – Turn on Q4. (default)
    1 – Turn off Q4 after tBATFET_DLY delay time (REG07[3])

  BATFET_RST_WVBUS:
    Start BATFET full system reset with or without adapter present.
    0 – Start BATFET full system reset after adapter is removed from
    VBUS. (default)
    1 – Start BATFET full system reset when adapter is present on VBUS.

  BATFET_DLY:
    Delay from BATFET_DIS (REG07[5]) set to 1 to BATFET turn off during
    ship mode.
    0 – Turn off BATFET immediately when BATFET_DIS bit is set.
    1 – Turn off BATFET after tBATFET_DLY (typ 10 s) when BATFET_DIS bit
    is set. (default)

  BATFET_RST_EN:
    Enable BATFET full system reset. The time to start of BATFET full
    system reset is based on the setting of BATFET_RST_WVBUS bit.
    0 – Disable BATFET reset function
    1 – Enable BATFET reset function when REG07[5] is also 1. (default)

  VINDPM_BAT_TRACK:
    Sets VINDPM to track BAT voltage. Actual VINDPM is higher of register
    value and VBAT + VINDPM_BAT_TRACK.
    00 – Disable function (VINDPM set by register) (default)
    01 – VBAT + 200 mV
    10 – VBAT + 250 mV
    11 – VBAT + 300 mV
  */

  bool setChgCtrl3(uint8_t IINDET_EN = 0, uint8_t TMR2X_EN = 1, uint8_t BATFET_DIS = 0, uint8_t BATFET_RST_WVBUS = 0, uint8_t BATFET_DLY = 1, uint8_t BATFET_RST_EN = 1, uint8_t VINDPM_BAT_TRACK = 0);

  /*
  Charger Control 4 – REG0C

  JEITA_COOL_ISET (bits 7:6):
  Fast charge current setting during cool temperature range (T1 - T2), as
  percentage of ICHG in REG02[5:0].
  00 – No Charge
  01 – 20% of ICHG (default)
  10 – 50% of ICHG
  11 – 100% of ICHG (safety timer does not become 2X)

  JEITA_WARM_ISET (bits 5:4):
    Fast charge current setting during warm temperature range (T3 - T5), as
    percentage of ICHG in REG02[5:0].
    00 – No Charge
    01 – 20% of ICHG
    10 – 50% of ICHG
    11 – 100% of ICHG (safety timer does not become 2X) (default)

  JEITA_VT2 (bits 3:2):
    00 – VT2% = 70.75% (5.5°C)
    01 – VT2% = 68.25% (10°C) (default)
    10 – VT2% = 65.25% (15°C)
    11 – VT2% = 62.25% (20°C)

  JEITA_VT3 (bits 1:0):
    00 – VT3% = 48.25% (40°C)
    01 – VT3% = 44.75% (44.5°C) (default)
    10 – VT3% = 40.75% (50.5°C)
    11 – VT3% = 37.75% (54.5°C)
  */

  bool setChargerControl4(uint8_t JEITA_COOL_ISET = 1, uint8_t JEITA_WARM_ISET = 3, uint8_t JEITA_VT2 = 1, uint8_t JEITA_VT3 = 1);

  // Status (Reads status registers)
  void getChargerStatus0();
  void getChargerStatus1();
  void getChargerStatus2();
  void getPartInfo();

  /*
  Triggers register reset by setting REG_RST (bit 7 of REG0B)
  Bit self-clears after reset completes.
  */
  bool resetPartRegisters();
  void initAllChargerRegisters();


private:
  uint8_t _addr;
  uint8_t readRegs(uint8_t reg);
  void writeRegs(uint8_t reg, uint8_t val);

  // Register addresses
  static const uint8_t reg_Iin = 0x00;
  static const uint8_t reg_CC0 = 0x01;
  static const uint8_t reg_Ichg = 0x02;
  static const uint8_t reg_Iter = 0x03;
  static const uint8_t reg_Vbat = 0x04;
  static const uint8_t reg_CC1 = 0x05;
  static const uint8_t reg_CC2 = 0x06;
  static const uint8_t reg_CC3 = 0x07;
  static const uint8_t reg_st0 = 0x08;
  static const uint8_t reg_st1 = 0x09;
  static const uint8_t reg_st2 = 0x0A;
  static const uint8_t reg_id  = 0x0B;
  static const uint8_t reg_CC4 = 0x0C;
};

#endif
