# BQ25611D Library for the Esp32
  ![img](resources/BQ25611D.png "BQ25611D")

  This library was made to easily setup the BQ25611D Buck Battery Charger, to use in products developed by StratoLit.

## How it works
  The ```initAllChargerRegisters()``` function writes the default values of all the setup registers and read the values of the status registers.

  To change the values of the registers according to your needs just change the default variable values set in the header file, the datasheet can be found in the resources folder.

  To use it in the Arduino IDE select the ```ESP32 Dev Module``` board.