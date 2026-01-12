# BQ25798-mppt-solar-charger
BQ25798 I2C Controlled, 1- to 4-Cell, 5-A Buck-Boost Battery Charger with MPPT for Solar Panels.

![ast](doc/V2.1/BQ25798%20Test%20Board%20V2.1%203d%20noBG.png)

This project is a compact, efficient battery charger based on the TI BQ25798 IC. It is capable of charging 1s to 4s lithium-ion battery packs with up to 5A output current. The integrated buck-boost converter supports a wide range of input sources, including USB-C and solar panels.

Key features:
- I2C control interface for full configuration and monitoring
- Integrated Maximum Power Point Tracking (MPPT) to maximize efficiency from solar panels
- Automatic input source detection and power path management
- Programmable charging profiles and safety features
- Ideal for portable, off-grid, and solar-powered embedded applications

This board is suitable for developers building energy-efficient, battery-powered systems that rely on variable power sources like small PV panels.

---
## Change Log
### Mistakes V1.0
1. no Marking on PCB of min max Voltages.
2. CE Pin not having a resistor to pull LOW to enable chip.
3. too small footprints for soldering. 

### V2.0 changes
1. Added support for Dual Power Input
2. Added Backup Mode Support
3. changed charging current to 1A
4. changed Voltage divider for NTC temperature
5. changed PCB layout to accommodate new 2 inputs and outputs
6. added better description to PCB

### Mistakes V2.0
1. VAC1 and VAC2 connected to VBUS instead of VIN1 and VIN2.

### V2.1 changes
1. Fixed VAC issues of V2.0
2. changed R6 (NTC resistor Network) to right value of 4.3k Ohm


## [OSHWLab](https://oshwlab.com/georg.marek687/bq25798-test-board-mppt-solar)
## [View BQ25798 docs](doc/BQ25798_docs.pdf)