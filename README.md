# Arduino Focuser

## Overview

This Platform IO based Arduino code is to create a Focusser to be used with
Linux's INDI Astrophotography Software. This is just the start of the process,
there should be more to come, but here is the start.

This project starts with:
* The Dream Focuser driver in INDILIB. This driver has some limitations and drawbacks
  but the overall protocol is easy to implement.
* There are 3 main components to be used as part of the focuser:
  * Arduino Nano (any variety as long as it works
    with Platform IO)
  * Motor Driver, in this case a A4988 Driver. This
    driver is pretty simple and can be driven with 2
    wires. 
  * SDD1306 OLED Display Breakout Board from
    Adafruit for output and monitoring.
  * Possible DHT11 (or better, later) for Weather

The code in this repository is a redo of a previous 
attempt. This combination works perfectly on one 
Arduino Nano, but that code was lost in a drive crash. This code is to replicate and build on that.

