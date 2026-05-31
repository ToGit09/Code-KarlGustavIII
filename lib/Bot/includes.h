#ifndef INCLUDES_H
#define INCLUDES_H

#include <Arduino.h>
#include <Wire.h>              // for port expanders (Motor direction control, switches, Kicker)
#include <Adafruit_NeoPixel.h> // for RGB LEDs
#include <SPI.h>               // for ADC, Display and touch controller
#include <Adafruit_BNO055.h>   // compass sensor
#include <elapsedMillis.h>     // Timing
#include <IR_ring.h>           // IR-Ring Library
#include <ESC.h>               // ESC Library
#include <SRF08.h>             // Ultraschallsensor Library
#include <INA.h>               // INA219 Library


#ifndef PARAMETER_H
#include "Data/Parameter.h"
#endif

#ifndef GLOBALEVARIABLEN_H
#include "Data/GlobaleVariablen.h"
#endif

#ifndef READ_H
#include "executiveLayer/Read.h"
#endif

#ifndef ACTION_H
#include "executiveLayer/Action.h"
#endif

#endif