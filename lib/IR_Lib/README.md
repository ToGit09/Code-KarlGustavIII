# IR-Ring Library (IR_Lib)
***

*Aktualisiert: 26.02.2026*

Diese Alternative IR-Ring-Bibliothek bietet Unterstützung für den [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) zur Kommunikation mit der [IR-Ring](https://git.markdorf-robotics.de/PCBs/IR-Ring/-/releases/permalink/latest) Sensorplatine des Karl-Gustav III Roboters der Roboter-AG.

Die Firmware für den IR-Ring ist unter [Firmware/IR-Ring](https://git.markdorf-robotics.de/firmware/ir-ring-fw) zu finden.

***
## Beispielprogramm

``` c++
#include <Arduino.h>
#include <Wire.h>

#include "ir_ring.h"

#define I2C_BUS Wire1     // IRL-Ring on Wire1
#define I2C_SPEED 1000000 // 1 MHz

// IR-Ring setup
I2C_bus i2c1(I2C_BUS);
Device_handle ir_ring_handle = {i2c1, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

void setup()
{
  Serial.begin(115200);

  // I2C setup
  I2C_BUS.begin();
  I2C_BUS.setClock(1000000);
}

void loop()
{
  Serial.print("Ball angle: "); Serial.println(ir_ring.read_ball_angle());
  Serial.print("Ball distance: "); Serial.println(ir_ring.read_ball_distance());

  uint16_t calib_data[16];
  ir_ring.read_calibrated_values(calib_data);
  
  Serial.print("Calibrated data: ");
  for (int i=0; i<16; i++)
  {
    Serial.print(i+1); Serial.print(": ");
    Serial.print(calib_data[i]); Serial.print("  ");
  }
  Serial.println();

  delayMicroseconds(500);
}
```

