//rudimentary hardware test program

#include <stdio.h>               //needed at least for displaying fixed-width ADC values using sprintf
#include <Arduino.h>
#include <Wire.h>                // for port expanders (Motor direction control, switches, Kicker)
//#include "portexpander.h"      //tests use direct I2C accesses
#include <Adafruit_NeoPixel.h>   // for RGB LEDs
#include <SPI.h>                 // for ADC, Display and touch controller
#include <Adafruit_BNO055.h>     //compass sensor

#include <Adafruit_GFX.h>        // display: Core graphics library
#include <Adafruit_ILI9341.h>    // display: Hardware-specific library
#include <TouchScreen.h>         // display

//not used, we use direct I2C accesses
//TCA9534 expander0 = TCA9534(0x20, &Wire1);
//TCA9534 expander1 = TCA9534(0x21, &Wire1);

//RGB LEDs use the Adafruit NeoPixel library
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel RGB_LEDs = Adafruit_NeoPixel(10, 20, NEO_GRB + NEO_KHZ800);

//compass is a BNO055 module Adafruit containing a BNO055 from Bosch Sensortec
Adafruit_BNO055 Compass = Adafruit_BNO055(55, 0x28);

//Display is a 2,4-inch display from Adafruit with resistive touch (RT) controller
Adafruit_ILI9341 Display = Adafruit_ILI9341(7, 8);

//Touch controller is a separate object
// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
//TouchScreen Touch = TouchScreen(XP, YP, XM, YM, 300);

void InitSystem(void);
void PrintMenue(void);
void TestSwitches(void);
void TestMotors(void);
void TestRgbled(void);
void TestADC(void);
void TestBatVoltage(void);
void TestCompass(void);
void TestUltrasonic(void);
void SetUltrasonicAddress(void);
void TestLightBarrier(void);
void TestDisplay(void);
void TestBacklight(void);
void TestDebugConnector(void);

int main(void)
{
  char c;
  InitSystem(); //initialize all peripherals
  Serial.begin(115200); //not necessary on Teensy
  PrintMenue();
  while(1) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      c = Serial.read();

      Serial.println(c);
      switch(c) {
      case '1':
        TestSwitches();
        break;
      case '2':
        TestMotors();
        break;
      case '3':
        TestRgbled();
        break;
      case '4':
        TestADC();
        break;
      case '5':
        TestBatVoltage();
        break;
      case '6':
        TestCompass();
        break;
      case '7':
        TestUltrasonic();
        break;
      case '8':
        SetUltrasonicAddress();
        break;
      case '9':
        TestLightBarrier();
        break;
      case 'd':
        TestDisplay();
        break;
      case 'b':
        TestBacklight();
        break;
      case 'D':
        TestDebugConnector();
        break;
      case '?':
      case 'h':
        //PrintMenue();  help (Menue) will be printed anyway
        break;
      default:
        Serial.println("Dieses Kommando kenne ich nicht");
        break;
      }
      PrintMenue();
    }
  }
}

void InitSystem(void)
{
  //Bestandsaufnahme Schnittstellen am Teensy auf dem 1V1LWL Board
  //
  //Aus Sicht Teensy:
  //Pin       Funktion         Schnittstelle Angeschlossen
  //0         DISPLAY_IRQ      Interrupt     Display
  //1         DISPLAY_RT_CS    GPIO          Display (Touch)
  //2         DISPLAY_LITE     PWM/GPIO      Display (Beleuchtung)
  //3         START_SWITCH     GPIO          Schalter
  //4         PWM0             PWM           Motor0
  //5         PWM1             PWM           Motor1
  //6         PWM2             PWM           Motor2
  //7         DISPLAY_TFT_CS   GPIO?         Display
  //8         DISPLAY_TFT_DC   ??            Display
  //9         PWM3             PWM           Motor3 ???
  //10        ADC_CS           SPI/GPIO      IR-S.-ADC
  //11        MOSI             SPI           IR-S.-ADC, Display, Touch
  //12        MISO             SPI           "
  //13        SCK              SPI           "
  //14        TNS_TX/PIXY_RX   ??            Kamera
  //15        TNS_RX/PIXY_TX   ??            Kamera
  //16        TNS_RX/DEBUG_TX  ??            Debugstecker
  //17        TNS_TX/DEBUG_RX  ??            Debugstecker
  //18        SDA_3V3          I2C           Portexpander(20,21),J7(??),BN0055/Kompass,J6/Ultraschall(??),
  //19        SCL_3V3          I2C           "
  //20        RGB_DATA_3V3     GPIO?         RGB LEDs (10xWS2812B, J8)
  //21        LIGHT_BARRIER    GPIO?         Lichtschranke
  //22        BAT_VOLTAGE      ADC           Batteriespannung
  //23        LED0             GPIO          LED?
  /*************************************************************************/
  /* pin 18,19: I2C to port expanders (Kicker, switches, motor directions */
  Wire.begin(); //start I2C operation in master mode
  Wire.beginTransmission(0x20); //address first port expander
  Wire.write(0x03);             // select config register
  Wire.write(0xFC);             // Bits 0,1 output, others input
  Wire.endTransmission();
  Wire.beginTransmission(0x20); //address first port expander
  Wire.write(0x01);             // select output register
  Wire.write(0x00);             // set Kicker and Kicker_SW to zero
  Wire.endTransmission();
  Wire.begin(); //start I2C operation in master mode
  Wire.beginTransmission(0x21); //address second port expander
  Wire.write(0x01);             // select output register
  Wire.write(0x00);             // set all ports to zero before switching direction
  Wire.endTransmission();
  Wire.beginTransmission(0x21); //address second port expander
  Wire.write(0x03);             // select config register
  Wire.write(0x00);             // all ports ouput
  Wire.endTransmission();
  /*************************************************************************/
  /* pin 4,5,6,9: PWM for motor speed control */
  /* 0=free running, 256 = always on/brake */
  analogWriteFrequency(4, 5000); /* 5kHz so we can hear during test... */
  analogWriteFrequency(5, 5000); /* 5kHz so we can hear during test... */
  analogWriteFrequency(6, 5000); /* 5kHz so we can hear during test... */
  analogWriteFrequency(9, 5000); /* 5kHz so we can hear during test... */
  /*************************************************************************/
  /* pin 10-13 SPI with ADC chip select*/
  SPI.begin();
  //ToDo: How to make sure the output does not glitch when switching direction?
  pinMode(10, OUTPUT);  /* ADC chip select */
  digitalWrite(10, HIGH);  /* prepare default state of ADC chip select */
  pinMode(1, OUTPUT);  /* resistive touch (RT) chip select */
  digitalWrite(1, HIGH);  /* prepare default state resistive touch (RT) chip select */
  pinMode(7, OUTPUT);  /* TFT chip select */
  digitalWrite(7, HIGH);  /* prepare default state of TFT chip select */
  /*************************************************************************/
  /* pin 20 RGB LEDs */
  RGB_LEDs.begin();
  delay(10);         //1ms tested 10x ok, but to be sure stay with 10
  RGB_LEDs.setBrightness(0);
  RGB_LEDs.show(); // Initialize all pixels to 'off'
  /*************************************************************************/
  /* pin 23 Status LED*/
  pinMode(23, OUTPUT);    /* Status LED */
  digitalWrite(23, HIGH); /* Status LED */
  /*************************************************************************/
  /* pins 7,8 (und SPI): Display */
  /* default orientation is 240x320 which is modified to 320x240 */
  /* CAUTION: ILI9341_TFTWIDTH is still 240! */
  Display.begin();
  Display.setRotation(3);            //same orientation as PCB silkscreen,
  Display.fillScreen(ILI9341_BLACK); //clear display
  /*************************************************************************/
  /* pin 2: Display backlight PWM */
  analogWriteFrequency(2, 1000); /* 1kHz should fit */
  analogWrite(2,255);
  /*************************************************************************/
  /* pin 16,17: debug connector (second serial port) */
  Serial4.begin(115200);
}

void PrintMenue(void)
{
  Serial.println("1  Show switch status");
  Serial.println("2  Test motor controls");
  Serial.println("3  RGB LED test");
  Serial.println("4  ADC Test");
  Serial.println("5  Battery voltage");
  Serial.println("6  Compass");
  Serial.println("7  Ultrasonic");
  Serial.println("8  Set ultrasonic sensor address");
  Serial.println("9  Light barrier");
  Serial.println("d  Display");
  Serial.println("b  Display brightness (backlight)");
  Serial.println("D  Debug Connector (Serial4)");
  Serial.println("?  show this menue");   
  Serial.print("\nPlease select: ");
}

void TestSwitches(void)
{
  uint8_t stat;
  Serial.println("Switch test. press any key to exit.");
  Serial.println("BTN1 BTN2 BTN3 SWI1 SWI2 SWI3 Start");    
  for (;;) {
#if 1
    Wire.beginTransmission(0x20);       //address first port expander
    Wire.write(0x00);                   // select input register
    Wire.endTransmission(false);        // send repeated start instead of stop
    Wire.requestFrom(0x20, 1);          // Start a read access and expect one byte
    stat = Wire.read();                 // read inputs
    Wire.endTransmission();             //now send a stop
    Serial.print((stat & 0x04)? "off  " : "on   ");
    Serial.print((stat & 0x08)? "off  " : "on   ");
    Serial.print((stat & 0x10)? "off  " : "on   ");
    Serial.print((stat & 0x20)? "on   " : "off  ");
    Serial.print((stat & 0x40)? "on   " : "off  ");
    Serial.print((stat & 0x80)? "on   " : "off  ");
    Serial.print((digitalRead(3) == HIGH)? "on " : "off");
    Serial.print("\r");
    if (Serial.available() > 0) {
      // read the incoming byte:
      Serial.read();
      Serial.println("");
      break;
    }
#else
    Wire.beginTransmission(0x20);       //address first port expander
    Wire.write(0x00);                   // select input register
    Wire.endTransmission(false);        // send restart instead of stop
    Wire.requestFrom(0x20, 1);          // Start a read access and expect one byte
    stat = Wire.read();                 // read inputs
    Wire.endTransmission();             //now send a stop
    Serial.print("Button 1: ");
    Serial.println((stat & 0x04)? "off" : "on");
    Serial.print("Button 2: ");
    Serial.println((stat & 0x08)? "off" : "on");
    Serial.print("Button 3: ");
    Serial.println((stat & 0x10)? "off" : "on");
    Serial.print("Switch 1: ");
    Serial.println((stat & 0x20)? "on"  : "off");
    Serial.print("Switch 2: ");
    Serial.println((stat & 0x40)? "on"  : "off");
    Serial.print("Switch 3: ");
    Serial.println((stat & 0x80)? "on"  : "off");
    Serial.print("Start:    ");
    Serial.println((digitalRead(3) == HIGH)? "on" : "off");
    Serial.print("press q to exit, any other key to repeat ");
    while(1) {
      char c;
      if (Serial.available() > 0) {
        // read the incoming byte:
        c = Serial.read();
        if (c == 'q' || c == 'Q')
          return;
        else {
          Serial.println("");
          break;
        }
      }
    }
#endif
  }
}

void TestMotorsMenue() {
  Serial.println("Motor test\n");
  Serial.println("0..3: select motor\n"
                 "f:    let Motor run free\n"
                 "b:    brake motor\n"
                 "r:    rotate right\n"
                 "l:    rotate left\n"
                 "+:    increase speed\n"
                 "-:    decrease speed\n"
                 "?:    show this menue again\n"
                 "q,Q   exit motor test\n\n");
}

void TestMotorsUpdate(int motor, int dir, int speed) {
  uint8_t ports;
  //get current port state (slow but simple)
  Wire.beginTransmission(0x21); //address second port expander
  Wire.write(0x01);             // select output register
  Wire.endTransmission(false);  // send restart instead of stop
  Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
  ports = Wire.read();          // get current port state
  Wire.endTransmission();
  switch (motor) {
  case 0:
    analogWrite(4, (uint16_t)speed * 256 / 100);
    ports = (ports & 0xFC) |  ((dir & 0x03) << 0);
    break;
  case 1:
    analogWrite(5, (uint16_t)speed * 256 / 100);
    ports = (ports & 0xF3) |  ((dir & 0x03) << 2);
    break;
  case 2:
    analogWrite(6, (uint16_t)speed * 256 / 100);
    ports = (ports & 0xCF) |  ((dir & 0x03) << 4);
    break;
  case 3:
    analogWrite(9, (uint16_t)speed * 256 / 100);
    ports = (ports & 0x3F) |  ((dir & 0x03) << 6);
    break;
  }
  Wire.beginTransmission(0x21); //address second port expander
  Wire.write(0x01);             // select output register
  Wire.write(ports);            // update output ports
  Wire.endTransmission();
}

void TestMotors(void)
{
  int motor         = 0;      //current motor
  int dir           = 0;      //current mode (H-bridge)
  int speed         = 0;      //current speed (0...100%)
  int motor_n       = 0;      //new motor
  int dir_n         = 0;      //new mode (H-bridge)
  int speed_n       = 0;      //new speed
  boolean bReturn   = false;

  TestMotorsMenue();
  for (;;) {
    /*********************************************************************/
    /* show current state */
    Serial.print("motor ");
    Serial.print(motor);
    if (speed == 0 || dir == 0){
      Serial.print(" free run ");
    }
    else {
      if (dir == 1) {
        Serial.print(" ");
        Serial.print(speed);
        Serial.print("% right");
      }
      else if (dir == 2) {
        Serial.print(" ");
        Serial.print(speed);
        Serial.print("% left");
      }
      else if (dir == 3) {
        Serial.print(" brake");
      }   
    }
    Serial.flush();
    /*********************************************************************/
    /* wait for user input */
    while(1) {
      char c;
      if (Serial.available() > 0) {
        // read the incoming byte:
        c = Serial.read();
        switch (c) {
        case '0':
          motor_n = 0;
          break;
        case '1':
          motor_n = 1;
          break;
        case '2':
          motor_n = 2;
          break;
        case '3':
          motor_n = 3;
          break;
        case 'f':
          speed_n = 0;
          break;
        case 'b':
          dir_n = 3;
          break;
        case 'r':
          dir_n = 1;
          break;
        case 'l':
          dir_n = 2;
          break;
        case '+':
          if (speed < 100)
            speed_n++;
          break;
        case '-':
          if (speed)
            speed_n--;
          break;
        case '?':
          TestMotorsMenue();
          break;
        case 'q':
        case 'Q':
          bReturn = true;
          break;
        default:
          break;
        }
        break;
      }
    }
    /*********************************************************************/
    /* clean up and return to main menu */
    if (bReturn == true) {
      // switch all motors to free run
      TestMotorsUpdate(0, 0, 0);
      TestMotorsUpdate(1, 0, 0);
      TestMotorsUpdate(2, 0, 0);
      TestMotorsUpdate(3, 0, 0);
      Serial.print("\n");
      return;
    }
    /*********************************************************************/
    /* apply new settings */
    /* handle some specific state transitions */
    if (speed_n > 0 && dir == 0)
      dir = 1; //default to right turn
    if (motor != motor_n && speed) {
      TestMotorsUpdate(motor, 0, 0); // switch current motor to free run
      delay(500);
    }
    if (((dir == 1 && dir_n == 2) ||
         (dir == 2 && dir_n == 1)) &&
        speed) {
      TestMotorsUpdate(motor, 0, 0); // switch current motor to free run
      delay(500);
    }
    TestMotorsUpdate(motor_n, dir_n, speed_n);
    /* update current values for next run */
    motor = motor_n;
    dir   = dir_n;
    speed = speed_n;
    TestMotorsUpdate(motor, dir, speed);
    /*********************************************************************/
    /* jump to beginning of first line */
    Serial.print("            \r");
  }
}

void TestRgbled(void) {
  int i;
  Serial.println("RGB LEDs Test. Press any key to exit at the end of the sequence");
  RGB_LEDs.setBrightness(100);
  digitalWrite(23, HIGH); /* Status LED on */
  for (i = 0; i < 10; i++) {
    RGB_LEDs.setPixelColor(i, RGB_LEDs.Color(255, 0, 0));
    RGB_LEDs.show();
    delay(100);
  }
  delay(1000);
  digitalWrite(23, LOW); /* Status LED off */
  for (i = 0; i < 10; i++) {
    RGB_LEDs.setPixelColor(i, RGB_LEDs.Color(0, 255, 0));
    RGB_LEDs.show();
    delay(100);
  }
  digitalWrite(23, HIGH); /* Status LED on */
  delay(1000);
  for (i = 0; i < 10; i++) {
    RGB_LEDs.setPixelColor(i, RGB_LEDs.Color(0, 0, 255));
    RGB_LEDs.show();
    delay(100);
  }
  delay(1000);
  digitalWrite(23, LOW); /* Status LED off */
  for (i = 0; i < 10; i++) {
    RGB_LEDs.setPixelColor(i, RGB_LEDs.Color(16, 16, 16));
    RGB_LEDs.show();
    delay(100);
  }
  /* wait for user input */
  while(1) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      Serial.read();
      break;
    }
  }
  for (i = 0; i < 10; i++) {
    RGB_LEDs.setPixelColor(i, RGB_LEDs.Color(0, 0, 0));
    RGB_LEDs.show();
    delay(100);
  }
  digitalWrite(23, HIGH); /* Status LED off */
  return;
}

/* Test ADC / IR sensors */
/* Each SPI transfer */
/* a) performs a measurement of the previously selected channel */
/* b) selects the channel for the next measurement */
void TestADC(void) {
  int Ch;                /* current channel */
  uint16_t Cc;           /* control word to send to ADC (contains channel selection) */
  uint16_t u16Adcval[8]; /* conversion results */
  Serial.println("ADC (IR Sensor) Test. Press any key to exit");
  Serial.println("Kanal0       Kanal1       Kanal2       Kanal3       Kanal4       Kanal5       Kanal6       Kanal7");
  for (;;) {
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3)); /*8...16MHz allowed*/
    digitalWrite(10, LOW); /* activate chip select */
    for (Ch = 0; Ch < 8; Ch++) {
      Cc = Ch + 1;                        /* select channel for next conversion; very first one defaults to zero */
      Cc%= 8;                             /* limit to 0...7 (not really necessary) */
      Cc<<= 11;                           /* the ADC expects channel selection in bits 11...13 */
      u16Adcval[Ch] = SPI.transfer16(Cc); /* start conversion on next channel / get results of current channel */
      u16Adcval[Ch]>>= 2;                 /* conversion result is in bits 2...11 --> adjust to 0...9 */
    }
    digitalWrite(10, HIGH); /* deactivate chip select */
    SPI.endTransaction();
    /* show conversion results (outside of SPI transfers) */
    for (Ch = 0; Ch < 8; Ch++) {
      char  hexstring[6]; /* output string for value in hex (4 digits, space, \0) */
      float voltage;
      sprintf(hexstring, "%4.4X=", u16Adcval[Ch]);
      voltage = u16Adcval[Ch] * 3.3 / 1024;
      Serial.print(hexstring);
      Serial.print(voltage, 4);
      Serial.print("V ");
    }
    Serial.print("\r");   /* jump back to beginning of line */ 
    if (Serial.available() > 0) {
      Serial.read();            /* read but ignore incoming character */
      Serial.println("");
      break;
    }
  }
}

void TestBatVoltage(void)
{
  float voltage;
  voltage = analogRead(22);
  //voltage divider: 10k,2k4 --> 2.4/12.4 = 0.1935
  //multiplier frommeasrued voltage to battery voltage: 5.16666
  voltage*= 3.3;
  voltage/= 1024;
  voltage*= 5.1666;
  Serial.print("Battery voltage: ");
  Serial.print(voltage);
  Serial.println("V");
}

void TestCompass(void)
{
  sensor_t sensor;
  Serial.println("Orientation Sensor Test. press any key to exit"); Serial.println("");

  /* Initialise the sensor */
  if(!Compass.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.println("Can't detect Compass. Exiting.");
      return;
    }

  delay(1000);

  /* Use external crystal for better accuracy */
  Compass.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  Compass.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");

  /* display sensor values */
  for (;;) {
    /* Get a new sensor event */
    sensors_event_t event;
    uint8_t sys, gyro, accel, mag = 0;
    Compass.getEvent(&event);
    Serial.print(F("Orientation (x/y/z): "));
    Serial.print((float)event.orientation.x);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.y);
    Serial.print(F(" "));
    Serial.print((float)event.orientation.z);
    Serial.println(F(""));

    //Test: Umrechnung in 2D-Koordinatensystem
    {
      //x     dir_ist
      //0     90
      //359   91
      //1     89
      int dir_ist;
      dir_ist = int(roundf(event.orientation.x));
      dir_ist = 360 - dir_ist;
      dir_ist+= 90;
      dir_ist%= 360;
      Serial.println(dir_ist);
    }
        
    /* Also send calibration data for each sensor. */
    Compass.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.print(mag, DEC);
    Serial.print("              \r");

    delay(100 /* BNO055_SAMPLERATE_DELAY_MS */);
    if (Serial.available() > 0) {
      // read the incoming byte:
      Serial.read();
      break;
    }
  }
}

/* check any possible address for presence of a sensor and get a conversion if a sensor is present */
void TestUltrasonic(void)
{
  int idx;                      //zählt Sensoren durch
  int sensorAddress;            //I2C Adresse des aktuellen Sensors
  bool sensorPresent[16];       //hier wird gespeichert ob der Sensor vorhanden ist
  unsigned char rev;            //software revision of the sensor
  Serial.println("Ultrasonic sensor test. Press any key to exit");
  //check for available sensors
  sensorAddress = 0x70;
  for (idx = 0; idx < 16; idx++) {
    /* check for sensor presence by reading its software version */
    Wire.beginTransmission(sensorAddress);
    Wire.write(0x00);                   // select version register
    Wire.endTransmission(false);        // switch to read direction
    Wire.requestFrom(sensorAddress, 1); // Start a read access and expect one byte
    rev = Wire.read();                  // get software version
    Wire.endTransmission();
    if (rev != 0xFF)                    //anything that is not FF is a present sensor
      {
        Serial.print("Sensor found at ");
        Serial.print(sensorAddress, HEX);
        Serial.print("h, software version ");
        Serial.print(rev);
        Serial.println("h");
        sensorPresent[idx] = true;
      }
    else {
      Serial.print("No sensor at ");
      Serial.print(sensorAddress, HEX);
      Serial.println("h");
      sensorPresent[idx] = false;
    }
    sensorAddress++;
  }
  for (;;) {
    sensorAddress = 0x70;
    for (idx = 0; idx < 16; idx++) {
      if (sensorPresent[idx] == true) {
        uint16_t distance;
        Wire.beginTransmission(sensorAddress);
        Wire.write(0x00);                   // select version/command register
        Wire.write(0x51);                   // send command: read in cm
        Wire.endTransmission();
#if 0 /* set to 1 to enable polling for result, set to 0 for simply waiting */       
        rev = 0xFF;
        while (rev == 0xFF) {               //measurement is done if software revision is readable again
          Wire.beginTransmission(sensorAddress);
          Wire.write(0x00);                   // select version register
          Wire.endTransmission(false);        // switch to read direction
          Wire.requestFrom(sensorAddress, 1); // Start a read access and expect one byte
          rev = Wire.read();                  // get software version
          Wire.endTransmission();
        }
#else
        //                    delay(64); //simply wait until conversion is done
        delay(100); //simply wait until conversion is done
#endif
        //get first result from sensor
        Wire.beginTransmission(sensorAddress);
        Wire.write(0x02);                   // select first echo high-byte register
        Wire.endTransmission(false);        // switch to read direction
        Wire.requestFrom(sensorAddress, 2); // Start a read access and expect 17 range words
        distance = Wire.read() << 8;        //get high byte
        distance+= Wire.read();             //get low byte
        Wire.endTransmission();
        /* show first measurement result */
        Serial.print(distance, DEC);
        Serial.print("cm ");
      }
      sensorAddress++;
    }
    //jump back to beginning of line
    Serial.print("\r");
    /* exit if key pressed */
    if (Serial.available() > 0) {
      Serial.read(); // read and ingnore the incoming byte
      break;
    }
  }
}

/* Get a byte in hex from the terminal. First a prompt string is printed, then */
/* incoming characters are processed as follows:       */
/* q or Q:         show entered character and abort, returning FFFF   */
/* 0-9,a-f or A-F: show entered character and use it as 4-bit hexadecimal digit */
/* <Backspace>:    delete the last-entered character and go back one digit */
/* <Enter>:        use currently entered value, either one or two digits. */
uint16_t GetHexbyte(String text) {
  char          c;           //entered character
  int           digit = 0;   //indicates current digit (0 or 1; 2=we're finished)
  unsigned char hexbyte = 0; //entered value
  Serial.print(text);
  for (;;) {
    //get a character
    while (Serial.available()== 0); //wait for character
    c = Serial.read();              // read incoming character
    //check what was entered...
    if (c == 'q' || c == 'Q') {
      Serial.println(c); //show entered character...
      return 0xFFFF;     //...and abort
    }
    if (digit < 2 &&
        c >= '0'  && c <= '9') { //decimal digit
      Serial.print(c);         //show entered character.
      hexbyte<<= 4;            //make space in lower 4 bits
      hexbyte+= c - '0';       //convert from '0'-'9' to 0-9 and add to result
      digit++;                 //switch to next digit
      continue;                //restart loop-->get next character
    }
    if (digit < 2 &&
        c >= 'A' && c <= 'F') {  //hexadecimal digit (uppercase)
      Serial.print(c);         //show entered character.
      hexbyte<<= 4;            //make space in lower 4 bits
      hexbyte+= c - 'A' + 10;  //convert from 'A'-'F' to 10-15 and add to result
      digit++;                 //switch to next digit
      continue;                //restart loop-->get next character
    }
    if (digit < 2 &&
        c >= 'a' && c <= 'f') {  //hexadecimal digit (lowercase)
      Serial.print(c);         //show entered character.
      hexbyte<<= 4;            //make space in lower 4 bits
      hexbyte+= c - 'a' + 10;  //convert from 'a'-'a' to 10-15 and add to result
      digit++;                 //switch to next digit
      continue;                //restart loop-->get next character
    }
    if (digit > 0 &&
        c == '\b') {             //backspace character
      Serial.print('\b');      //delete last entered character
      digit--;                 //go back one digit
      hexbyte>>= 4;            //remove lower 4 bits
      continue;                //restart loop-->get next character
    }
    if (c == '\r' ||
        c == '\n') {             //carriage return or linefeed character = Enter
      delay(100);
      Serial.read();           //trow away newline after carriage return
      Serial.println("");
      return hexbyte;          //we're done, return current value, regardles whether 0, 1 or 2 digits
    }
    //        Serial.print("\a");          //any other character is not allowed --> beep
  }
}

void SetUltrasonicAddress(void) {
  int           oldAddress;
  int           newAddress;
  uint16_t      tmp;         //temporary address
  unsigned char rev;         //sensor software revision for checking presence
  Serial.println("Make sure that only one sensor is connected!\n"
                 "Note: minimal editing is supported (backspace).");
  tmp = GetHexbyte("Enter current 7-Bit I2C address (80...7F) or q to quit: ");
  if (tmp < 0x70 || tmp > 0x7F) {
    Serial.print("Invalid address entered: ");
    Serial.println(tmp, HEX);
    Serial.println(". Aborting");
    return;
  }
  else
    oldAddress = (unsigned char)tmp;
  tmp = GetHexbyte("Enter new 7-Bit I2C address (80...7F) or q to quit:     ");
  if (tmp < 0x70 || tmp > 0x7F) {
    Serial.print("Invalid address entered: ");
    Serial.println(tmp, HEX);
    Serial.println(". Aborting");
    return;
  }
  else
    newAddress = (unsigned char)tmp;
#ifdef DEBUG
  //show resulting address
  Serial.print("Old address: ");
  Serial.println(old_address, HEX);
  Serial.print("New address: ");
  Serial.println(new_address, HEX);
#endif
  //set address in sensor
  //1) check that old address is available
  Wire.beginTransmission(oldAddress);
  Wire.write(0x00);                   // select version register
  Wire.endTransmission(false);        // switch to read direction
  Wire.requestFrom(oldAddress, 1);    // Start a read access and expect one byte
  rev = Wire.read();                  // get software version
  Wire.endTransmission();
  if (rev == 0xFF) {                  //anything that is not FF is a present sensor
    Serial.println("No sensor found at old address. Aborting");
    return;
  }
  //2) set new addres
  Wire.beginTransmission(oldAddress);
  Wire.write(0x00);                   // select command/version register
  Wire.write(0xA0);                   // first command for setting address
  Wire.endTransmission();
  Wire.beginTransmission(oldAddress);
  Wire.write(0x00);                   // select command/version register
  Wire.write(0xAA);                   // second command for setting address
  Wire.endTransmission();
  Wire.beginTransmission(oldAddress);
  Wire.write(0x00);                   // select command/version register
  Wire.write(0xA5);                   // third command for setting address
  Wire.endTransmission();
  Wire.beginTransmission(oldAddress);
  Wire.write(0x00);                   // select command/version register
  Wire.write(newAddress << 1);        // new address (sensor wants 8-bit address)
  Wire.endTransmission();
  //3) check new address
  delay(1000);                        //it is not documented how long the sensor takes to change its address or if it requires a power cycle.
  Wire.beginTransmission(newAddress);
  Wire.write(0x00);                   // select version register
  Wire.endTransmission(false);        // switch to read direction
  Wire.requestFrom(newAddress, 1);    // Start a read access and expect one byte
  rev = Wire.read();                  // get software version
  Wire.endTransmission();
  if (rev == 0xFF) {                  //anything that is not FF is a present sensor
    Serial.println("No sensor found at new address. Aborting");
    return;
  }
  else {
    Serial.println("Sensor found at new address.");
    return;
  }
}

void TestLightBarrier(void)
{
  float volt;
  int val = analogRead(21);
  volt = val * 3.3 / 1024;
  Serial.print("light barrier voltage (3.3V=open, <3.3V=barrier detected): ");
  Serial.print(volt, 3);
  Serial.println("V");
}

void TestDisplay(void) {
  unsigned int x;
  Serial.println("Display test. Press any key to switch between test images.");
  /*********** Test image 1 ***********/
  Serial.println("color gradients with single pixel frame. Check image alignment and corect colors");
  Display.drawLine(0,                     0,                    0,                     ILI9341_TFTWIDTH - 1, ILI9341_WHITE);
  Display.drawLine(0,                     ILI9341_TFTWIDTH - 1, ILI9341_TFTHEIGHT - 1, ILI9341_TFTWIDTH - 1, ILI9341_WHITE);
  Display.drawLine(ILI9341_TFTHEIGHT - 1, ILI9341_TFTWIDTH - 1, ILI9341_TFTHEIGHT - 1, 0,                    ILI9341_WHITE);
  Display.drawLine(ILI9341_TFTHEIGHT - 1, 0,                    0,                     0,                    ILI9341_WHITE);
  for (x = 1; x < (ILI9341_TFTHEIGHT - 2); x++) {
    //each color gradient has a width of 240/3=80 (excluding white frame)
    //color coding: RRRRRGGGGGGBBBBB
    Display.drawLine(x,   1,  x,  79, (0x1F - (x * 32 / 320)) << 11);
    Display.drawLine(x,  80,  x, 159, (0x3F - (x * 64 / 320)) <<  5);
    Display.drawLine(x, 160,  x, 238, (0x3F - (x * 32 / 320)) <<  0);
  }
  //void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
  //void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
  Display.drawChar(2,  30, 'R', ILI9341_WHITE, ILI9341_RED,   3);
  Display.drawChar(2, 110, 'G', ILI9341_WHITE, ILI9341_GREEN, 3);
  Display.drawChar(2, 190, 'B', ILI9341_WHITE, ILI9341_BLUE , 3);
  while(!Serial.available());
  Serial.read();
  Display.fillScreen(ILI9341_BLACK); //clear display
}

void TestBacklight(void) {
  Serial.println("Backlight test. Press any key to exit.");
  Display.fillScreen(ILI9341_WHITE); //show full white display
  for(;;) {
    int pwmval;
    for (pwmval = 0; pwmval < 255; pwmval++) {
      analogWrite(2, pwmval);
      delay(20);
    }
    for (pwmval = 255; pwmval >= 0; pwmval--) {
      analogWrite(2, pwmval);
      delay(20);
    }
    if(Serial.available()) {
      Serial.read();
      analogWrite(2, 255);
      Display.fillScreen(ILI9341_BLACK); //clear display
      break;
    }
  }
}

void TestDebugConnector(void) {
  Serial.println("Debug connector test. Connect another serial port to debug connector.\n"
                 "and start a second serial monitor. All characters will be echoed.");
  Serial.println("Press any key on this interface to exit the test");
  if (Serial.available()) {
    Serial.read();
    return;
  }
  if (Serial4.available())
    Serial4.write(Serial4.read());
}
