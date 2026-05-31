/** Port Belegung v3.0 ********************************************************************/
// Pin       Funktion          Verfahren      Angeschlossen
//----------------------------------------------------------------------------------------//
// 0         DISPLAY_IRQ      Interrupt      Display
// 1         DISPLAY_RT_CS    GPIO           Display (Touch)
// 2         DISPLAY_LITE     PWM/GPIO       Display (Beleuchtung)
// 3         START_SWITCH     GPIO           Schalter
// 4         PWM0             PWM            Motor0
// 5         PWM1             PWM            Motor1
// 6         PWM2             PWM            Motor2
// 7         DISPLAY_TFT_CS   GPIO?          Display
// 8         DISPLAY_TFT_DC   ??             Display
// 9         PWM3             PWM            Motor3 ???
// 10        ADC_CS           SPI/GPIO       IR-S.-ADC
// 11        MOSI             SPI            IR-S.-ADC, Display, Touch
// 12        MISO             SPI            "
// 13        SCK              SPI            "
// 14        TNS_TX/PIXY_RX   ??             Kamera
// 15        TNS_RX/PIXY_TX   ??             Kamera
// 16        TNS_RX/DEBUG_TX  ??             Debugstecker
// 17        TNS_TX/DEBUG_RX  ??             Debugstecker
// 18        SDA_3V3          I2C            Portexpander(20,21),J7(??),BN0055/Kompass,J6/Ultraschall(??),
// 19        SCL_3V3          I2C            "
// 20        RGB_DATA_3V3     GPIO?          RGB LEDs (10xWS2812B, J8)
// 21        LIGHT_BARRIER    GPIO?          Lichtschranke
// 22        BAT_VOLTAGE      ADC            Batteriespannung
// 23        LED0             GPIO           LED?
//----------------------------------------------------------------------------------------//
/** Motor Belegung ************************************************************************/
// Motor 0 = hinten
// Motor 1 = links
// Motor 2 = rechts
// Motor 3 = Dribbler
/******************************************************************************************/

/** Setup *********************************************************************************/

#if true // Defines
// Pin Definitions
// 0,1,2 Display
#define MAINSWITCH_PORT 3
#define MOTOR0_PORT_PWM 4
#define MOTOR1_PORT_PWM 5
#define MOTOR2_PORT_PWM 6
// 7,8 Display
#define MOTORD_PORT_PWM 9
#define ADC_PORT_CS 10
#define SPI_MOSI_PORT 11
#define SPI_MISO_PORT 12
#define SPI_SCK_PORT 13
#define PXY_RX_PORT 14
#define PXY_TX_PORT 15
// 16,17 Debug TX/RX
// 18,19 I2C
#define LED_PORT_DATA 20
#define LDR_PORT 21
#define BAT_VOLTAGE_PORT 22
#define LED0_PORT 23

// Data
#define LED_LENGTH 10
#endif

#if true // Libraries include
#include <Arduino.h>
#include <Wire.h>              // for port expanders (Motor direction control, switches, Kicker)
#include <Adafruit_NeoPixel.h> // for RGB LEDs
#include <SPI.h>               // for ADC, Display and touch controller
#include <Adafruit_BNO055.h>   // compass sensor
#include <elapsedMillis.h>     // Timing
#include <Pixy2I2C.h>          // Pixy2
#endif

#if true                // Parameter
float sqrt3;
int LEDBrightness = 8;
int MotorFreqency = 200;    // PWM Frequenz für die Motoren
int US_address[4] = {0x70, 0x71, 0x72, 0x73};
int DribblerFreqency = 200; // PWM Frequenz für die Motoren

int LDR_Schwelle = 400; // Grenzwert Lichtscharnke
#endif

#if true                 // global Variables

bool LDR_Ballda = false; // LDR
int LDRval = 0;          // LDR Wert
#endif

#if true // Start Libraries
Adafruit_NeoPixel RGBs = Adafruit_NeoPixel(LED_LENGTH, LED_PORT_DATA, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2I2C pixy;
#endif

#if true             // Timing
elapsedMillis Timer; // Timer
#endif

/** Main **********************************************************************************/

void LDR(void)
{
    LDRval = analogRead(LDR_PORT);        // LDR auslesen
    LDR_Ballda = (LDRval > LDR_Schwelle); // Lichtschranke auswerten

    if (LDRval > LDR_Schwelle + 300) // LDR Schwelle anpassen
    {
        LDR_Schwelle = LDRval - 250;
    }
}

void initialize(void)
{
    //-Serial---------------------------------------------------------------------------------//
    Serial.begin(9600);
    //----------------------------------------------------------------------------------------//

    //-PinModes-------------------------------------------------------------------------------//
    // pinMode(0, OUTPUT); // Display Interrupt
    pinMode(1, OUTPUT);  // Display Touch CS
    pinMode(2, OUTPUT);  // Display Backlight
    pinMode(3, INPUT);   // MainSwitch
    pinMode(4, OUTPUT);  // Motor PWM 0
    pinMode(5, OUTPUT);  // Motor PWM 1
    pinMode(6, OUTPUT);  // Motor PWM 2
    pinMode(7, OUTPUT);  // Display CS
    pinMode(8, OUTPUT);  // Display DC
    pinMode(9, OUTPUT);  // Motor PWM 3 (Dribbler)
    pinMode(10, OUTPUT); // ADC CS
    pinMode(21, INPUT);  // LDR
    pinMode(22, INPUT);  // Battery
    pinMode(23, OUTPUT); /*LED0*/
    //----------------------------------------------------------------------------------------//

    //-RGB LEDs-------------------------------------------------------------------------------//
    // INIT RGBS
    RGBs.begin();
    delay(10);
    RGBs.setBrightness(0);
    RGBs.show();

    // Hochfahr Animation
    RGBs.clear();
    RGBs.setBrightness(LEDBrightness);
    for (int i = 0; i < 10; i++)
    {
        RGBs.setPixelColor(i, 255, 0, 0);
    }
    RGBs.show();
    //----------------------------------------------------------------------------------------//

    //-Expander-------------------------------------------------------------------------------//
    /* pin 18,19: I2C to port expanders (Kicker, switches, motor directions */
    Wire.begin();                 // start I2C operation in master mode
    Wire.beginTransmission(0x20); // address first port expander
    Wire.write(0x03);             // select config register
    Wire.write(0xFC);             // Bits 0,1 output, others input
    Wire.endTransmission();
    Wire.beginTransmission(0x20); // address first port expander
    Wire.write(0x01);             // select output register
    Wire.write(0x00);             // set Kicker and Kicker_SW to zero
    Wire.endTransmission();

    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x01);             // select output register
    Wire.write(0x00);             // set all ports to zero before switching direction
    Wire.endTransmission();
    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x03);             // select config register
    Wire.write(0x00);             // all ports ouput
    Wire.endTransmission();
    //----------------------------------------------------------------------------------------//

    //-BNO------------------------------------------------------------------------------------//
    bno.begin();
    bno.setExtCrystalUse(true);
    //----------------------------------------------------------------------------------------//

    //-SPI------------------------------------------------------------------------------------//
    /* pin 9-13 SPI with ADC chip select*/
    SPI.begin();
    pinMode(ADC_PORT_CS, OUTPUT);    // ADC1 chip select
    digitalWrite(ADC_PORT_CS, HIGH); // prepare default state of ADC1 chip select
    //----------------------------------------------------------------------------------------//

    //-Pixy2----------------------------------------------------------------------------------//
    pixy.init();
    pixy.setLamp(0, 0);   // turn off the lamp
    pixy.setLED(0, 0, 0); // turn off the LED
    pixy.setCameraBrightness(91);
    //----------------------------------------------------------------------------------------//

    //-US-------------------------------------------------------------------------------------//
    int idx;                  // zählt Sensoren durch
    int USnum = 0;            // Anzahl der gefundenen Sensoren
    int sensorAddress = 0x70; // I2C Adresse des aktuellen Sensors
    unsigned char rev;        // software revision of the sensor

    // Programm akkzeptiert Sensoren mit der Addresse 0x70 bis 0x77
    for (idx = 0; idx < 8; idx++) // Alle US Sensoren anfragen
    {
        /* check for sensor presence by reading its software version */
        Wire.beginTransmission(sensorAddress);
        Wire.write(0x00);                   // select version register
        Wire.endTransmission(false);        // switch to read direction
        Wire.requestFrom(sensorAddress, 1); // Start a read access and expect one byte
        rev = Wire.read();                  // get software version
        Wire.endTransmission();
        if (rev != 0xFF) // anything that is not FF is a present sensor
        {
            Serial.print("Sensor found at ");
            Serial.print(sensorAddress, HEX);
            US_address[USnum] = sensorAddress;
            USnum++;
        }
        else
        {
            Serial.print("No sensor at ");
            Serial.print(sensorAddress, HEX);
            Serial.println("h");
        }
        sensorAddress++;

        for (int i = 0; i < (int)roundf((float)idx / 8 - 0.2); i++)
        {
            RGBs.setPixelColor(i, 255, 255, 0); // LED aus
        }
        RGBs.show();
        delay(50); // wait a bit before next sensor
    }
    //----------------------------------------------------------------------------------------//

    //-LDR------------------------------------------------------------------------------------//
    pinMode(24, INPUT);
    //----------------------------------------------------------------------------------------//

    //-Motor PWM------------------------------------------------------------------------------//
    analogWriteFrequency(MOTOR0_PORT_PWM, MotorFreqency);
    analogWriteFrequency(MOTOR1_PORT_PWM, MotorFreqency);
    analogWriteFrequency(MOTOR2_PORT_PWM, MotorFreqency);
    analogWriteFrequency(MOTORD_PORT_PWM, DribblerFreqency);
    //----------------------------------------------------------------------------------------//

    //-Variablen------------------------------------------------------------------------------//
    sqrt3 = sqrtf(3);
    //----------------------------------------------------------------------------------------//

    //-Optik----------------------------------------------------------------------------------//
    for (int i = 0; i < 10; i++)
    {
        RGBs.setPixelColor(i, 0, 255, 0);
        RGBs.show();
        delay(50);
    }

    delay(100);

    RGBs.clear();
    RGBs.show();
    //----------------------------------------------------------------------------------------//

    //-Timing---------------------------------------------------------------------------------//
    Timer = 0;
    //----------------------------------------------------------------------------------------//
}

void setup()
{
    initialize();
}

void loop()
{
    if (Timer >= 20) // alle 20ms
    {
        Timer = 0;
        LDR(); // LDR auswerten
        if (LDR_Ballda)
        {
            Serial.print("LDR: ");
            Serial.print(LDRval);
            Serial.print(" : ");
            Serial.print(LDR_Schwelle);
            Serial.print(" : ");
            Serial.println("Ball da");
        }
        else
        {
            Serial.print("LDR: ");
            Serial.print(LDRval);
            Serial.print(" : ");
            Serial.print(LDR_Schwelle);
            Serial.print(" : ");
            Serial.println("Kein Ball");
        }
    }
}
/******************************************************************************************/