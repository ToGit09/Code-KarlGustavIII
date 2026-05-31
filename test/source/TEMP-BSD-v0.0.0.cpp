/** Port Belegung v3.5 ********************************************************************/
// Pin       Funktion           Verfahren       Angeschlossen
//----------------------------------------------------------------------------------------//
// 0         TNS_RX/DEBUG_TX    UART            DEBUG
// 1         TNS_TX/DEBUG_RX    UART            DEBUG
// 2         PWM0               PWM             Motor0
// 3         PWM1               PWM             Motor1
// 4         PWM2               PWM             Motor2
// 5         PWM3               PWM             Dribbler
// 6         START_SWITCH       GPIO            Schalter
// 7         TNS_RX/BLT_TX      UART            Bluetooth Modul
// 8         TNS_TX/BLT_RX      UART            Bluetooth Modul
// 9         ADC1_CS            GPIO            ADC1
// 10        ADC2_CS            GPIO            ADC2
// 11        MOSI               SPI             ADC1+2, Display
// 12        MISO               SPI             ADC1+2, Display
// 13        SCK                SPI             ADC1+2, Display
// 14        TNS_TX/OPENMV1_RX  UART3           Kamera (OpenMV1)
// 15        TNS_RX/OPENMV1_TX  UART3           Kamera (OpenMV1)
// 16        TNS_TX/OPENMV2_RX  UART3           Kamera (OpenMV2)
// 17        TNS_RX/OPENMV2_TX  UART3           Kamera (OpenMV2)
// 18        SDA_3V3            I2C             Portexpander(0x20,0x21, 0x22(, 0x25)),BN0055
// 19        SCL_3V3            I2C             Portexpander(0x20,0x21, 0x22(, 0x25)),BN0055
// 20        TNS_TX/RCJ_RX      UART3           RCJ Kommunikation
// 21        TNS_RX/RCJ_TX      UART3           RCJ Kommunikation
// 22        DISPLAY_TFT_CS     GPIO            Display
// 23        DISPLAY_RT_CS      GPIO            Display
//
// 24        LIGHT_BARRIER      I2C             Lichtschranke
// 25        DISPLAY_IRQ        GPIO            Display
// 26        KICKER             ?               Kicker Trigger
// 27        KICKER_SW          ?               Kicker 
// 28        DISPLAY_TFT_DC     GPIO            Display
// 29        RGB_DATA_3V3       I2C             RGB LEDs
// 30        SPI_EXT_CS         GPIO            Externer IR Ring
// 31        DISPLAY_SDCARD_CS  GPIO            Display
// 32        SPI_EXT_CS         GPIO            Externer IR Ring
// 33        DISPLAY_SDCARD_DET GPIO            Display

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

/** Setup *********************************************************************************/

#define __V_3_5__
#define __V_3_0__

#ifdef __V_3_5__ // Portbelegung für v3.5 Board

#define __IRBoard__

#define LED_LENGTH 13
#define LED_PORT_DATA 29

#define SPI_MOSI_PORT 11
#define SPI_MISO_PORT 12
#define SPI_SCK_PORT 13

#define DISPLAY_TFT_PORT_CS 22
#define DISPLAY_TFT_PORT_DC 28
#define DISPLAY_RT_PORT_CS 23
// #define DISPLAY_PORT_BACKLIGHT NICHT_BELEGT

#define MOTOR0_PORT_PWM 2
#define MOTOR1_PORT_PWM 3
#define MOTOR2_PORT_PWM 4
#define MOTORD_PORT_PWM 5
#define KICKER 26
#define KICKER_SW 27

#define ADC_PORT_CS 9
#define ADC2_PORT_CS 10
#define EXTSPI1_PORT_CS 30
#define EXTSPI2_PORT_CS 32

#endif

#ifdef __V_3_0__ // Portbelegung für v3.0 Board
#define LED_LENGTH 10
#define LED_PORT_DATA 20
#define LED0_PORT 23

#define SPI_MOSI_PORT 11
#define SPI_MISO_PORT 12
#define SPI_SCK_PORT 13

#define DISPLAY_TFT_PORT_CS 7
#define DISPLAY_TFT_PORT_DC 8
#define DISPLAY_RT_PORT_CS 1
#define DISPLAY_PORT_BACKLIGHT 2

#define MOTOR0_PORT_PWM 4
#define MOTOR1_PORT_PWM 5
#define MOTOR2_PORT_PWM 6
#define MOTORD_PORT_PWM 9

#define ADC_PORT_CS 10

#endif

#if true // Libraries include
#include <Arduino.h>
#include <Wire.h>              // for port expanders (Motor direction control, switches, Kicker)
#include <Adafruit_NeoPixel.h> // for RGB LEDs
#include <SPI.h>               // for ADC, Display and touch controller
#include <Adafruit_BNO055.h>   // compass sensor
#include <elapsedMillis.h>     // Timing

#include <Adafruit_GFX.h>     // display: Core graphics library
#include <Adafruit_ILI9341.h> // display: Hardware-specific library
#include <ILI9341_t3n.h>      // display: Hardware-specific library
#include <TouchScreen.h>      // display
#endif

#if true // Parameter
// Internal
String version = "V0.0.0"; // Version des Boards

// Software
int IR_Range = 8; // In wie viele Bereiche der IR Ring geteiolt wird // 8, 12, 20

// PID
float PID = 0;
float PID_P = 0;
float PID_I = 0;
float PID_D = 0;
float PID_P_Multiplier = 4;   // 4
float PID_I_Multiplier = 1.4; // 1.4
float PID_D_Multiplier = 15;  // 15
float PID_Stand_Multiplier = 70;
float PID_Multiplikator = 0.06; // 0.06
float PIDpa_IlimitMax = 105;
float PIDpa_IlimitMin = -105;
float PIDpa_Imultiplier = 0.45;
float PID_I_threshhold = 12;

// Hardware
int MotorFreqency = 200;    // PWM Frequenz für die Motoren
int DribblerFreqency = 200; // PWM Frequenz für die Motoren
int US_address[4] = {0x70, 0x71, 0x72, 0x73};
int US_value[4] = {0, 0, 0, 0};
int IR_Adressen[8];                                                // IR Adressen Katalog
int IR_min[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // IR Minimalwerte
int IR_max[16] = {
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
    1023,
}; // IR Maximalwerte
int LDR_Schwelle = 400; // Grenzwert Lichtscharnke
#endif

#if true // global Variables

float sqrt3;

// Sensor und Laufzeit Variablen
float OrbitDirection = 0; // Richtung des Tores
uint8_t calibrationGyro;
uint8_t calibrationMag;
float IR_Direction = 0; // Richtung des IR Sensors
float IR_Heading = 0;   // Ungefähres Areal IR Sensors
float US_Vorne = 0;     // US
float US_Hinten = 0;
float US_Links = 0;
float US_Rechts = 0;
float US_offsetx = 0;
float US_offsety = 0;
bool LDR_Ballda = false; // LDR
bool SWI1 = false;       // Schalter
bool SWI2 = false;
bool SWI3 = false;
bool SWI4 = false;
bool SWI5 = false;
bool SWI6 = false;
bool MainSwitch = false;
bool BTN1 = false; // Taster
bool BTN2 = false;
bool BTN3 = false;
bool BTN4 = false;
bool BTN5 = false;
float rt_OrbitDirection = 0; // Richtung des Tores
uint8_t rt_calibrationGyro;
uint8_t rt_calibrationMag;
float rt_IR_Direction = 0; // Richtung des IR Sensors
float rt_IR_Heading = 0;   // Ungefähres Areal IR Sensors
float rt_US_Vorne = 0;     // US
float rt_US_Hinten = 0;
float rt_US_Links = 0;
float rt_US_Rechts = 0;
float rt_US_offsetx = 0;
float rt_US_offsety = 0;
bool rt_LDR_Ballda = false; // LDR
bool rt_SWI1 = false;       // Schalter
bool rt_SWI2 = false;
bool rt_SWI3 = false;
bool rt_SWI4 = false;
bool rt_SWI5 = false;
bool rt_SWI6 = false;
bool rt_MainSwitch = false;
bool rt_BTN1 = false; // Taster
bool rt_BTN2 = false;
bool rt_BTN3 = false;
bool rt_BTN4 = false;
bool rt_BTN5 = false;

// Taktische Variablen
float angleOfAttack = 0; // Anstellwinkel

// System Variablen
float PIDKorrektur_memory;
int IR_values[16] = {0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1}; // IR Werte (manchmal nur 8)
bool switchValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};                              // Schalter
float KompassCliValue = 0;                                                    // Kompass
int DisplayRT = 0;
int lastLoopTime = 0;
int lastUpdateTime = 0;
#endif

#if true // Start Libraries
Adafruit_NeoPixel RGBs = Adafruit_NeoPixel(LED_LENGTH, LED_PORT_DATA, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
ILI9341_t3n Display = ILI9341_t3n(DISPLAY_TFT_PORT_CS, DISPLAY_TFT_PORT_DC, -1, SPI_MOSI_PORT, SPI_SCK_PORT, SPI_MISO_PORT);
#endif

/** Lowlevel ******************************************************************************/

// Rechnet die Rohen Werte in nutzbare werte um
class CodeCalculate
{
public:
    // verarbeitet die ausgelesenen IR Sensoren
    void IR_Calc(void)
    {
        float Ballx = 0;
        float Bally = 0;

        if (IR_values[8] == -1) // berechnung mit 8 Sensoren
        {
            for (int i = 0; i < 16; i++)
            {
                Ballx += cosf((i * 22.5) * PI / 180) * IR_values[i];
                Bally += sinf((i * 22.5) * PI / 180) * IR_values[i];
            }
        }
        else // Berechnung mit 16 Sensoren
        {
            for (int i = 0; i < 16; i++)
            {
                IR_values[i] = map(IR_values[i], IR_min[i], IR_max[i], 0, 100);
            }

            // IR Berechnung
            for (int i = 0; i < 16; i++)
            {
                Ballx += cosf((i * 22.5) * PI / 180) * IR_values[i];
                Bally += sinf((i * 22.5) * PI / 180) * IR_values[i];
            }
        }

        IR_Direction = atan2f(Bally, Ballx) * 180 / PI; // evtl *55

        IR_Heading = int(round((IR_Direction + 180) / (360 / IR_Range)));

        if (IR_Heading == IR_Range) // begrenzen der Ergebnisse auf 0...IR_Range-1
            IR_Heading = 0;

        IR_Heading -= IR_Range / 2; // Rotation so dass 0° in der Mitte ist
    }

    // verarbeitet die ausgelesenen US Sensoren
    void US_Calc(void)
    {
        US_Vorne = US_value[3];
        US_Rechts = US_value[1];
        US_Hinten = US_value[2];
        US_Links = US_value[0];

        US_offsetx = (US_Rechts - US_Links) / 2;
        US_offsety = (US_Vorne - US_Hinten) / 2;
    }
};

// Liest die Sensoren aus
class CodeRead : private CodeCalculate
{
public:
    // liest IR Sensoren aus und übergibt an IR_Calc
    void IR(void)
    {
        uint16_t Cc;
        SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE3));
#ifdef __IRBoard__
        digitalWrite(EXTSPI1_PORT_CS, HIGH); // EXTSPI used for IR Ring
#else
        digitalWrite(ADC_PORT_CS, LOW); // Builtin ADC used for IR Ring
#endif

        for (int i = 0; i < 8; i++)
        {
            Cc = i + 1; // select channel for next conversion; very first one defaults to zero
            Cc %= 8;    // limit to 0...7 (not really necessary)
            Cc <<= 11;  // the ADC expects channel selection in bits 11...13
            // SPI.transfer(0x01 << i); // COPILOT
            IR_Adressen[i] = SPI.transfer16(0x00);
            IR_Adressen[i] >>= 2;
        }
#ifdef __IRBoard__
        digitalWrite(EXTSPI1_PORT_CS, HIGH); // EXTSPI used for IR Ring
#else
        digitalWrite(ADC_PORT_CS, HIGH); // Builtin ADC used for IR Ring
#endif
        SPI.endTransaction();

        // 2. ADC Nutzen falls vorhanden
#ifdef __IRBoard__
        SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE3));
        digitalWrite(EXTSPI2_PORT_CS, LOW); // EXTSPI used for IR Ring
        for (int i = 8; i < 16; i++)
        {
            Cc = i + 1; // select channel for next conversion; very first one defaults to zero
            Cc %= 8;    // limit to 0...7 (not really necessary)
            Cc <<= 11;  // the ADC expects channel selection in bits 11...13
            // SPI.transfer(0x01 << i); // COPILOT
            IR_Adressen[i] = SPI.transfer16(0x00);
            IR_Adressen[i] >>= 2;
        }

        digitalWrite(EXTSPI2_PORT_CS, HIGH); // EXTSPI used for IR Ring
        SPI.endTransaction();

#else
        for (int i = 8; i < 16; i++)
        {
            IR_Adressen[i] = -1;
        }
#endif

        // IR Berechnung
        IR_Calc();
    }

    // liest die UltraschallSensoren aus
    void US(void)
    {
        uint16_t distance;
        { // get
            Wire.beginTransmission(US_address[0]);
            Wire.write(0x02);                   // select first echo high-byte register
            Wire.endTransmission(false);        // switch to read direction
            Wire.requestFrom(US_address[0], 2); // Start a read access and expect 17 range words
            distance = Wire.read() << 8;        // get high byte
            distance += Wire.read();            // get low byte
            Wire.endTransmission();
            US_value[0] = int(distance);

            Wire.beginTransmission(US_address[1]);
            Wire.write(0x02);                   // select first echo high-byte register
            Wire.endTransmission(false);        // switch to read direction
            Wire.requestFrom(US_address[1], 2); // Start a read access and expect 17 range words
            distance = Wire.read() << 8;        // get high byte
            distance += Wire.read();            // get low byte
            Wire.endTransmission();
            US_value[1] = int(distance);

            Wire.beginTransmission(US_address[2]);
            Wire.write(0x02);                   // select first echo high-byte register
            Wire.endTransmission(false);        // switch to read direction
            Wire.requestFrom(US_address[2], 2); // Start a read access and expect 17 range words
            distance = Wire.read() << 8;        // get high byte
            distance += Wire.read();            // get low byte
            Wire.endTransmission();
            US_value[2] = int(distance);

            Wire.beginTransmission(US_address[3]);
            Wire.write(0x02);                   // select first echo high-byte register
            Wire.endTransmission(false);        // switch to read direction
            Wire.requestFrom(US_address[3], 2); // Start a read access and expect 17 range words
            distance = Wire.read() << 8;        // get high byte
            distance += Wire.read();            // get low byte
            Wire.endTransmission();
            US_value[3] = int(distance);
        }

        { // Read
            Wire.beginTransmission(US_address[0]);
            Wire.write(0x00); // select version/command register
            Wire.write(0x51); // send command: read in cm
            Wire.endTransmission();
            Wire.beginTransmission(US_address[1]);
            Wire.write(0x00); // select version/command register
            Wire.write(0x51); // send command: read in cm
            Wire.endTransmission();
            Wire.beginTransmission(US_address[2]);
            Wire.write(0x00); // select version/command register
            Wire.write(0x51); // send command: read in cm
            Wire.endTransmission();
            Wire.beginTransmission(US_address[3]);
            Wire.write(0x00); // select version/command register
            Wire.write(0x51); // send command: read in cm
            Wire.endTransmission();
        }

        US_Calc();
    }

    // Liest die Kompass Werte aus
    void Compass(void)
    {
        uint8_t dump;
        uint8_t dump2;
        calibrationGyro = 0;
        calibrationMag = 0;
        bno.getCalibration(&dump, &calibrationGyro, &dump2, &calibrationMag);

        sensors_event_t event;
        bno.getEvent(&event);
        float KompassOutput = roundf(event.orientation.x); // 0 - 359

        OrbitDirection = KompassOutput - 180;

        OrbitDirection -= KompassCliValue; // Richtung des Tores
    }

    // liest die Lichtschranke aus
    void LDR(void)
    {
        int LDRval = digitalRead(24);
        LDR_Ballda = (LDRval > LDR_Schwelle);
    }

    // liest die schalter und Taster aus
    void Switches(void)
    {
        Wire.beginTransmission(0x20);       // address first port expander
        Wire.write(0x00);                   // select input register
        Wire.endTransmission(false);        // send repeated start instead of stop
        Wire.requestFrom(0x20, 1);          // Start a read access and expect one byte
        int switchValuesRead = Wire.read(); // read inputs
        Wire.endTransmission();             // now send a stop

        for (int i = 0; i < 8; i++)
        {
            byte Switches_new = switchValuesRead;
            byte Swi_byte = Switches_new >> i;
            int Swi_byte_new = Swi_byte;
            if (i > 3) // Switches 1-4
            {
                if (Swi_byte_new % 2 == 0)
                    switchValues[i] = false;
                else
                    switchValues[i] = true;
            }
            else // Buttons
            {
                if (Swi_byte_new % 2 == 0)
                    switchValues[i] = true;
                else
                    switchValues[i] = false;
            }
        }
        MainSwitch = digitalRead(3);
        BTN1 = switchValues[0];
        BTN2 = switchValues[1];
        BTN3 = switchValues[2];
        BTN4 = switchValues[3];
        BTN5 = false;
        SWI1 = switchValues[4];
        SWI2 = switchValues[5];
        SWI3 = switchValues[6];
        SWI4 = switchValues[7];
        SWI5 = false;
        SWI6 = false;

#ifdef __IRBoard__

        Wire.beginTransmission(0x25);       // address external port expander
        Wire.write(0x00);                   // select input register
        Wire.endTransmission(false);        // send repeated start instead of stop
        Wire.requestFrom(0x25, 1);          // Start a read access and expect one byte
        int switchValuesRead = Wire.read(); // read inputs
        Wire.endTransmission();             // now send a stop
        for (int i = 1; i < 4; i++)
        {
            byte Switches_new = switchValuesRead;
            byte Swi_byte = Switches_new >> i;
            int Swi_byte_new = Swi_byte;
            if (i == 1) // Switches 1-4
                SWI5 = (Swi_byte_new % 2 == 0) ? false : true;
            else if (i == 2)
                SWI6 = (Swi_byte_new % 2 == 0) ? false : true;
            else // Buttons
                BTN5 = (Swi_byte_new % 2 == 0) ? true : false;
        }
#endif
    }
};

// Handelt das Display
class CodeDisplay
{
    // 320 x 240
    // Rand 2
    // Kreis: Radius: 60, x: 250, y: 80
    // Rechteck: x: 4, y: 10, w: 126, h: 100
    // Rechteck: x: 4, y: 170, w: 232, h: 110
    bool CleanScreen = true;

public:
    void DrawStartScreen(void)
    {
        Display.fillScreen(ILI9341_BLACK);
        Display.drawRect(2, 2, 316, 236, ILI9341_RED); // Rechteck (Display)

        Display.setCursor(4, 4);             // Textposition
        Display.setTextColor(ILI9341_WHITE); // Textcolor
        Display.setTextSize(10);             // Textsize
        Serial.println("Bodensee Devils");   // Text
    }

    void DrawInfoScreen(void)
    {
        Display.fillScreen(ILI9341_BLACK);

        Display.drawRect(2, 2, 316, 236, ILI9341_RED);     // Rechteck (Display)
        Display.drawLine(0, 120, 320, 120, ILI9341_GREEN); // horizontale Linie (Display)
        Display.drawLine(160, 0, 160, 240, ILI9341_GREEN); // vertikale Linie (Display)

        Display.setCursor(4, 4);                         // Textposition
        Display.setTextColor(ILI9341_WHITE);             // Textcolor
        Display.setTextSize(2);                          // Textsize
        Display.println("Bodensee Devils - " + version); // Text

        Display.drawCircle(250, 80, 60, ILI9341_BLUE);                    // Kreis (IR/Kompass)
        Display.drawPartialCircle(250, 80, 180, 360, 60, ILI9341_YELLOW); // Kreis (IR area)
        Display.drawPartialCircle(250, 80, 19, 21, 60, ILI9341_RED);      // Kreis (IR Degree)
        Display.drawPartialCircle(250, 80, 359, 1, 60, ILI9341_GREEN);    // Kreis (Goal Degree)
        Display.drawCircle(250, 50, 4, ILI9341_GREEN);                    // Kreis (LDR)
        Display.setTextSize(1);                                           // Textsize
        Display.setCursor(240, 70);                                       // Textposition
        Display.setTextColor(ILI9341_VIOLET);                             // Textcolor
        Display.println("IR");                                            // Text
        Display.println("Km");                                            // Text

        Display.drawRect(4, 10, 126, 100, ILI9341_YELLOW); // Rechteck (US)
        Display.drawLine(4, 60, 130, 60, ILI9341_YELLOW);  // vertikale Linie (US)
        Display.drawLine(67, 10, 67, 110, ILI9341_YELLOW); // horizontale Linie (US)
        Display.drawCircle(67, 60, 4, ILI9341_DARKBLUE);   // Kreis (US)

        Display.drawRect(4, 170, 232, 110, ILI9341_MAGENTA); // Rechteck (Switches)
        Display.fillRect(4, 172, 33, 50, ILI9341_GREEN);     // Rechteck (Switches 1)
        Display.fillRect(37, 172, 33, 50, ILI9341_GREEN);    // Rechteck (Switches 2)
        Display.fillRect(70, 172, 33, 50, ILI9341_GREEN);    // Rechteck (Switches 3)
        Display.fillRect(103, 172, 33, 50, ILI9341_GREEN);   // Rechteck (Switches 4)
        Display.fillRect(136, 172, 34, 50, ILI9341_BLACK);   // Rechteck (PLATZHALTER)
        Display.fillRect(170, 172, 33, 50, ILI9341_GREEN);   // Rechteck (Switches 5)
        Display.fillRect(203, 172, 33, 50, ILI9341_YELLOW);  // Rechteck (MAIN_SW)

        Display.fillRect(4, 228, 33, 50, ILI9341_GREEN);   // Rechteck (Switches 1)
        Display.fillRect(37, 228, 33, 50, ILI9341_GREEN);  // Rechteck (Switches 2)
        Display.fillRect(70, 228, 33, 50, ILI9341_GREEN);  // Rechteck (Switches 3)
        Display.fillRect(103, 228, 33, 50, ILI9341_GREEN); // Rechteck (Switches 4)
        Display.fillRect(136, 228, 34, 50, ILI9341_BLACK); // Rechteck (PLATZHALTER)
        Display.fillRect(170, 228, 33, 50, ILI9341_GREEN); // Rechteck (Switches 5)
        Display.fillRect(203, 228, 33, 50, ILI9341_BLACK); // Rechteck (ON/OFF_SW)
    }

    void DrawClearScreen(void)
    {
        if (CleanScreen)
            return;

        Display.fillScreen(ILI9341_BLACK);
        CleanScreen = true;
    }
};

// Optische Outputs
class CodeOptical : public CodeDisplay
{
public:
    // Schaltet einzelne LEDs
    // @param i: LED Index
    // @param r: Rotwert (0-255)
    // @param g: Grünwert (0-255)
    // @param b: Blauwert (0-255)
    void singleLED(int i, int r, int g, int b)
    {
    }

    // Schaltet alle LEDS nach den Aktuellen Werten ein
    void LED(void)
    {
        if (SWI1)
        {
        }
        else
            RGBs.clear();

        RGBs.show();
    }

    // Zeigt alles relevante auf dem Display an
    void Display(void)
    {
        DisplayTime = 0;
        if (SWI1)
            DrawInfoScreen();
        else
            DrawClearScreen();
        DisplayRT = DisplayTime;
    }

    // Gibt Seriel Werte aus
    void doSerial(void)
    {
        Serial.println("Bodensee Devils - TEMP - " + version);
        Serial.print("TimeStamp: ");
        Serial.print(totalTime);
        Serial.print("; LoopTime: ");
        Serial.print(lastLoopTime);
        Serial.print("; UpdateTime: ");
        Serial.print(lastUpdateTime);
        Serial.print("; FPS: ");
        Serial.println(1000 / DisplayTime);
        Serial.print("TBD ");
        Serial.println("- Coming soon ");
    }
};

// Alles was sich Bewegt
class CodeAction
{
public:
    // PID Regler berechnen
    // ausgenommen von rT Regel da unabhängig vom Mainloop
    void calculatePID(void)
    {
        float PIDKorrektur = OrbitDirection - angleOfAttack;

        PID_I = PID_I + PIDKorrektur;

        if (PIDKorrektur > -(PID_I_threshhold) && PIDKorrektur < PID_I_threshhold) // wenn I gegen null geht auf 0 setzen
            PID_I = 0;

        if (PID_I > PIDpa_IlimitMax) // I nach oben begrenzen
            PID_I = PIDpa_IlimitMax;
        else if (PID_I < PIDpa_IlimitMin) // I nach unten begrenzen
            PID_I = PIDpa_IlimitMin;

        PID_P = PIDKorrektur * PID_P_Multiplier; // Berechnung P wert

        PID_I = PID_I * PID_I_Multiplier; // Berechnung I wert

        PID_D = (PIDKorrektur - PIDKorrektur_memory) * PID_D_Multiplier; // Berechnung D wert

        PID = PID_P + PID_I + PID_D;        // PID Wert berechnen
        PIDKorrektur_memory = PIDKorrektur; // PIDKorrektur speichern für D wert
    }

    // Setzt einen Motor
    // @param i: Motor Index (0 = hinten, 1 = links, 2 = rechts, 3 = Dribbler)
    // @param dir: Richtung (0: Freilauf, 1: Rechts, 2: Links, 3: Bremsen)
    // @param speed: Geschwindigkeit (0-100)
    void setMotor(int i, int dir, int speed)
    {
        uint8_t ports;
        // get current port state (slow but simple)
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.endTransmission(false);  // send restart instead of stop
        Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
        ports = Wire.read();          // get current port state
        // Wire.endTransmission(); //nicht benötigt

        switch (i)
        {
        case 0:
            analogWrite(4, (uint16_t)speed * 256 / 100);
            ports = (ports & 0xFC) | ((dir & 0x03) << 0);
            break;
        case 1:
            analogWrite(5, (uint16_t)speed * 256 / 100);
            ports = (ports & 0xF3) | ((dir & 0x03) << 2);
            break;
        case 2:
            analogWrite(6, (uint16_t)speed * 256 / 100);
            ports = (ports & 0xCF) | ((dir & 0x03) << 4);
            break;
        case 3:
            analogWrite(9, (uint16_t)speed * 256 / 100);
            ports = (ports & 0x3F) | ((dir & 0x03) << 6);
            break;
        }

        // motorsteuerung senden
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(ports);            // update output ports
        Wire.endTransmission();
    }

    // Movement Funktion für Omnidirektionale Bewegung
    // @param angle: Winkel in Grad (-180-180)
    // @param speed: Geschwindigkeit (0-100)
    // @param aoa: Anstellwinkel (-180-180)
    void move(float angle, float speed, float aoa)
    {
        // Winkel in x und y Bewegung umrechnen
        float angleRad = angle * PI / 180; // Winkel in Radiant umrechnen
        float x = cosf(angleRad) * speed;  // x Bewegung
        float y = sinf(angleRad) * speed;  // y Bewegung

        // PID Regler berechnen
        angleOfAttack = aoa; // Anstellwinkel
        float turn = PID * PID_Multiplikator;

        // Movement berechnen
        int Motor0 = (int)(x + turn);                            // Motor 0
        int Motor1 = (int)((-0.5 * x) - (sqrt3 / 2 + y) + turn); // Motor 1
        int Motor2 = (int)((-0.5 * x) + (sqrt3 / 2 + y) + turn); // Motor 2

        // Standbewegung
        if (speed == 0)
        {
            Motor0 = (turn * PID_Stand_Multiplier);
            Motor1 = (turn * PID_Stand_Multiplier);
            Motor2 = (turn * PID_Stand_Multiplier);
        }

        // Richtungen bercehnen
        int dir0 = Motor0 > 0 ? 1 : 2;
        int dir1 = Motor1 > 0 ? 1 : 2;
        int dir2 = Motor2 > 0 ? 1 : 2;

        // Mapping
        float mapping = 1;

        if (Motor0 > Motor1 && Motor0 > Motor2)
            mapping = speed / abs(Motor0);
        else if (Motor1 > Motor0 && Motor1 > Motor2)
            mapping = speed / abs(Motor1);
        else
            mapping = speed / abs(Motor2);

        Motor0 = Motor0 * mapping;
        Motor1 = Motor1 * mapping;
        Motor2 = Motor2 * mapping;

        setMotor(0, dir0, abs(Motor0));
        setMotor(1, dir1, abs(Motor1));
        setMotor(2, dir2, abs(Motor2));
    }

    // Alle Motoren stoppen
    void brake(void)
    {
        setMotor(0, 0, 0);
        setMotor(1, 0, 0);
        setMotor(2, 0, 0);
    }

    // Den Dribbler bedienen
    // @param an: Dribbler an/aus
    // @param dir: Dribbler Richtung
    void dribbler(bool an, bool dir)
    {
        setMotor(3, dir ? 1 : 2, an ? 100 : 0);
    }

    // lädt den kicker auf
    void kicker_load(void)
    {
        if (KickerTimer > 200)
        {
            digitalWrite(KICKER, LOW);
        }
    }

    // Schießt den Ball
    void kick(void)
    {
        if (KickerTimer > 500)
        {
            digitalWriteFast(KICKER, HIGH);
            KickerTimer = 0;
        }
    }
};

/** Highlevel *****************************************************************************/

// Code für ie Taktiken
class CodeTactics
{
public:
    CodeAction action;

    // Ins eigene Tor fahren wenn der Ball nicht gesehen wird
    void homing(void)
    {
    }

    // Den Ball anfahren
    void ballanfahrt(void)
    {
    }

    // Mit dem Ball ins Tor fahren / Schiessen
    void toranfahrt(void)
    {
    }
};

// Der Main Code für den Roboter
class CodeRobot
{
public:
    CodeRead Read;
    CodeTactics Tactics;
    CodeOptical Optical;

    // Initialisierung des Roboters
    void initialize(void)
    {
        //-Expander-------------------------------------------------------------------------------//
#ifdef __V_3_5__

        // 0x20 BTN and SWI
        Wire.begin();                 // join i2c bus
        Wire.beginTransmission(0x20); // address first port expander
        Wire.write(0x03);             // select config register
        Wire.write(0xFF);             // all input
        Wire.endTransmission();       // send stop

        // 0x21 Motor (0, 1, 2, D)
        Wire.begin();                 // join i2c bus
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(0x00);             // set all ports to zero before switching direction
        Wire.endTransmission();       // send stop
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x03);             // select config register
        Wire.write(0x00);             // all ports ouput
        Wire.endTransmission();

        // 0x22 Comunication Board and LED0-3
        Wire.begin();                 // join i2c bus
        Wire.beginTransmission(0x22); // address third port expander
        Wire.write(0x03);             // select config register
        Wire.write(0xF0);             // 0, 1, 2, 3 input, 4, 5, 6, 7 output
        Wire.endTransmission();       // send stop
        Wire.beginTransmission(0x22); // address third port expander
        Wire.write(0x01);             // select output register
        Wire.write(0x04);             // set all ports to zero except LED1 Port
        Wire.endTransmission();       // send stop
#else
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
        Wire.begin();                 // start I2C operation in master mode
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(0x00);             // set all ports to zero before switching direction
        Wire.endTransmission();
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x03);             // select config register
        Wire.write(0x00);             // all ports ouput
        Wire.endTransmission();
#endif

        //-BNO------------------------------------------------------------------------------------//
        bno.begin();
        bno.setExtCrystalUse(true);
        //----------------------------------------------------------------------------------------//

        //-SPI------------------------------------------------------------------------------------//
        /* pin 9-13 SPI with ADC chip select*/
        SPI.begin();
        pinMode(ADC_PORT_CS, OUTPUT);    // ADC1 chip select
        digitalWrite(ADC_PORT_CS, HIGH); // prepare default state of ADC1 chip select
#ifdef __V_3_5__
        pinMode(ADC2_PORT_CS, OUTPUT);    // ADC2 chip select
        digitalWrite(ADC2_PORT_CS, HIGH); // prepare default state of ADC2 chip select
        pinMode(EXTSPI1_PORT_CS, OUTPUT); // External SPI1 chip select
        digitalWrite(ADC2_PORT_CS, HIGH); // prepare default state of ADC2 chip select
        pinMode(ADC2_PORT_CS, OUTPUT);    // ExternalSPI2 chip select
        digitalWrite(ADC2_PORT_CS, HIGH); // prepare default state of ADC2 chip select
#endif
        pinMode(DISPLAY_RT_PORT_CS, OUTPUT);     // resistive touch (RT) chip select
        digitalWrite(DISPLAY_RT_PORT_CS, HIGH);  // prepare default state resistive touch (RT) chip select
        pinMode(DISPLAY_TFT_PORT_CS, OUTPUT);    // TFT chip select
        digitalWrite(DISPLAY_TFT_PORT_CS, HIGH); // prepare default state of TFT chip select
        //----------------------------------------------------------------------------------------//

        //-US-------------------------------------------------------------------------------------//
        // Alle US Sensoren anfragen
        Wire.beginTransmission(US_address[0]);
        Wire.write(0x00); // select version/command register
        Wire.write(0x51); // send command: read in cm
        Wire.endTransmission();
        Wire.beginTransmission(US_address[1]);
        Wire.write(0x00); // select version/command register
        Wire.write(0x51); // send command: read in cm
        Wire.endTransmission();
        Wire.beginTransmission(US_address[2]);
        Wire.write(0x00); // select version/command register
        Wire.write(0x51); // send command: read in cm
        Wire.endTransmission();
        Wire.beginTransmission(US_address[3]);
        Wire.write(0x00); // select version/command register
        Wire.write(0x51); // send command: read in cm
        Wire.endTransmission();
        //----------------------------------------------------------------------------------------//

        //-LDR------------------------------------------------------------------------------------//
        pinMode(24, INPUT);
        //----------------------------------------------------------------------------------------//

        //-Display--------------------------------------------------------------------------------//
#ifdef __V_3_0__
        analogWriteFrequency(DISPLAY_PORT_BACKLIGHT, 100);
        analogWrite(DISPLAY_PORT_BACKLIGHT, 255);
#endif
        Display.begin();
        Display.setRotation(3);            // same orientation as PCB silkscreen,
        Display.fillScreen(ILI9341_BLACK); // clear display
        Optical.DrawStartScreen();
        //----------------------------------------------------------------------------------------//

        //-RGB LEDs-------------------------------------------------------------------------------//
        // INIT RGBS
        RGBs.begin();
        delay(10);
        RGBs.setBrightness(0);
        RGBs.show();
        //----------------------------------------------------------------------------------------//

        //-Motor PWM------------------------------------------------------------------------------//
        analogWriteFrequency(MOTOR0_PORT_PWM, MotorFreqency);
        analogWriteFrequency(MOTOR1_PORT_PWM, MotorFreqency);
        analogWriteFrequency(MOTOR2_PORT_PWM, MotorFreqency);
        analogWriteFrequency(MOTORD_PORT_PWM, DribblerFreqency);
        //----------------------------------------------------------------------------------------//

        //-Variablen------------------------------------------------------------------------------//
        sqrt3 = sqrtf(3);
        IR_Adressen[0] = 2048;
        IR_Adressen[1] = 4096;
        IR_Adressen[2] = 6144;
        IR_Adressen[3] = 8192;
        IR_Adressen[4] = 10240;
        IR_Adressen[5] = 12288;
        IR_Adressen[6] = 14336;
        IR_Adressen[7] = 16384;
        //----------------------------------------------------------------------------------------//

        delay(100);
        Optical.DrawClearScreen();
    }

    // Liest alle Sensoren aus und zeigt die Werte auf dem Display an
    void update(void)
    {
        UpdateTime = 0;
        Read.IR();
        Read.Compass();
        Read.LDR();
        Tactics.action.calculatePID();
        Tactics.action.kicker_load();

        if (ReadTimer % 50 >= 45)
        {
            Read.Switches();
        }

        if (ReadTimer >= 110)
        {
            Read.US();
            Optical.Display();
            ReadTimer = 0;
        }
    
        lastUpdateTime = UpdateTime;
    }

    // Handelt das Timing der Game Funktion und speichert alle sensor Werte
    void system(void)
    {

        if (LoopTimer >= 10) // Alle 10 ms Ausführen
        {
            LoopTime = 0;
            // System loop

            { // Alle gloablen Variablen speichern
                noInterrupts();
                rt_OrbitDirection = OrbitDirection;   // Richtung des Tores
                rt_calibrationGyro = calibrationGyro; // Kalibrierungswerte
                rt_calibrationMag = calibrationMag;
                rt_IR_Direction = IR_Direction; // Richtung des IR Sensors
                rt_IR_Heading = IR_Heading;     // Ungefähres Areal IR Sensors
                rt_US_Vorne = US_Vorne;         // US
                rt_US_Hinten = US_Hinten;
                rt_US_Links = US_Links;
                rt_US_Rechts = US_Rechts;
                rt_US_offsetx = US_offsetx;
                rt_US_offsety = US_offsety;
                rt_LDR_Ballda = LDR_Ballda; // LDR
                rt_SWI1 = SWI1;             // Schalter
                rt_SWI2 = SWI2;
                rt_SWI3 = SWI3;
                rt_SWI4 = SWI4;
                rt_SWI5 = SWI5;
                rt_SWI6 = SWI6;
                rt_MainSwitch = MainSwitch;
                rt_BTN1 = BTN1; // Taster
                rt_BTN2 = BTN2;
                rt_BTN3 = BTN3;
                rt_BTN4 = BTN4;
                rt_BTN5 = BTN5;
                interrupts();
            }

            if (rt_MainSwitch)
            {
                if (rt_SWI5)
                    ;
                else
                    game();
            }
            else
            {
                Tactics.action.brake();
                Optical.doSerial();
            }

            lastLoopTime = LoopTime;
            LoopTimer = 0;
        }
    }

    // Verarbeitet die wichtigsten Sensor Werte und entscheidet über die Taktiken
    void game(void)
    {
        // Game loop
    }
};

/** Main **********************************************************************************/
CodeRobot TEMP;
IntervalTimer InteruptTimer; // 5ms, Robot::update
elapsedMillis ReadTimer;     // Read Timer
elapsedMillis KickerTimer;   // Kicker Timer
elapsedMillis totalTime;     // Insgesamte Zeit die verstrichen ist
elapsedMillis LoopTime;      // Zeit die ein durchlauf braucht
elapsedMillis UpdateTime;    // Zeit die ein Update durchlauf braucht
elapsedMillis LoopTimer;     // immer hochzählen bis zehn oder so und dann starten
elapsedMillis LOPTimer;      // Lack of progress Timer
elapsedMillis DisplayTime;   // Zeit die ein Display durchlauf braucht

void InteruptTimerFunc(void)
{
    TEMP.update();
}

void setup()
{
    TEMP.initialize();
    InteruptTimer.begin(InteruptTimerFunc, 5 * 1000);
}

void loop()
{
    TEMP.system();
}