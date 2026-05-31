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
#include <Pixy2I2C.h>         // Pixy2
#endif

#if true // Parameter
// Internal
String version = "V0.0.0"; // Version des Boards

// Software
long unsigned int LoopTiming = 10; // Zeit in ms wann der Loop durchlaufen wird
int IR_Range = 8;                  // In wie viele Bereiche der IR Ring geteiolt wird // 8, 12, 20
float IR_CircleR = 37;
bool IR_Mode = false; // true = Tropfen, false = Kreis
float USdivider = 0.8;
float USminSpeed = 40.0;
float USminSpeedH = 30.0;
float USdriveAngle = 1.2;
long unsigned int LOP_TimerLimit = 8000;
long unsigned int LOP_BackTime = 500;
long unsigned int LOP_FrontTime = 1100;

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
int LEDBrightness = 8;
int MotorFreqency = 200;    // PWM Frequenz für die Motoren
int DribblerFreqency = 200; // PWM Frequenz für die Motoren
int US_address[4] = {0x70, 0x71, 0x72, 0x73};
int US_value[4] = {0, 0, 0, 0};
int IR_Schwelle = 5;                                                       // Ab wann der ball als gesehen anerkannt wird(in % für IR_Unreliable)
int IR_Adressen[8] = {2048, 4096, 6144, 8192, 10240, 12288, 14336, 16384}; // IR Adressen Katalog
float Gewicht[8] = {0, 45, 90, 135, 180, -135, -90, -45};                  // IR anordnung in Grad (gemessen zu Fahrtrichtung) Mode 2
int IR_min[8] = {20, 20, 20, 670, 20, 16, 24, 16};                         // IR Minimalwerte
int IR_max[8] = {
    3200,
    2500,
    3350,
    3500,
    3400,
    1500,
    3400,
    3600};                                                                                // IR Maximalwerte
int LDR_Schwelle = 400;                                                                   // Grenzwert Lichtscharnke
int MotorPorts[4] = {MOTOR0_PORT_PWM, MOTOR1_PORT_PWM, MOTOR2_PORT_PWM, MOTORD_PORT_PWM}; // Motor Ports

// Pixy
int PixyDistSchwelle = 50; // Pixy Distanzschwelle
int PixyYDumpValue = 250;  // Pixy Y Wert ab dem die Blöcke ignoriert werden
int PixyMaxHeight = 200;   // Pixy Maximalhöhe
int PixyMinHeight = 0;     // Pixy Minimalhöhe
int PixyCamWidth = 320;    // Pixy Cam Width
float PixyDivider = 5;     // Pixy Divider (umso größer umso kleiner die Abweichung bei 0 => umso größer umso genauer)
#endif

#if true // global Variables
float sqrt3;

// Sensor und Laufzeit Variablen
float OrbitDirection = 0; // Richtung des Tores
uint8_t calibrationGyro;
uint8_t calibrationMag;
float IR_Direction = 0; // Richtung des IR Sensors
float IR_Distance = 0;  // Abstand zum Ball des IR Sensors
float IR_Heading = 0;   // Ungefähres Areal IR Sensors
bool IR_unreliable = true;
float US_Vorne = 0; // US
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
float rt_IR_Distance = 0;  // Richtung des IR Sensors
float rt_IR_Heading = 0;   // Ungefähres Areal IR Sensors
bool rt_IR_unreliable = true;
float rt_US_Vorne = 0; // US
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
float GoalTargetAngel = 0;

// System Variablen
float PIDKorrektur_memory;
int IR_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};     // IR Werte
int IR_values_raw[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // IR Werte (uint16_t)
bool switchValues[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Schalter
float KompassCliValue = 0;                       // Kompass
int DisplayRT = -1;
int InitT = -1;
int lastLoopTime = 0;
int lastUpdateTime = 0;
float stdSpeed = 0;
float Ballanfahrt = 0;      // DEBUG
float IR_Direction_Mod = 0; // Richtung des IR Sensors
bool Dribbleran = false;    // Dribblerstatus aktuell
int LoopError = 0;
int currentMotorstates[4] = {0, 0, 0, 0}; // Motorstates
byte newMotorstates[4] = {0, 0, 0, 0};    // Motorstates
float RawOrbit = 0;
int blocks;
int HomeSign = 0;                      // Pixy Own goal Signature
int EnemSign = 0;                      // Pixy Enemy Signature
int BallSign = 0;                      // Pixy Ball Signature
int PixyX[3] = {0, 0, 0};              // PixyX-coordinate
int PixyY[3] = {0, 0, 0};              // PixyY-coordinate
int PixyW[3] = {0, 0, 0};              // PixyWidth
int PixyH[3] = {0, 0, 0};              // PixyHeight
bool PixyS[3] = {false, false, false}; // PixySees
bool Pixy_knowsGoal = false;           // Pixy knows goal
int Pixy_GoalIs = 0;                   // -1 (Links), 0(Mitte), 1 (Rechts)
int Pixy_GoalDist = 0;                 // Pixy Distanz zum Tor
#endif

#if true // Start Libraries
Adafruit_NeoPixel RGBs = Adafruit_NeoPixel(LED_LENGTH, LED_PORT_DATA, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2I2C pixy;
#endif

#if true // Timing

IntervalTimer InteruptTimer; // 5ms, Robot::update

elapsedMillis ReadTimer;     // Read Timer
elapsedMillis KickerTimer;   // Kicker Timer
elapsedMillis DribblerTimer; // Kicker Timer
elapsedMillis LEDTimer;      // RGB Timer
elapsedMillis totalTime;     // Insgesamte Zeit die verstrichen ist
elapsedMillis LoopTime;      // Zeit die ein durchlauf braucht
elapsedMillis UpdateTime;    // Zeit die ein Update durchlauf braucht
elapsedMillis LoopTimer;     // immer hochzählen bis zehn oder so und dann starten
elapsedMillis LOPTimer;      // Lack of progress Timer
elapsedMillis InitTime;      // Zeit die ein initialize durchlauf braucht
elapsedMillis PIDTimer;      // PID Timer
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
        float Ballxm = 0;
        float Ballym = 0;

        IR_unreliable = true;

        // mapping
        for (int i = 0; i < 8; i++)
        {
            IR_values[i] = int(round(map(IR_values[i], IR_min[i], IR_max[i], 0, 100)));
            if (IR_values[i] > IR_Schwelle)
                IR_unreliable = false;
        }

        // Direction berechnen
        for (int i = 0; i < 8; i++)
        {

            if (IR_values[i] >= IR_Schwelle)
            {
                Bally += cosf((i * 45) * PI / 180) * IR_values[i];
                Ballx += sinf((i * 45) * PI / 180) * IR_values[i];
                Ballym += IR_values[i] * cosf(Gewicht[i] * PI / 180) - 4;
                Ballxm += IR_values[i] * sinf(Gewicht[i] * PI / 180);
            }
        }

        IR_Direction = atan2f(Ballx, Bally) * 180 / PI;       // Balloffset nicht einbauen
        IR_Direction_Mod = atan2f(Ballxm, Ballym) * 180 / PI; // Balloffset einbauen

        // IR_Heading berechnen
        IR_Heading = int(round((IR_Direction + 180) / (360 / IR_Range)));

        if (IR_Heading == IR_Range) // begrenzen der Ergebnisse auf 0...IR_Range-1
            IR_Heading = 0;

        IR_Heading += IR_Range / 2;
        if (IR_Heading >= IR_Range)
            IR_Heading -= IR_Range;

        // IR_Distance berechnen
        int maxValue = 0;
        int secondmaxValue = 0;

        for (int i = 0; i < 8; i++)
        {
            if (IR_values[i] > maxValue)
            {
                secondmaxValue = maxValue;
                maxValue = IR_values[i];
            }
            else if (IR_values[i] > secondmaxValue)
            {
                secondmaxValue = IR_values[i];
            }
        }

        float DistanceSum = maxValue + secondmaxValue;                  // Wert von 0 bis 200
        float DistanceSumpercentage = map(DistanceSum, 0, 200, 0, 100); // + (sinf(((IR_Direction + 11.25) * 8) * PI / 180) * 40); // Wert von 0 bis 100???
        IR_Distance = DistanceSumpercentage;

        // Ballanfahrt berechnen
        if (IR_Mode)
        {                                                                    // Ballanfahrtswinkel berechnen
            float AbsoluteDirection = abs(IR_Direction_Mod);                 // Wert von 0 bis 180
            float BallDirpercentage = map(AbsoluteDirection, 0, 180, 0, 80); // Wert von 0 bis 100 // Limit bei 80 sonst einfluss zu groß

            if (IR_Direction_Mod < 0)
            {
                BallDirpercentage = map(-IR_Direction_Mod, 0, 180, 0, 80) + 20;
            }
            else
            {
                BallDirpercentage = map(IR_Direction_Mod, 0, 180, 0, 80) + 20;
            }

            float DistanceCalcBase = 0.00012;
            float DistanceCalcFaktor = 1 + (DistanceSumpercentage * BallDirpercentage * DistanceCalcBase);
            Ballanfahrt = DistanceCalcFaktor * IR_Direction_Mod;
        }
        else
        {

            if (IR_Distance < IR_CircleR)
                Ballanfahrt = IR_Direction;
            else if (abs(IR_Direction) > 20)
            {
                if (US_offsetx > 30)
                    Ballanfahrt = IR_Direction - 85;
                else if (US_offsetx < -30)
                    Ballanfahrt = IR_Direction + 85;
                else
                {
                    if (Ballx < 0)
                        Ballanfahrt = IR_Direction - 85;
                    else
                        Ballanfahrt = IR_Direction + 85;
                }
            }
            else
                Ballanfahrt = IR_Direction * 1.6;
        }
    }

    // verarbeitet die ausgelesenen US Sensoren
    void US_Calc(void)
    {
        US_Vorne = US_value[3];
        US_Rechts = US_value[1];
        US_Hinten = US_value[2];
        US_Links = US_value[0];

        // tatsächliche Abstände Berechnen
        US_Rechts = US_Rechts * (1 + cosf(OrbitDirection * 180 / PI));
        US_Links = US_Links * (1 + cosf(OrbitDirection * 180 / PI));
        US_Vorne = US_Vorne * (1 + sinf(OrbitDirection * 180 / PI));
        US_Hinten = US_Hinten * (1 + sinf(OrbitDirection * 180 / PI));

        // Offset berechnen
        US_offsetx = (US_Rechts - US_Links) / 2;
        US_offsety = (US_Vorne - US_Hinten) / 2;
    }

    // verarbeitet die ausgelesenen Kamera Werte
    void Pixy_Calc(void)
    {
        // Homesign = eigenes Tor
        // Enemsign = gegnerisches Tor
        // Ballsign = Ball (EVtl bald)

        // Konzept:
        // alles was über einem bestimmten y wert ist sofort dumpen (Kalibrierungsfarben)
        // Center ausrechenen
        // Center x abweichung berechnen und nutzen
        // Output : GaolIs = -1 (Links), 0(Mitte), 1 (Rechts)
        // Output : Goaldistance = Höhe / Faktor X // Mapping (0, maxHeight, 100, 0)

        // EnemyGoal
        if (!(PixyY[EnemSign] > PixyYDumpValue))
        {
            Pixy_GoalDist = map(PixyY[EnemSign], PixyMinHeight, PixyMaxHeight, 100, 0); // Distance to goal

            //---

            int pixyXoffset = PixyX[EnemSign] + (PixyW[EnemSign] / 2);                                // X offset
            pixyXoffset = map(pixyXoffset, 0, PixyCamWidth, -1 * PixyCamWidth / 2, PixyCamWidth / 2); // Mapping to -160 to 160

            // pixyXoffset : Abweichung zur Mitte

            if (pixyXoffset < -PixyCamWidth / PixyDivider)
                Pixy_GoalIs = -1; // Links
            else if (pixyXoffset > PixyCamWidth / PixyDivider)
                Pixy_GoalIs = 1; // Rechts
            else
                Pixy_GoalIs = 0; // Mitte
        }
    }
};

// Liest die Sensoren aus
class CodeRead : private CodeCalculate
{
public:
    // liest IR Sensoren aus und übergibt an IR_Calc
    void IR(void)
    {
        SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE3));
        digitalWriteFast(ADC_PORT_CS, LOW); // Builtin ADC used for IR Ring

        for (int i = 0; i < 8; i++)
        {
            int val = SPI.transfer16(IR_Adressen[i]);
            IR_values[i] = val;
            IR_values_raw[i] = val;
        }

        digitalWriteFast(ADC_PORT_CS, HIGH); // Builtin ADC used for IR Ring
        SPI.endTransaction();
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

        { // send Read command
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
        // Calibration
        uint8_t dump1, dump2;
        calibrationGyro = 0;
        calibrationMag = 0;
        bno.getCalibration(&dump1, &calibrationGyro, &dump2, &calibrationMag);

        // Direction
        sensors_event_t event;
        bno.getEvent(&event);
        float KompassOutput = roundf(event.orientation.x); // 0 - 359

        RawOrbit = KompassOutput;

        OrbitDirection = KompassOutput - KompassCliValue;

        if (OrbitDirection < -180)
            OrbitDirection += 360;
        else if (OrbitDirection > 180)
            OrbitDirection -= 360;
    }

    // liest die Lichtschranke aus
    void LDR(void)
    {
        int LDRval = analogRead(LDR_PORT);    // LDR auslesen
        LDR_Ballda = (LDRval > LDR_Schwelle); // Lichtschranke auswerten

        if (LDRval > LDR_Schwelle + 300) // LDR Schwelle anpassen
        {
            LDR_Schwelle = LDRval - 250;
        }
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

        for (int i = 2; i < 8; i++)
        {
            byte Switches_new = switchValuesRead;
            byte Swi_byte = Switches_new >> i;
            int Swi_byte_new = Swi_byte;
            if (i > 4)
            {
                if (Swi_byte_new % 2 == 0)
                {
                    switchValues[i - 2] = 0;
                }
                else
                {
                    switchValues[i - 2] = 1;
                }
            }
            else
            {
                if (Swi_byte_new % 2 == 0)
                {
                    switchValues[i - 2] = 1;
                }
                else
                {
                    switchValues[i - 2] = 0;
                }
            }
        }

        MainSwitch = digitalRead(3);
        BTN1 = switchValues[0];
        BTN2 = switchValues[1];
        BTN3 = switchValues[2];
        BTN4 = false;
        BTN5 = false;
        SWI1 = switchValues[3];
        SWI2 = switchValues[4];
        SWI3 = switchValues[5];
        SWI4 = false;
        SWI5 = false;
        SWI6 = false;
    }

    // liest Pixycam aus
    void Pixy(void)
    {
        blocks = pixy.ccc.getBlocks(); // Number of found blocks

        for (int i = 0; i < 3; i++) // gespeicherte Blöcke zurücksetzen
        {
            PixyS[i] = false;
            PixyX[i] = 0;
            PixyY[i] = 0;
            PixyW[i] = 0;
            PixyH[i] = 0;
        }

        if (blocks) // wenn Blöcke gefunden wurden
        {
            for (int j = 0; j < blocks; j++)
            {

                if (true) // Debug ausgaben
                {
                    Serial.print("Block ");
                    Serial.print(j);
                    Serial.print(" : ");
                    pixy.ccc.blocks[j].print();
                }

                int sign = pixy.ccc.blocks[j].m_signature;
                if (sign < 2)
                {
                    PixyS[sign] = true;
                    PixyX[sign] = pixy.ccc.blocks[j].m_x;
                    PixyY[sign] = pixy.ccc.blocks[j].m_y;
                    PixyW[sign] = pixy.ccc.blocks[j].m_width;
                    PixyH[sign] = pixy.ccc.blocks[j].m_height;
                }
            }
        }

        Pixy_Calc();
    }

    // Kalibriert Pixy
    void calibratePixy(void)
    {
        Pixy();
        if (PixyS[0])
        {
            HomeSign = 0;
        }
        else if (PixyS[1])
        {
            HomeSign = 1;
        }
    }
};

// Optische Outputs
class CodeDebug
{
public:
    // Schaltet alle LEDS nach den Aktuellen Werten ein
    // @param doRGB true = RGB LEDs an, false = RGB LEDs aus
    void LED(bool doRGB)
    {
        if (LEDTimer < 100)
            return; // nur alle 100ms aktualisieren

        LEDTimer = 0; // Timer zurücksetzen

        if (doRGB)
        {

            // Kalibrierung (0, 1)
            if (calibrationGyro == 0)               // Gyro nicht kalibriert
                RGBs.setPixelColor(0, 255, 0, 0);   // rot
            else if (calibrationGyro == 1)          // Gyro so gut wie nicht kalibriert
                RGBs.setPixelColor(0, 255, 80, 0);  // orange
            else if (calibrationGyro == 2)          // Gyro so gut wie kalibriert
                RGBs.setPixelColor(0, 255, 255, 0); // gelb
            else if (calibrationGyro == 3)          // Gyro kalibriert
                RGBs.setPixelColor(0, 0, 255, 0);   // grün

            if (calibrationMag == 0)                // Magnetometer nicht kalibriert
                RGBs.setPixelColor(1, 255, 0, 0);   // rot
            else if (calibrationMag == 1)           // Magnetometer so gut wie nicht kalibriert
                RGBs.setPixelColor(1, 255, 80, 0);  // orange
            else if (calibrationMag == 2)           // Magnetometer so gut wie kalibriert
                RGBs.setPixelColor(1, 255, 255, 0); // gelb
            else if (calibrationMag == 3)           // Magnetometer kalibriert
                RGBs.setPixelColor(1, 0, 255, 0);   // grün

            // Kompass (7, LED0)
            if (OrbitDirection > -2 && OrbitDirection < 2)
                digitalWrite(23, HIGH);
            else
                digitalWrite(23, LOW);

            if (OrbitDirection <= 0)
                RGBs.setPixelColor(7, 0, 255 - (OrbitDirection * 1.41), (OrbitDirection * 1.41)); // grün bis Türkis
            else if (OrbitDirection > 0)
                RGBs.setPixelColor(7, (OrbitDirection * -1.41), 255 - (OrbitDirection * -1.41), 0); // grün bis Rot über Gelb

            // Status (8) // eig egal
            if (MainSwitch)                       // wenn Mainswitch umgelegt (Gameloop)
                RGBs.setPixelColor(8, 0, 255, 0); // grün
            else                                  // Wenn Mainswitch nicht umgelegt (Passiv)
                RGBs.setPixelColor(8, 0, 0, 255); // blau

            // Ballda (9)
            if (LDR_Ballda)                       // wenn Mainswitch umgelegt (Gameloop)
                RGBs.setPixelColor(9, 0, 255, 0); // grün
            else                                  // Wenn Mainswitch nicht umgelegt (Passiv)
                RGBs.setPixelColor(9, 255, 0, 0); // rot

            // IR (6)
            if (IR_unreliable)                                                                     // wenn IR unzuverlässig
                RGBs.setPixelColor(6, 0, 0, 255);                                                  // grün
            else if (IR_Heading == 0)                                                              // wenn Ball vor mir
                RGBs.setPixelColor(6, 0, 255, 0);                                                  // blau
            else if (IR_Heading > 0 && IR_Heading < IR_Range / 2)                                  // wenn Ball neben mir
                RGBs.setPixelColor(6, 255, 0, (int)(255 / (IR_Range / 2 - 1) * IR_Heading));       // rot bis lila
            else if (IR_Heading == 4)                                                              // wenn Ball hinter mir
                RGBs.setPixelColor(6, 255, 0, 0);                                                  // rot
            else if (IR_Heading > 4 && IR_Heading < 8)                                             // wenn ball neben mir
                RGBs.setPixelColor(6, 255, (int)(255 / (IR_Range / 2 - 1) * (IR_Heading - 4)), 0); // rot bis gelb
        }
        else
            RGBs.clear();

        RGBs.show();
    }

    // Gibt Seriel Werte aus
    // @param debug true = Debug, false = Normal
    void doSerial(bool debug = false)
    {
        if (!debug)
        {
            // Standard Ausgabe
            Serial.println("Bodensee Devils - Karl_GustavIII - " + version);
            Serial.print("TimeStamp: ");
            Serial.print(totalTime);
            Serial.print("; LoopTime: ");
            Serial.print(lastLoopTime);
            Serial.print("; LoopError: ");
            Serial.print(LoopError);
            Serial.print("; UpdateTime: ");
            Serial.println(lastUpdateTime);

            Serial.print("OrbitDirection: ");
            Serial.print(OrbitDirection);
            Serial.print("; IR_unreliable: ");
            Serial.print(IR_unreliable ? "Ja" : "Nein");
            Serial.print("; IR_Heading: ");
            Serial.print(IR_Heading);
            Serial.print("; IR_Direction: ");
            Serial.print(IR_Direction);
            Serial.print("; IR_DriveAngle (EXP): ");
            Serial.print(Ballanfahrt);
            Serial.print("; LDR_Ballda: ");
            Serial.println(LDR_Ballda ? "Da" : "Nicht Da");

            Serial.print("US_Vorne : ");
            Serial.print(US_Vorne);
            Serial.print("; US_Rechts : ");
            Serial.print(US_Rechts);
            Serial.print("; US_Hinten : ");
            Serial.print(US_Hinten);
            Serial.print("; US_Links : ");
            Serial.print(US_Links);
            Serial.print("; Usoffset x:");
            Serial.print(US_offsetx);
            Serial.print(", y:");
            Serial.println(US_offsety);

            /*Serial.print("TBD ");
            Serial.println("- Coming soon ");*/
        }
        else
        {
            Serial.print("Debug: ");
            Serial.print("TimeStamp: ");
            Serial.println(totalTime);

            String debugMode = "IR";

            if (debugMode == "LDR")
            {
                Serial.print("LDR: ");
                Serial.print(LDR_Ballda ? "Da" : "Nicht Da");
                Serial.print("; Raw: ");
                Serial.println(analogRead(LDR_PORT));
            }
            else if (debugMode == "IRcal")
            {
                Serial.print("IR_values: ");
                Serial.print(IR_values_raw[0]);
                Serial.print(", ");
                Serial.print(IR_values_raw[1]);
                Serial.print(", ");
                Serial.print(IR_values_raw[2]);
                Serial.print(", ");
                Serial.print(IR_values_raw[3]);
                Serial.print(", ");
                Serial.print(IR_values_raw[4]);
                Serial.print(", ");
                Serial.print(IR_values_raw[5]);
                Serial.print(", ");
                Serial.print(IR_values_raw[6]);
                Serial.print(", ");
                Serial.print(IR_values_raw[7]);
                Serial.println(";");
            }
            else if (debugMode == "IR")
            {
                Serial.print("IR_Direction: ");
                Serial.print(IR_Direction);
                Serial.print("; IR_Heading: ");
                Serial.print(IR_Heading);
                Serial.print("; IR_Distance: ");
                Serial.print(IR_Distance);
                Serial.print("; Ballanfahrt: ");
                Serial.print(Ballanfahrt);
                Serial.print("; IR_unreliable: ");
                Serial.println(IR_unreliable ? "Ja" : "Nein");
                Serial.print("IR_values: ");
                Serial.print(IR_values[0]);
                Serial.print(", ");
                Serial.print(IR_values[1]);
                Serial.print(", ");
                Serial.print(IR_values[2]);
                Serial.print(", ");
                Serial.print(IR_values[3]);
                Serial.print(", ");
                Serial.print(IR_values[4]);
                Serial.print(", ");
                Serial.print(IR_values[5]);
                Serial.print(", ");
                Serial.print(IR_values[6]);
                Serial.print(", ");
                Serial.print(IR_values[7]);
                Serial.println(";");
            }
            else if (debugMode == "US")
            {
                // Kopf
                Serial.print("US_Vorne: ");
                Serial.print(US_Vorne);
                Serial.print("; US_Rechts: ");
                Serial.print(US_Rechts);
                Serial.print("; US_Hinten: ");
                Serial.print(US_Hinten);
                Serial.print("; US_Links: ");
                Serial.println(US_Links);

                // Sensoren
                Serial.print("Sensor 1: 0,");    // 12 Zeichen
                Serial.print(" Sensor 2: 1,");   // 12 Zeichen
                Serial.print(" Sensor 3: 2,");   // 12 Zeichen
                Serial.println(" Sensor 4: 3,"); // 12 Zeichen

                for (int i = 0; i < 4; i++)
                {
                    char buf1[4];
                    sprintf(buf1, "%3.3d ", US_value[i]);
                    Serial.print(buf1);
                    Serial.print(", "); // 12 Zeichen ausgabe
                }

                Serial.print("US_offset x: ");
                Serial.print(US_offsetx);
                Serial.print(", y: ");
                Serial.println(US_offsety);
            }
            else if (debugMode == "Kompass")
            {
                // Kopf
                Serial.print("Kompass: ");
                Serial.print(KompassCliValue);
                Serial.print("; OrbitDirection: ");
                Serial.println(OrbitDirection);
            }
        }
    }
};

// Alles was sich Bewegt
class Codeaction
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
        Wire.endTransmission();

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

        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(ports);            // update output ports
        Wire.endTransmission();
    }

    // Setzt einen Motor
    // @param motpr: Motor Index (0 = hinten, 1 = links, 2 = rechts, 3 = Dribbler)
    // @param dir: Richtung (0: Freilauf, 1: Rechts, 2: Links, 3: Bremsen)
    // @param speed: Geschwindigkeit (0-100)
    void setMotorSpeed(int motor, bool dir, int speed)
    {
        bool Richtung = dir;
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.endTransmission(false);  // send restart instead of stop
        Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
        uint8_t ports = Wire.read();  // get current port state
        Wire.endTransmission();

        if (((motor == 0) && (Richtung == true)) || (currentMotorstates[0] == 1))
        {
            newMotorstates[0] = B00000001;
            currentMotorstates[0] = 1;
        }
        if (((motor == 0) && (Richtung == false)) || (currentMotorstates[0] == 2))
        {
            newMotorstates[0] = B00000010;
            currentMotorstates[0] = 2;
        }

        if (((motor == 1) && (Richtung == true)) || (currentMotorstates[1] == 1))
        {
            newMotorstates[1] = B00000100;
            currentMotorstates[1] = 1;
        }
        if (((motor == 1) && (Richtung == false)) || (currentMotorstates[1] == 2))
        {
            newMotorstates[1] = B00001000;
            currentMotorstates[1] = 2;
        }

        if (((motor == 2) && (Richtung == true)) || (currentMotorstates[2] == 1))
        {
            newMotorstates[2] = B00010000;
            currentMotorstates[2] = 1;
        }
        if (((motor == 2) && (Richtung == false)) || (currentMotorstates[2] == 2))
        {
            newMotorstates[2] = B00100000;
            currentMotorstates[2] = 2;
        }

        if (((motor == 3) && (Richtung == true)) || (currentMotorstates[3] == 1))
        {
            newMotorstates[3] = B01000000;
            currentMotorstates[3] = 1;
        }
        if (((motor == 3) && (Richtung == false)) || (currentMotorstates[3] == 2))
        {
            newMotorstates[3] = B10000000;
            currentMotorstates[3] = 2;
        }

        analogWrite(MotorPorts[motor], (uint16_t)speed * 256 / 100);

        byte Output = newMotorstates[0] | newMotorstates[1] | newMotorstates[2] | newMotorstates[3];
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(Output);           // update output ports Port_Seting
        Wire.endTransmission();
    }

    // Stoppt einen Motor
    // @param motor: Motor Index (0 = hinten, 1 = links, 2 = rechts, 3 = Dribbler)
    void brakeMotor(int motor)
    {
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.endTransmission(false);  // send restart instead of stop
        Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
        uint8_t ports = Wire.read();  // get current port state
        Wire.endTransmission();

        if ((motor == 0) || (currentMotorstates[0] == 0))
        {
            newMotorstates[0] = B00000011;
            currentMotorstates[0] = 0;
        }
        if ((motor == 1) || (currentMotorstates[1] == 0))
        {
            newMotorstates[1] = B00001100;
            currentMotorstates[1] = 0;
        }
        if ((motor == 2) || (currentMotorstates[2] == 0))
        {
            newMotorstates[2] = B00110000;
            currentMotorstates[2] = 0;
        }
        if ((motor == 3) || (currentMotorstates[3] == 0))
        {
            newMotorstates[3] = B11000000;
            currentMotorstates[3] = 0;
        }

        analogWrite(MotorPorts[motor], 0);

        byte Output = newMotorstates[0] | newMotorstates[1] | newMotorstates[2] | newMotorstates[3];
        Wire.beginTransmission(0x21); // address second port expander
        Wire.write(0x01);             // select output register
        Wire.write(Output);           // update output ports Port_Seting
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

        setMotorSpeed(0, dir0, abs(Motor0));
        setMotorSpeed(1, dir1, abs(Motor1));
        setMotorSpeed(2, dir2, abs(Motor2));
    }

    // Alle Motoren(FAHREN) stoppen
    void brake(void)
    {
        brakeMotor(0);
        brakeMotor(1);
        brakeMotor(2);
    }

    // Den Dribbler bedienen
    // @param an: Dribbler an/aus
    // @param dir: Dribbler Richtung (true : ball anziehen, false : ball abstoßen)
    void dribbler(bool an, bool dir)
    {
        // setMotorSpeed(3, dir ? 1 : 2, an ? 100 : 0); // Dribbler an
        setMotorSpeed(3, 2, an ? 100 : 0); // Dribbler an
    }

    // lädt den kicker auf
    void kicker_reset(void)
    {
        if (KickerTimer > 200)
        {
            Wire.beginTransmission(0x20); // address second port expander
            Wire.write(0x01);             // select output register
            Wire.write(0x00);             // set all ports to zero before switching direction
            Wire.endTransmission();
        }
    }

    // Schießt den Ball
    void kick(void)
    {
        if (KickerTimer > 500)
        {
            Wire.beginTransmission(0x20); // address second port expander
            Wire.write(0x01);             // select output register
            Wire.write(0xFD);             // Bit 0 on
            Wire.endTransmission();

            KickerTimer = 0;
        }
    }
};

/** Highlevel *****************************************************************************/

// Code für die Taktiken
class CodeTactics
{
    // nur rt_ Variablen verwenden
public:
    Codeaction action;

    // vor und zurück fahren wenn ein Lack of progress erkannt wird
    void LOP(void)
    {
        return; // SICHERUNG

        action.dribbler(true, true);

        // nur rt_ Variablen verwenden
        if (LOPTimer < LOP_TimerLimit + LOP_BackTime)
            action.move(180, 30, 0); // LOP_BackTime millisekunden Rückwärts fahren
        else if (LOPTimer < LOP_TimerLimit + LOP_BackTime + LOP_FrontTime)
            action.move(0, 100, 0); // LOP_FrontTime millisekunden volle kanne vorwärts donnern
        else
            LOPTimer = 0; // LOP Timer zuruecksetzen
    }

    // Ins eigene Tor fahren wenn der Ball nicht gesehen wird
    void homing(void)
    {
        return; // SICHERUNG

        // nur rt_ Variablen verwenden
        action.dribbler(false, true);

        float tmp_offX = rt_US_offsetx;
        float tmp_offY = rt_US_offsety - 40;                                       // US Offset formatieren
        float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                      // Richtung berechnen
        float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeedH; // Speed berechnen
        if (USspeed > stdSpeed - 20)                                               // wenn speed zu groß
            USspeed = stdSpeed - 20;                                               // Speed mappen
        action.move(sy_dir * USdriveAngle, USspeed, 0);                            // Fahren
    }

    // Den Ball anfahren
    void ballanfahrt(void)
    {
        return; // SICHERUNG
        // nur rt_ Variablen verwenden

        action.dribbler(true, true);

        if (true)
        {
            action.move(Ballanfahrt, stdSpeed, 0); // Ball anfahren
            return;
        }

        // Alte Ballanfahrtsfunktion

        if (rt_IR_Heading == 4) // genau hinter mir
        {
            if (rt_US_offsetx > 0)
                action.move(125, stdSpeed, 0); // seitlich nach hinten fahren
            else
                action.move(-125, stdSpeed, 0); // seitlich nach hinten fahren
        }
        else if (rt_IR_Heading == 3)          // schräg hinter mir
            action.move(180, stdSpeed, 0);    // Schräg nach hinten fahren
        else if (rt_IR_Heading == 5)          // schräg hinter mir
            action.move(-180, stdSpeed, 0);   // schräg nach hinten fahren
        else if (rt_IR_Heading == 2)          // neben mir
            action.move(150, stdSpeed, 25);   // fast seitlich fahren
        else if (rt_IR_Heading == 6)          // neben mir
            action.move(-150, stdSpeed, -25); // fast seitlich fahren
        else if (rt_IR_Heading == 1)          // vor und neben mir
            action.move(80, stdSpeed, 15);    // seitlich nach vorne fahren (mit offset)
        else if (rt_IR_Heading == 7)          // vor und neben mir
            action.move(-80, stdSpeed, -15);  // seitlich nach vorne fahren (mit offset)
        else                                  // Ball vor mir (0)
            action.move(0, stdSpeed, 0);      // geradeaus fahren
    }

    // Mit dem Ball ins Tor fahren / Schiessen
    // @param PixyDrive true = nach Pixy fahren, false = nach Kompass und US fahren
    void toranfahrt(bool PixyDrive)
    {
        return; // SICHERUNG
        // nur rt_ Variablen verwenden
        action.dribbler(true, true);

        if (PixyDrive) // Nach Pixy fahren
        {
            if (Pixy_GoalIs = 0 && Pixy_GoalDist > PixyDistSchwelle)
            {
                action.dribbler(true, false);
                action.kick();
            }
            else if (Pixy_GoalIs = 0 && Pixy_GoalDist < PixyDistSchwelle)
                action.move(0, stdSpeed, 0);
            else
                action.move(90 * Pixy_GoalIs, stdSpeed, 0);
        }
        else // Nach Kompass und US fahren
        {
            if (rt_OrbitDirection >= GoalTargetAngel - 2 && rt_OrbitDirection <= GoalTargetAngel + 2) // wenn ausgerichtet (entweder auf 0 oder aufs Torfür kicker vorbereitung)
            {
                if (rt_US_Vorne > 35) // Wenn weiter weg vom Tor (35 + Zentimeter) ins Tor kicken
                {
                    float tmp_offX = rt_US_offsetx;
                    float tmp_offY = rt_US_offsety + 35;                  // US Offset formatieren
                    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI; // TorRichtung berechnen

                    GoalTargetAngel = sy_dir; // neue GoalTargetDir setzen

                    if (rt_OrbitDirection >= GoalTargetAngel - 5 && rt_OrbitDirection <= GoalTargetAngel + 5) // wenn zum Tor gedreht
                        action.kick();                                                                        // kicken
                }
                else if (rt_US_Vorne > 10) // Wenn kurz vor Tor (35 - 10 Zentimeter) ins Tor fahren
                {
                    float tmp_offX = rt_US_offsetx;
                    float tmp_offY = rt_US_offsety + 35;                                      // US Offset formatieren
                    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                     // Richtung berechnen
                    float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeed; // Speed berechnen
                    if (USspeed > stdSpeed)                                                   // wenn speed zu groß
                        USspeed = stdSpeed;                                                   // Speed mappen
                    action.move(sy_dir * USdriveAngle, USspeed, 0);                           // Fahren
                }
                else // Wenn in Ecke oder ganz nah vor dem Gegnerischen Roboter nach Rechts oder links fahren
                {
                    if (rt_US_offsetx > 0)              // rechts
                        action.move(95, stdSpeed, -10); // nach Links fahren
                    else                                // Links
                        action.move(-95, stdSpeed, 10); // nach Rechts fahren
                }
            }
            else                                    // wenn nicht ausgerichtet
                action.move(0, 0, GoalTargetAngel); // Ausrichten
        }
    }
};

// Der Main Code für den Roboter
class CodeRobot
{
public:
    CodeRead Read;
    CodeTactics Tactics;
    CodeDebug Optical;

    // Initialisierung des Roboters
    void initialize(void)
    {
        InitTime = 0; // InitTimer zurücksetzen

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

        InitT = InitTime; // Initialisierungszeit speichern

        //-Optik----------------------------------------------------------------------------------//
        Serial.print(InitTime);

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
        ReadTimer = 0;
        KickerTimer = 0;
        LoopTime = 0;
        UpdateTime = 0;
        LoopTimer = 0;
        LOPTimer = 0;
        //----------------------------------------------------------------------------------------//
    }

    // Liest alle Sensoren aus und zeigt die Werte auf dem Display an
    void update(void)
    {
        UpdateTime = 0;
        Read.IR();
        Read.Compass();
        Read.LDR();
        Read.Pixy();
        Tactics.action.calculatePID();
        Tactics.action.kicker_reset();

        if (ReadTimer % 50 >= 45)
        {
            Read.Switches();
        }

        if (ReadTimer >= 110)
        {
            Read.US();
            ReadTimer = 0;
        }

        lastUpdateTime = UpdateTime;
    }

    // Handelt das Timing der Game Funktion und speichert alle sensor Werte
    void system(void)
    {
        if (LoopTimer >= LoopTiming) // Alle 10 ms Ausführen
        {
            LoopError = LoopTimer - LoopTiming; // Fehler im Loop Timer
            LoopTime = 0;                       // Timer der die Länge des Loops misst

            { // Alle gloablen Variablen speichern
                noInterrupts();
                rt_OrbitDirection = OrbitDirection;   // Richtung des Tores
                rt_calibrationGyro = calibrationGyro; // Kalibrierungswerte
                rt_calibrationMag = calibrationMag;
                rt_IR_Direction = IR_Direction; // Richtung des IR Sensors
                rt_IR_Distance = IR_Distance;   // Distanz des IR Sensors
                rt_IR_Heading = IR_Heading;     // Ungefähres Areal IR Sensors
                rt_IR_unreliable = IR_unreliable;
                rt_US_Vorne = US_Vorne; // US
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
                Optical.LED(false);
            }
            else
            {
                // SWI 1 = TBD
                // SWI 2 = Speed
                // SWI 3 = Debug
                // BTN 1 = Kick
                // BTN 2 = Pixy Own Goal einstellen
                // BTN 3 = Kalibrieren

                // DEBUG
                Tactics.action.brake();  // deactivate all motors
                Optical.LED(true);       // Debug HUD
                Optical.doSerial(false); // Serial Debug

                // BTNs und SWIs auswerten
                if (rt_BTN1)
                    Tactics.action.kick();

                if (rt_BTN2)
                    Read.calibratePixy();

                if (rt_BTN3)
                    KompassCliValue = RawOrbit;

                if (rt_SWI1) // TBD
                    ;
                else
                    ;

                if (rt_SWI2) // Speed toogle
                    stdSpeed = 50;
                else
                    stdSpeed = 80;

                if (rt_SWI3) // Dribbler debug
                    Tactics.action.dribbler(true, true);
                else
                    Tactics.action.dribbler(false, true);
            }

            lastLoopTime = LoopTime; // letzte Loop Zeit speichern
            LoopTimer = 0;           // System Timing zurücksetzen
        }
    }

    // Verarbeitet die wichtigsten Sensor Werte und entscheidet über die Taktiken
    void game(void)
    {
        // nur rt_ Variablen verwenden
        // Game loop
        return; // SICHERUNG

        bool hasBall = (rt_LDR_Ballda && rt_IR_Heading == 0 && !rt_IR_unreliable); // LDR und IR Sensor sehen den Ball

        if (LOPTimer >= LOP_TimerLimit)
            Tactics.LOP();
        else if (Pixy_knowsGoal) // Wenn die Pixy dass Tor sieht und der Ball da ist
            Tactics.toranfahrt(true);
        else if (hasBall) // LDR und IR Sensor sehen den Ball
            Tactics.toranfahrt(false);
        else if (!rt_IR_unreliable) // IR Sensor sieht den Ball
        {
            GoalTargetAngel = 0; // Toranfahrt zurücksetzen
            LOPTimer = 0;        // LOP Timer zuruecksetzen
            Tactics.ballanfahrt();
        }
        else
            Tactics.homing();
    }
};

/** Main **********************************************************************************/
CodeRobot Karl_GustavIII;

// alle Taktiken sind noch verplombt
// erst alle Sensorwerte ordentlich durchchecken
// dann display ordentlich durchchecken
// und dann die Taktiken einzeln via Debug Switch ordentlich durchchecken.
// dann den Gameloop testen

void InteruptTimerFunc(void)
{
    Karl_GustavIII.update();
}

void setup()
{
    Karl_GustavIII.initialize();
    InteruptTimer.begin(InteruptTimerFunc, 5 * 1000);
}

void loop()
{
    Karl_GustavIII.system();
}
/******************************************************************************************/