/** Port Belegung ********************************************************************************/
// Pin       Funktion          Verfahren      Angeschlossen
//----------------------------------------------------------------------------
// 0         DISPLAY_IRQ       Interrupt      Display
// 2         DISPLAY_LITE      PWM            Display (Light)
// 3         START_SWITCH      GPIO           Schalter
// 4         PWM0              PWM            Motor0
// 5         PWM1              PWM            Motor1
// 6         PWM2              PWM            Motor2
// 7         DISPLAY_TFT_CS    GPIO           Display
// 8         DISPLAY_TFT_DC    GPIO           Display
// 9         PWM3              PWM            Dribbler
// 10        ADC_CS            GPIO           ADC
// 11        MOSI              SPI            ADC, Display
// 12        MISO              SPI            ADC, Display
// 13        SCK               SPI            ADC, Display
// 14        TNS_TX/PIXY_RX    UART3          Kamera
// 15        TNS_RX/PIXY_TX    UART3          Kamera
// 16        SPE-Data          JU_Ser         SP-Expander
// 17        SPE-Clock         JU_Ser         SP-Expander
// 18        SDA_3V3           I2C            Portexpander(0x20,0x21),BN0055
// 19        SCL_3V3           I2C            Portexpander(0x20,0x21),BN0055
// 20        RGB_DATA_3V3      I2C            RGB LEDs
// 21        LIGHT_BARRIER     I2C            Lichtschranke
// 22        BAT_VOLTAGE       ADC            Batteriespannung
// 23        LED0              GPIO           LED
//---------------------------------------------------------------------------------

/** Setup  ********************************************************************************/
#if true // Lib. include
#include <Arduino.h>
#include <Wire.h>              // for port expanders (Motor direction control, switches, Kicker)
#include <Adafruit_NeoPixel.h> // for RGB LEDs
#include <SPI.h>               // for ADC, Display and touch controller
#include <Adafruit_BNO055.h>   // compass sensor
#endif

#if true // Parameter
int UNBENUTZTEVARIABLEFÜRDASAUSSEHEN = 100;
int db_ProgrammSpeed = 25;                                        // Programm Samplezeit in Millisek.
float BallAnfartsFaktor = 1.22;                                   // zwischen 1 - 1.4 (1 == direkt auf den Ball ohne Kurve / >1 kurven Größe)
float AdditionBall = 10;                                          // Addition zur Ball Anfahrt
float AnfahrtsAbstandSchwelle = 50;                               // ab welcher entfernung zum ball der Roboter eine kurve einleitet
float BallDriveCut = 25;                                          // Additions Ende (Winkel) 25
float topSpeed = 100;                                             // wert zwischen 0-100 (im HIGH-Speed-Mode)
float highSpeed = 80;                                             // wert zwischen 0-100 (im HIGH-Speed-Mode)
float lowSpeed = 50;                                              // wert zwischen 0-100 (im LOW-Speed-Mode)
float MotorFreqency = 200;                                        // PWM Freqenz in Hz (optimal 200)
float IRGewichtung[8] = {0, 315, 270, 225, 180, 135, 90, 45};     // IR anordnung in Grad (gemessen zu Fahrtrichtung) {0,45,90,135,180,225,270,315}
int IR_min[8] = {20, 20, 20, 20, 20, 16, 20, 20};                 // minimale Signalstärke der IR's (digital = LOW)
int IR_max[8] = {3300, 3000, 3500, 3400, 3400, 3300, 3100, 3300}; // maximale Signalstärke der IR's (digital = HIGH)
int IR_schwelle[8] = {6, 6, 6, 22, 7, 6, 7, 6};                   // Nullwert + (25 bis 50)%
bool IR_unrelieable = false;                                      // gibt an ob die IR-Werte verlässlich sind
int IR_Range = 12;                                                 // In wie viele Bereiche der IR Ring geteiolt wird // 8, 12, 20
int LDR_Schwelle = 400;                                           // Grenzwert Lichtscharnke
bool IR_Raw = true;                                               // Rohe IR Werte (true) oder normierte Werte (false)
float USdivider = 0.8;
float USminSpeed = 30;
float USdriveAngle = 1.5;
int LEDBrightness = 8;

// PID
float PIDpa_P = 4;   // 4
float PIDpa_I = 1.4; // 1.4
float PIDpa_D = 15;  // 15
float PID_Stand_Multiplikator = 70;
float PID_Multiplikator = 0.06; // 0.06
float PIDpa_IlimitMax = 105;
float PIDpa_IlimitMin = -105;
float PIDpa_Imultiplier = 0.45;
float PID_I_threshhold = 12;
#endif

#if true // Lib. start
IntervalTimer myTimer;
Adafruit_NeoPixel RGBs = Adafruit_NeoPixel(10, 20, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#endif

#if true // Variablen
float db_Orbit_Dir2;
float stdSpeed;
float sqrt3;
float calibrationGyro_comp;
float db_euler_comp;
float db_comp_comp;
float PID_InternOutput;
uint8_t calibrationGyro;
uint8_t db_accel;
uint8_t calibrationMag;
uint8_t db_system2;
float db_Orbit_Dir;
float db_Error;
int db_Offset;
volatile unsigned int timingIndexSave;
volatile unsigned int timingIndex;
unsigned int timingIndexMemory;
float db_Dir0;
float db_Dir1;
float db_Dir2;
int db_G_Dir;
int IR_Adressen[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool db_Check;
int switchState1;
int switchState3;
int switchState2;
int db_Comp_Offset = 0;
int db_Ball_Anfahrts_Calc;
byte db_Zahl_OUT;
byte db_Motor0_N;
byte db_Motor1_N;
byte db_Motor2_N;
byte db_Motor3_N;
float Debug_Diff;
float db_Ball_dir_calc_x;
float db_Ball_dir_calc_y;
int db_motor0_U;
int db_motor1_U;
int db_motor2_U;
int db_motor3_U;
byte db_last_Byte;
float db_Velocity[3];
float db_Velocity_D[3]; // Velocity + und - zum Debbugen
float IR_value[8];      // Messwerte vom ADC, 0...1020 in 4er Schritten, Minimalwert
float db_Ball_dir_calc;
float db_Ball_dir_relativ;
float db_Ball_dir_dist;
float db_x_dir;
float db_y_dir;
float db_TurnFormal;
float db_CheckSumme;
int switchValuesRead;
int switchValues[7];
int db_last_LDR = 1026;
float db_esum;                                        // I-Regler Integrall
float db_e;                                           // P-Regler Fehler
float db_ealt;                                        // D-Regler Abweichung
double db_Setpoint1, db_Input1, PID_InternOutputput1; // PID variables
float Speed;
bool LDR_Ballda = false;     // Lichtschranke durchbrochen?
int relativeBallheading = 0; // ungefähre Richtung zum Ball
int USoffsetx = 0;
int USoffsety = 0;
uint16_t distance;
int searchoffset;
bool btnpressed = false;
bool topspeed = false;
float _highSpeed = 80;
int NewBallCalc_RelativeHeading;
int US_vorne = 0;
int US_hinten = 0;
#endif

class handleLED
{
public:
  void all(int r, int g, int b)
  {
    RGBs.clear();                      // initiativ alle ausmachen
    RGBs.setBrightness(LEDBrightness); // Brightness setzen
    for (int i = 0; i <= 10; i++)
    {
      RGBs.setPixelColor(i, RGBs.Color(r, g, b));
    }
    RGBs.show(); // änderungen anzeigen
  }

  void single(int i, int r, int g, int b)
  {
    RGBs.setBrightness(LEDBrightness); // Brightness setzen
    RGBs.setPixelColor(i, r, g, b);
  }

  void Update()
  {
    RGBs.show();
  }

  void status(int state) // 1 - Drive; 2 - Passive; 3 - Debug; 5 - Warning; 99 - ERROR
  {
    int r8 = 0;
    int g8 = 0;
    int b8 = 0;
    int r9 = 0;
    int g9 = 0;
    int b9 = 0;

    switch (state)
    {
    case 1: // Drive
      g8 = 255;
      break;

    case 11: // Drive, Ball suchen
      g8 = 255;
      r9 = 255;
      break;

    case 12: // Drive, Ball suchen: vor dem Roboter
      g8 = 255;
      r9 = 255;
      b9 = 255;
      break;

    case 13: // Drive, mit Ball ausrichten
      g8 = 255;
      b9 = 255;
      break;

    case 14: // Drive, shoot bevor USRead
      g8 = 255;
      r9 = 255;
      g9 = 255;
      break;

    case 15: // Drive, shoot nach USRead
      g8 = 255;
      g9 = 255;
      break;

    case 2: // Passive
      r8 = 255;
      break;

    case 31: // Debug standard
      b8 = 255;
      g9 = 255;
      break;

    case 32: // Debug Dongle
      b8 = 255;
      b9 = 255;
      break;

    case 33: // New Debug
      b8 = 255;
      r9 = 255;
      g9 = 255;
      b9 = 255;
      break;

    case 5:
      r8 = 255;
      g8 = 255;
      r9 = 255;
      g9 = 255;
      break;

    case 99: // ERROR
      r8 = 255;
      r9 = 255;
      break;
    }

    RGBs.setPixelColor(8, RGBs.Color(r8, g8, b8));
    RGBs.setPixelColor(9, RGBs.Color(r9, g9, b9));
    RGBs.show(); // änderungen anzeigen
  }
};

handleLED LED;

/** Lowlevel Code  ********************************************************************************/
// Berechnungen
class CalculateC
{
private:
public:
  void LDR()
  {
    int LDR_Wert = analogRead(21);
    LDR_Ballda = (LDR_Wert > LDR_Schwelle);
  }

  void Ball_Anfahrt()
  {
    if (db_CheckSumme > AnfahrtsAbstandSchwelle)
    {
      if (db_Ball_dir_calc < -4 && db_Ball_dir_calc > -BallDriveCut)
      {
        db_Ball_Anfahrts_Calc = db_Ball_dir_calc * BallAnfartsFaktor;
      }
      else if (db_Ball_dir_calc > 4 && db_Ball_dir_calc < BallDriveCut)
      {
        db_Ball_Anfahrts_Calc = db_Ball_dir_calc * BallAnfartsFaktor;
      }
      else if (db_Ball_dir_calc < -4)
      {
        db_Ball_Anfahrts_Calc = db_Ball_dir_calc * BallAnfartsFaktor - AdditionBall;
      }
      else if (db_Ball_dir_calc > 4)
      {
        db_Ball_Anfahrts_Calc = db_Ball_dir_calc * BallAnfartsFaktor + AdditionBall;
      }
      else
      {
        db_Ball_Anfahrts_Calc = 0;
      }
    }
    /*if entfernung > 60 :
          if degree2>83 and degree2<160 :
              heading=180
          elif degree2<=83 and  degree2>0:
              heading=degree2*1.8
          elif degree2<-83 and  degree2>-160:
              heading=180
          elif degree2>=-83 and  degree2<0:
              heading=degree2*1.8
          elif degree2<=-160 :
              heading=135
          elif degree2>160 :
              heading=-135
      else :
          heading=degree2*/
    else
    {
      db_Ball_Anfahrts_Calc = db_Ball_dir_calc;
    }
    db_Ball_Anfahrts_Calc = db_Ball_Anfahrts_Calc * (-1);
  }

  void Ball_Position()
  {

    db_CheckSumme = 0;
    for (int i = 0; i < 8; i++)
    {
      db_CheckSumme = IR_value[i] + db_CheckSumme;
    }

    db_x_dir = 0;
    db_y_dir = 0;

    for (int i = 0; i < 8; i++)
    {
      if (IR_value[i] > IR_schwelle[i])
      {
        db_x_dir = db_x_dir + IR_value[i] * cos(IRGewichtung[i] * PI / 180);
        db_y_dir = db_y_dir + IR_value[i] * sin(IRGewichtung[i] * PI / 180);
      }
    }

    db_Ball_dir_calc_x = db_x_dir;
    db_Ball_dir_calc_y = db_y_dir;
    db_Ball_dir_dist = (db_CheckSumme * (-1)) + 100;
    db_Ball_dir_calc = (atan2f(db_y_dir, db_x_dir) * (55));

    db_Ball_dir_relativ = db_Ball_dir_calc - db_Orbit_Dir;
  }

  float PID(float Offdrive)
  {

    db_Setpoint1 = Offdrive;

    db_Input1 = db_Orbit_Dir;

    db_e = db_Input1 - db_Setpoint1;

    db_esum = db_e + db_esum;

    if (db_esum > PIDpa_IlimitMax)
    {
      db_esum = PIDpa_IlimitMax;
    }
    else if (db_esum < PIDpa_IlimitMin)
    {
      db_esum = PIDpa_IlimitMin;
    }

    if (db_e > -(PID_I_threshhold) && db_e < PID_I_threshhold)
    {
      db_esum = 0;
    }

    Debug_Diff = (db_e - db_ealt);

    PID_InternOutput = ((PIDpa_P / 10) * db_e) + ((PIDpa_I / 10) * db_esum * PIDpa_Imultiplier) + ((PIDpa_D / 10) * (db_e - db_ealt));
    db_ealt = db_e;

    return -PID_InternOutput;
  }
};

class ActionC : public CalculateC
{

public:
  int Calibrate()
  {
    db_Offset = 0;

    imu::Vector<3> eulerCal = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    db_Offset = eulerCal.x();

    return db_Offset;
  }

  void Debug_BL()
  {
    // TBD
  }

  void move(float Degree2, float Speed_Percent, float TurnMoment)
  {

    Speed = Speed_Percent * 2.55;

    db_TurnFormal = PID(TurnMoment) * PID_Multiplikator;

    float rad = Degree2 * PI / 180;
    float CoorX = sin(rad) * 1;
    float CoorY = cos(rad) * 1;

    if (Speed == 0)
    {
      db_Velocity_D[0] = (db_TurnFormal * PID_Stand_Multiplikator);
      db_Velocity_D[1] = (db_TurnFormal * PID_Stand_Multiplikator);
      db_Velocity_D[2] = (db_TurnFormal * PID_Stand_Multiplikator);
    }
    else
    {
      db_Velocity_D[0] = ((((-0.5 * CoorX) + (sqrt3 / 2 * CoorY))) + db_TurnFormal);
      db_Velocity_D[1] = ((((-0.5 * CoorX) - (sqrt3 / 2 * CoorY))) + db_TurnFormal);
      db_Velocity_D[2] = ((CoorX) + db_TurnFormal);
    }

    // Alle werte auf + setzen
    if (db_Velocity_D[0] < 0)
    {
      db_Velocity[0] = db_Velocity_D[0] * -1;
      db_Dir0 = 0;
    }
    else
    {
      db_Velocity[0] = db_Velocity_D[0];
      db_Dir0 = 1;
    }
    if (db_Velocity_D[1] < 0)
    {
      db_Velocity[1] = db_Velocity_D[1] * -1;
      db_Dir1 = 0;
    }
    else
    {
      db_Velocity[1] = db_Velocity_D[1];
      db_Dir1 = 1;
    }
    if (db_Velocity_D[2] < 0)
    {
      db_Velocity[2] = db_Velocity_D[2] * -1;
      db_Dir2 = 0;
    }
    else
    {
      db_Velocity[2] = db_Velocity_D[2];
      db_Dir2 = 1;
    }

    // Werte Mappen auf "Speed"
    if (Speed != 0)
    {
      if ((db_Velocity[0] >= db_Velocity[1]) && (db_Velocity[0] >= db_Velocity[2]))
      {
        db_Error = Speed / db_Velocity[0];
        db_Velocity[0] = db_Velocity[0] * db_Error;
        db_Velocity[1] = db_Velocity[1] * db_Error;
        db_Velocity[2] = db_Velocity[2] * db_Error;
      }
      if ((db_Velocity[1] >= db_Velocity[0]) && (db_Velocity[1] >= db_Velocity[2]))
      {
        db_Error = Speed / db_Velocity[1];
        db_Velocity[0] = db_Velocity[0] * db_Error;
        db_Velocity[1] = db_Velocity[1] * db_Error;
        db_Velocity[2] = db_Velocity[2] * db_Error;
      }
      if ((db_Velocity[2] >= db_Velocity[1]) && (db_Velocity[2] >= db_Velocity[0]))
      {
        db_Error = Speed / db_Velocity[2];
        db_Velocity[0] = db_Velocity[0] * db_Error;
        db_Velocity[1] = db_Velocity[1] * db_Error;
        db_Velocity[2] = db_Velocity[2] * db_Error;
      }
    }

    if (db_Dir0 == 1)
    {
      Motor_Drive(0, true, db_Velocity[0]);
    }
    else
    {
      Motor_Drive(0, false, (db_Velocity[0]));
    }
    if (db_Dir1 == 1)
    {
      Motor_Drive(1, true, db_Velocity[1]);
    }
    else
    {
      Motor_Drive(1, false, (db_Velocity[1]));
    }
    if (db_Dir2 == 1)
    {
      Motor_Drive(2, true, db_Velocity[2]);
    }
    else
    {
      Motor_Drive(2, false, (db_Velocity[2]));
    }
  }

  void Stop()
  {
    Motor_Stop(0);
    Motor_Stop(1);
    Motor_Stop(2);
  }

  void Motor_Stop(int motor)
  {

    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x01);             // select output register
    Wire.endTransmission(false);  // send restart instead of stop
    Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
    db_last_Byte = Wire.read();   // get current port state
    Wire.endTransmission();

    if ((motor == 0) || (db_motor0_U == 0))
    {
      db_Motor0_N = B00000011;
      db_motor0_U = 0;
    }

    if ((motor == 1) || (db_motor1_U == 0))
    {
      db_Motor1_N = B00001100;
      db_motor1_U = 0;
    }

    if ((motor == 2) || (db_motor2_U == 0))
    {
      db_Motor2_N = B00110000;
      db_motor2_U = 0;
    }

    if ((motor == 3) || (db_motor3_U == 0))
    {
      db_Motor3_N = B11000000;
      db_motor3_U = 0;
    }

    if (motor == 0)
    {
      analogWrite(4, 0);
    }
    if (motor == 1)
    {
      analogWrite(5, 0);
    }
    if (motor == 2)
    {
      analogWrite(6, 0);
    }
    if (motor == 3)
    {
      analogWrite(9, 0);
    }

    db_Zahl_OUT = db_Motor0_N | db_Motor1_N | db_Motor2_N | db_Motor3_N;
    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x01);             // select output register
    Wire.write(db_Zahl_OUT);      // update output ports Port_Seting
    Wire.endTransmission();
  }

  void Motor_Drive(int motor, bool Richtung, float Speed)
  {

    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x01);             // select output register
    Wire.endTransmission(false);  // send restart instead of stop
    Wire.requestFrom(0x21, 1);    // Start a read access and expect one byte
    db_last_Byte = Wire.read();   // get current port state
    Wire.endTransmission();

    if (((motor == 0) && (Richtung == true)) || (db_motor0_U == 1))
    {
      db_Motor0_N = B00000001;
      db_motor0_U = 1;
    }
    if (((motor == 0) && (Richtung == false)) || (db_motor0_U == 2))
    {
      db_Motor0_N = B00000010;
      db_motor0_U = 2;
    }

    if (((motor == 1) && (Richtung == true)) || (db_motor1_U == 1))
    {
      db_Motor1_N = B00000100;
      db_motor1_U = 1;
    }
    if (((motor == 1) && (Richtung == false)) || (db_motor1_U == 2))
    {
      db_Motor1_N = B00001000;
      db_motor1_U = 2;
    }

    if (((motor == 2) && (Richtung == true)) || (db_motor2_U == 1))
    {
      db_Motor2_N = B00010000;
      db_motor2_U = 1;
    }
    if (((motor == 2) && (Richtung == false)) || (db_motor2_U == 2))
    {
      db_Motor2_N = B00100000;
      db_motor2_U = 2;
    }

    if (((motor == 3) && (Richtung == true)) || (db_motor3_U == 1))
    {
      db_Motor3_N = B01000000;
      db_motor3_U = 1;
    }
    if (((motor == 3) && (Richtung == false)) || (db_motor3_U == 2))
    {
      db_Motor3_N = B10000000;
      db_motor3_U = 2;
    }

    if (motor == 0)
    {
      analogWrite(4, Speed);
    }
    if (motor == 1)
    {
      analogWrite(5, Speed);
    }
    if (motor == 2)
    {
      analogWrite(6, Speed);
    }
    if (motor == 3)
    {
      analogWrite(9, Speed);
    }

    db_Zahl_OUT = db_Motor0_N | db_Motor1_N | db_Motor2_N | db_Motor3_N;
    Wire.beginTransmission(0x21); // address second port expander
    Wire.write(0x01);             // select output register
    Wire.write(db_Zahl_OUT);      // update output ports Port_Seting
    Wire.endTransmission();
  }
};

class ReadC
{
private:
  ActionC Action;

public:
  void CalibrationStatus()
  {
    db_system2 = 0;
    calibrationGyro = 0;
    db_accel = 0;
    calibrationMag = 0;
    bno.getCalibration(&db_system2, &calibrationGyro, &db_accel, &calibrationMag);
  }

  void Button()
  {
    Wire.beginTransmission(0x20);   // address first port expander
    Wire.write(0x00);               // select input register
    Wire.endTransmission(false);    // send repeated start instead of stop
    Wire.requestFrom(0x20, 1);      // Start a read access and expect one byte
    switchValuesRead = Wire.read(); // read inputs
    Wire.endTransmission();         // now send a stop

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
    switchValues[6] = digitalRead(3);

    if (switchValues[5] == true)
    {
      switchState1 = 1;
    }
    else if (switchValues[5] == false)
    {
      switchState1 = 0;
    }
    if (switchValues[4] == true)
    {
      switchState2 = 1;
    }
    else if (switchValues[4] == false)
    {
      switchState2 = 0;
    }
    if (switchValues[3] == true)
    {
      switchState3 = 1;
    }
    else if (switchValues[3] == false)
    {
      switchState3 = 0;
    }
  }

  void IR()
  {
    // Sensoren abfragen
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3)); // Start SPI und Einstellern der Übertragungsparameter
    digitalWrite(10, LOW);
    if (IR_Raw == false)
    {
      for (int i = 0; i < 8; i++)
      {
        IR_value[i] = SPI.transfer16(IR_Adressen[i]); // Ausgabewert: RAW
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        IR_value[i] = (SPI.transfer16(IR_Adressen[i]) * 100) / IR_max[i]; // Ausgabewert: ca. 0-100
      }
    }

    // IR Berechnung nach NewCode v0.2.3
    Action.Ball_Position();

    int temp = IR_Range;

    NewBallCalc_RelativeHeading = int(round((db_Ball_dir_relativ + 180) / (360 / temp)));

    if (NewBallCalc_RelativeHeading == temp)
    {
      NewBallCalc_RelativeHeading = 0;
    }

    NewBallCalc_RelativeHeading += temp / 2;
    if (NewBallCalc_RelativeHeading >= temp)
      NewBallCalc_RelativeHeading -= temp;

    // IR Berechnung nach NewCode v0.1.1

    int IR_max = 0;
    int IR_maxIndex = -1;

    IR_unrelieable = false;

    for (int i = 0; i < 8; i++) // Überprüfen welcher Sensor den größten Wert besitzt
    {
      if (IR_value[i] > IR_max && IR_value[i] > IR_schwelle[i]) // Mindestwert um Fehlmessungen zu vermeiden
      {
        IR_maxIndex = i;
        IR_max = IR_value[i];
      }
    }

    if (IR_maxIndex == -1)
      IR_unrelieable = true;

    relativeBallheading = NewBallCalc_RelativeHeading; // Ausgabewert in Variable Schreiben
    digitalWrite(10, HIGH);
    SPI.endTransaction();
  }

  float Quat_Vector()
  {
    imu::Quaternion quat = bno.getQuat();
    return quat.x() * 100;
  }

  float Acceleration()
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyro.x();
  }

  int alround_Compass()
  {
    sensors_event_t event;
    bno.getEvent(&event);
    db_G_Dir = round(event.orientation.x);

    db_G_Dir = db_G_Dir - db_Comp_Offset;
    if (db_G_Dir < 0)
    {
      db_G_Dir = db_G_Dir + 360;
    }
    else if (db_G_Dir > 359)
    {
      db_G_Dir = db_G_Dir - 360;
    }

    if (db_G_Dir <= 180)
    {
      db_G_Dir = db_G_Dir;
    }
    else if (db_G_Dir > 180)
    {
      db_G_Dir = db_G_Dir - 360;
    }
    if (db_G_Dir < -180)
    {
      db_G_Dir = db_G_Dir + 360;
    }
    else if (db_G_Dir > 359)
    {
      db_G_Dir = db_G_Dir - 360;
    }

    return db_G_Dir;
  }

  float euler_Vector()
  {
    float db_euler_vek;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    db_euler_vek = euler.x();
    db_euler_vek = db_euler_vek - db_Comp_Offset;
    if (db_euler_vek < 0)
    {
      db_euler_vek = db_euler_vek + 360;
    }
    else if (db_euler_vek > 359)
    {
      db_euler_vek = db_euler_vek - 360;
    }
    if (db_euler_vek <= 180)
    {
      db_euler_vek = db_euler_vek;
    }
    else if (db_euler_vek > 180)
    {
      db_euler_vek = db_euler_vek - 360;
    }
    if (db_euler_vek < -180)
    {
      db_euler_vek = db_euler_vek + 360;
    }
    else if (db_euler_vek > 359)
    {
      db_euler_vek = db_euler_vek - 360;
    }
    return db_euler_vek;
  }

  void Compass()
  {

    db_Orbit_Dir = alround_Compass();
    // db_Orbit_Dir = euler_Vector();
    // db_Orbit_Dir = Acceleration();

    if (db_Orbit_Dir < 2 && db_Orbit_Dir > -2)
    {
      digitalWrite(23, LOW);
    }
    else
    {
      digitalWrite(23, HIGH);
    }
  }
};

class handleUS
{
public:
  int address[4] = {0x70, 0x71, 0x72, 0x73};
  int value[4] = {0, 0, 0, 0};
  int offsetx = 0;
  int offsety = 0;
  int front;
  int left;
  int right;
  int back;

  void init(void)
  {
    // US Setup
    int idx;                  // zählt Sensoren durch
    int USnum = 0;            // Anzahl der gefundenen Sensoren
    int sensorAddress = 0x70; // I2C Adresse des aktuellen Sensors
    unsigned char rev;        // software revision of the sensor

    for (idx = 0; idx < 16; idx++)
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
        address[USnum] = sensorAddress;
        USnum++;
      }
      else
      {
        Serial.print("No sensor at ");
        Serial.print(sensorAddress, HEX);
        Serial.println("h");
      }
      sensorAddress++;
    }
    Read();
  }

  void Read(int num = 1)
  {
    if (num == 1)
    {
      Wire.beginTransmission(address[0]);
      Wire.write(0x00); // select version/command register
      Wire.write(0x51); // send command: read in cm
      Wire.endTransmission();
      Wire.beginTransmission(address[1]);
      Wire.write(0x00); // select version/command register
      Wire.write(0x51); // send command: read in cm
      Wire.endTransmission();
    }
    else
    {
      Wire.beginTransmission(address[2]);
      Wire.write(0x00); // select version/command register
      Wire.write(0x51); // send command: read in cm
      Wire.endTransmission();
      Wire.beginTransmission(address[3]);
      Wire.write(0x00); // select version/command register
      Wire.write(0x51); // send command: read in cm
      Wire.endTransmission();
    }
    /*
    for (int i = 0; i < 4; i++)
    {
      Wire.beginTransmission(address[i]);
      Wire.write(0x00); // select version/command register
      Wire.write(0x51); // send command: read in cm
      Wire.endTransmission();
    }*/
  }

  void get(int num = 1)
  {
    if (num == 1)
    {
      Wire.beginTransmission(address[0]);
      Wire.write(0x02);                // select first echo high-byte register
      Wire.endTransmission(false);     // switch to read direction
      Wire.requestFrom(address[0], 2); // Start a read access and expect 17 range words
      distance = Wire.read() << 8;     // get high byte
      distance += Wire.read();         // get low byte
      Wire.endTransmission();
      value[0] = int(distance);
      Wire.beginTransmission(address[1]);
      Wire.write(0x02);                // select first echo high-byte register
      Wire.endTransmission(false);     // switch to read direction
      Wire.requestFrom(address[1], 2); // Start a read access and expect 17 range words
      distance = Wire.read() << 8;     // get high byte
      distance += Wire.read();         // get low byte
      Wire.endTransmission();
      value[1] = int(distance);
    }
    else
    {
      Wire.beginTransmission(address[2]);
      Wire.write(0x02);                // select first echo high-byte register
      Wire.endTransmission(false);     // switch to read direction
      Wire.requestFrom(address[2], 2); // Start a read access and expect 17 range words
      distance = Wire.read() << 8;     // get high byte
      distance += Wire.read();         // get low byte
      Wire.endTransmission();
      value[2] = int(distance);
      Wire.beginTransmission(address[3]);
      Wire.write(0x02);                // select first echo high-byte register
      Wire.endTransmission(false);     // switch to read direction
      Wire.requestFrom(address[3], 2); // Start a read access and expect 17 range words
      distance = Wire.read() << 8;     // get high byte
      distance += Wire.read();         // get low byte
      Wire.endTransmission();
      value[3] = int(distance);
    }

    /*
    for (int i = 0; i < 4; i++)
    {
      Wire.beginTransmission(address[i]);
      Wire.write(0x02);                // select first echo high-byte register
      Wire.endTransmission(false);     // switch to read direction
      Wire.requestFrom(address[i], 2); // Start a read access and expect 17 range words
      distance = Wire.read() << 8;     // get high byte
      distance += Wire.read();         // get low byte
      Wire.endTransmission();
      value[i] = int(distance);
    }*/

    calculate();
  }

  void calculate(void)
  {
    // offsetx positiv wen links vom Mittelpunkt
    // offsety positiv wenn hinter dem Mittelpunkt
    front = value[3];
    left = value[0];
    right = value[1];
    back = value[2];

    US_vorne = front;
    US_hinten = back;

    if (right >= 240 || right <= 5)    // direkt an Rechter Wand
      offsetx = -60;                   // offset auf -60 setzen
    else if (left >= 240 || left <= 5) // direkt an Linker Wand
      offsetx = 60;                    // offset auf 60 setzen
    else                               // nicht an einer seitlichen Wand
      offsetx = (right - left) / 2;    // originalwerte benutzen

    if (back > 59)                           // Hintere Werte fehlerhaft
      offsety = (front - (160 - front)) / 2; // front Werte nehgmen
    else if (front > 59)                     // Wenn werte fehlerhaft sind
      offsety = ((160 - back) - back) / 2;   // front Wert simulieren
    else                                     // wenn Werte nicht Fehlerhaft sind
      offsety = (front - back) / 2;          // originalwerte benutzen

    USoffsetx = offsetx;
    USoffsety = offsety;
  }
};

/** Highlevel Code  ********************************************************************************/
class TacticsC
{
private:
  ReadC Read;
  ActionC Action;

public:
  void TorAnfahrt()
  {
    if (US_vorne <= 12 && USoffsetx > 0)
    {
      Action.move(90, stdSpeed, 10);
    }
    else if (US_vorne <= 12 && USoffsetx < 0)
    {
      Action.move(-90, stdSpeed, 10);
    }
    else
    {
      float tmp_offX = USoffsetx;
      float tmp_offY = USoffsety + 35;                                          // US Offset formatieren
      float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                     // Richtung berechnen
      float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeed; // Speed berechnen
      if (USspeed > stdSpeed)                                                   // wenn speed zu groß
        USspeed = stdSpeed;                                                     // Speed mappen
      Action.move(sy_dir * USdriveAngle, USspeed, 0);                           // Fahren
    }
  }

  void BallAnfahrt()
  {
    // 8 ist erprobt
    // 12 ist besser
    // 20 ist overkill

    if (IR_Range == 8)
    {
      if (/*USoffsetY <= -50 &&*/ USoffsetx > 20)
        searchoffset = -15;
      else if (USoffsetx < -20)
        searchoffset = 15;
      else
        searchoffset = 0;
      // Ballsuche
      if (relativeBallheading == 4) // genau hinter mir
      {
        if (USoffsetx > 0)
          Action.move(125, stdSpeed, searchoffset); // seitlich nach hinten fahren
        else
          Action.move(-125, stdSpeed, searchoffset); // seitlich nach hinten fahren
      }
      else if (relativeBallheading == 3)        // schräg hinter mir
        Action.move(180, stdSpeed, 0);          // Schräg nach hinten fahren
      else if (relativeBallheading == 5)        // schräg hinter mir
        Action.move(-180, stdSpeed, 0);         // schräg nach hinten fahren
      else if (relativeBallheading == 2)        // neben mir
        Action.move(150, stdSpeed, 0);          // fast seitlich fahren
      else if (relativeBallheading == 6)        // neben mir
        Action.move(-150, stdSpeed, 0);         // fast seitlich fahren
      else if (relativeBallheading == 1)        // vor und neben mir
        Action.move(80, stdSpeed, 15);          // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 7)        // vor und neben mir
        Action.move(-80, stdSpeed, -15);        // seitlich nach vorne fahren (mit offset)
      else                                      // Ball vor mir (0)
        Action.move(0, stdSpeed, 0); // geradeaus fahren
    }
    else if (IR_Range == 12)
    {
      if (/*USoffsetY <= -50 &&*/ USoffsetx > 20)
        searchoffset = -15;
      else if (USoffsetx < -20)
        searchoffset = 15;
      else
        searchoffset = 0;
      // Ballsuche
      if (relativeBallheading == 6) // genau hinter mir
      {
        if (USoffsetx > 0)
          Action.move(125, stdSpeed, searchoffset); // seitlich nach hinten fahren
        else
          Action.move(-125, stdSpeed, searchoffset); // seitlich nach hinten fahren
      }
      else if (relativeBallheading == 5)  // auf 5 Uhr
        Action.move(-160, stdSpeed, 0);   // seitlich vom Ball weg nach hinten
      else if (relativeBallheading == 7)  // auf 7 Uhr
        Action.move(160, stdSpeed, 0);    // seitlich vom Ball weg nach hinten
      else if (relativeBallheading == 4)  // Auf 4 Uhr
        Action.move(150, stdSpeed, 0);    // nach hinten und bissl seitlich fahren
      else if (relativeBallheading == 8)  // Auf 8 Uhr
        Action.move(-150, stdSpeed, 0);   // nach hinten und bissl seitlich fahren
      else if (relativeBallheading == 3)  // Auf 3 Uhr
        Action.move(110, stdSpeed, 0);    // fast seitlich fahren / bissl nach hinten
      else if (relativeBallheading == 9)  // Auf 9 Uhr
        Action.move(-110, stdSpeed, 0);   // fast seitlich fahren / bissl nach hinten
      else if (relativeBallheading == 2)  // neben und vor mir
        Action.move(90, stdSpeed, 0);     //  seitlich fahren
      else if (relativeBallheading == 10) // neben und vor mir
        Action.move(-90, stdSpeed, 0);    //  seitlich fahren
      else if (relativeBallheading == 1)  // vor und neben mir
        Action.move(60, stdSpeed, 15);    // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 11) // vor und neben mir
        Action.move(-60, stdSpeed, -15);  // seitlich nach vorne fahren (mit offset)
      else                                // Ball vor mir (0)
        Action.move(0, stdSpeed, 0);      // geradeaus fahren
    }
    else if (IR_Range == 20)
    {
      if (USoffsetx <= -50 && USoffsety > 20)
        searchoffset = 15;
      else if (USoffsety < -20)
        searchoffset = -15;
      else
        searchoffset = 0;
      // Ballsuche
      if (relativeBallheading == 10) // genau hinter mir
      {
        if (USoffsetx > 0)
          Action.move(125, stdSpeed, searchoffset); // seitlich nach hinten fahren
        else
          Action.move(-125, stdSpeed, searchoffset); // seitlich nach hinten fahren
      }
      else if (relativeBallheading == 9)  // schräg hinter mir
        Action.move(-140, stdSpeed, 0);   // Schräg nach hinten fahren
      else if (relativeBallheading == 11) // schräg hinter mir
        Action.move(140, stdSpeed, 0);    // schräg nach hinten fahren
      else if (relativeBallheading == 8)  // neben mir
        Action.move(-160, stdSpeed, 0);   // fast seitlich fahren
      else if (relativeBallheading == 12) // neben mir
        Action.move(160, stdSpeed, 0);    // fast seitlich fahren
      else if (relativeBallheading == 7)  // vor und neben mir
        Action.move(-170, stdSpeed, 0);   // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 13) // vor und neben mir
        Action.move(170, stdSpeed, 0);    // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 6)  // schräg hinter mir
        Action.move(160, stdSpeed, 0);    // Schräg nach hinten fahren
      else if (relativeBallheading == 14) // schräg hinter mir
        Action.move(-160, stdSpeed, 0);   // schräg nach hinten fahren
      else if (relativeBallheading == 5)  // neben mir
        Action.move(120, stdSpeed, 0);    // fast seitlich fahren
      else if (relativeBallheading == 15) // neben mir
        Action.move(-120, stdSpeed, 0);   // fast seitlich fahren
      else if (relativeBallheading == 4)  // vor und neben mir
        Action.move(90, stdSpeed, 0);     // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 16) // vor und neben mir
        Action.move(-90, stdSpeed, 0);    // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 3)  // schräg hinter mir
        Action.move(70, stdSpeed, 0);     // Schräg nach hinten fahren
      else if (relativeBallheading == 17) // schräg hinter mir
        Action.move(-70, stdSpeed, 0);    // schräg nach hinten fahren
      else if (relativeBallheading == 2)  // neben mir
        Action.move(45, stdSpeed, 0);     // fast seitlich fahren
      else if (relativeBallheading == 18) // neben mir
        Action.move(-45, stdSpeed, 0);    // fast seitlich fahren
      else if (relativeBallheading == 1)  // vor und neben mir
        Action.move(30, stdSpeed, 0);     // seitlich nach vorne fahren (mit offset)
      else if (relativeBallheading == 19) // vor und neben mir
        Action.move(-30, stdSpeed, 0);    // seitlich nach vorne fahren (mit offset)
      else                                // Ball vor mir (0)
        Action.move(0, stdSpeed, 0);      // geradeaus fahren
    }
    else
      Action.move((db_Ball_dir_calc * 1.3), stdSpeed, 0);
  }

  void homing()
  {
    float tmp_offX = USoffsetx;
    float tmp_offY = USoffsety - 75;                                          // US Offset formatieren
    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                     // Richtung berechnen
    float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeed; // Speed berechnen
    if (USspeed > stdSpeed)                                                   // wenn speed zu groß
      USspeed = stdSpeed;                                                     // Speed mappen
    Action.move(sy_dir, stdSpeed, 0);                                         // Fahren
  }
};

/** Main Code  ********************************************************************************/
class MainC
{
private:
  ReadC Read;
  ActionC Action;
  CalculateC Calculate;
  TacticsC Tactics;
  handleUS US;

public:
  void Game()
  {
    if (LDR_Ballda && (relativeBallheading == 0))
    {
      Tactics.TorAnfahrt();
    }
    else if (!IR_unrelieable)
    {
      Tactics.BallAnfahrt();
    }
    else
    {
      Tactics.homing();
    }
  }

  void System()
  {
    noInterrupts();                // TIMING
    timingIndexSave = timingIndex; // DONT TOUCH
    interrupts();                  // END TIMING

    if (timingIndexMemory != timingIndexSave) // Mainloop
    {
      if (timingIndexSave % 5 == 1) // US Sensoren auslesen (2. Hälfte)
      {
        Serial.print("X : ");
        US.get(1);
        US.Read(1);
      }
      else if (timingIndexSave % 5 == 3) // US Sensoren auslesen (1. Hälfte)
      {
        Serial.print("Y : ");
        US.get(2);
        US.Read(2);
      }
      else // DEBUG
      {
        Serial.print("    ");
      }

      Update(); // ReadSensors

      if (switchValues[0] == true) // Topspeed
      {
        if (!btnpressed)
        {
          if (topspeed)
            topspeed = false;
          else
            topspeed = true;
        }

        btnpressed = true;
      }
      else
        btnpressed = false;

      if (topspeed)
        _highSpeed = topSpeed;
      else
        _highSpeed = 80;

      if (switchState3 == 0)
        stdSpeed = lowSpeed; // Speedboost OFF
      else
        stdSpeed = _highSpeed; // Speedboost ON

      if (switchValues[2] == true)
        db_Comp_Offset = (Action.Calibrate()); // Comp Reset

      if (switchValues[6] == false)
      {                        // Game Schleifen Programm
        if (switchState2 == 1) // SW2 rot (Debug)
          Tactics.TorAnfahrt();
        // Action.move(0, 0, 0);
        else // (Gameloop)
          Game();
      }
      else // Start_Schleifen Programm
      {
        Action.Stop();            // Stehen bleiben
        Serial.print("timing: "); // Debug ausgaben
        Serial.print(timingIndexSave);
        Serial.print("; X: ");
        Serial.print(USoffsetx);
        Serial.print("  hinten: ");
        Serial.print(US.value[2]);
        Serial.print(", vorne: ");
        Serial.print(US.value[3]);
        Serial.print(", Y: ");
        Serial.println(US.offsety);
        /* Serial.print("; Balldircalc : ");
         Serial.print(db_Ball_dir_relativ);
         Serial.print("; NewBallcalc : ");
         Serial.print(NewBallCalc_RelativeHeading);
         Serial.print("; relativeHeading : ");
         Serial.print(relativeBallheading);
         Serial.println(";");*/
      }
    }

    timingIndexMemory = timingIndexSave; // TIMING DONT TOUCH
  }

  void DisplayRGBValues(void) // Werte an den RGBs anzeigen
  {
    // Calibration (0, 1)
    if (calibrationGyro == 0)      // Gyro nicht kalibriert
      LED.single(0, 255, 0, 0);    // rot
    else if (calibrationGyro == 1) // Gyro so gut wie nicht kalibriert
      LED.single(0, 255, 80, 0);   // orange
    else if (calibrationGyro == 2) // Gyro so gut wie kalibriert
      LED.single(0, 255, 255, 0);  // gelb
    else if (calibrationGyro == 3) // Gyro kalibriert
      LED.single(0, 0, 255, 0);    // grün

    if (calibrationMag == 0)      // Magnetometer nicht kalibriert
      LED.single(1, 255, 0, 0);   // rot
    else if (calibrationMag == 1) // Magnetometer so gut wie nicht kalibriert
      LED.single(1, 255, 80, 0);  // orange
    else if (calibrationMag == 2) // Magnetometer so gut wie kalibriert
      LED.single(1, 255, 255, 0); // gelb
    else if (calibrationMag == 3) // Magnetometer kalibriert
      LED.single(1, 0, 255, 0);   // grün

    // Kompass (7)
    if (db_Orbit_Dir <= 0)
      LED.single(7, 0, 255 - (db_Orbit_Dir * 1.41), (db_Orbit_Dir * 1.41));
    else if (db_Orbit_Dir > 0)
      LED.single(7, (db_Orbit_Dir * 1.41), 0, 255 - (db_Orbit_Dir * 1.41));

    // Status (8)
    if (switchValues[6] == 0)   // wenn Mainswitch umgelegt (Gameloop)
      LED.single(8, 0, 255, 0); // grün
    else                        // Wenn Mainswitch nicht umgelegt (Passiv)
      LED.single(8, 0, 0, 255); // blau

    // Ballda (9)
    if (LDR_Ballda)             // wenn Mainswitch umgelegt (Gameloop)
      LED.single(9, 0, 255, 0); // grün
    else                        // Wenn Mainswitch nicht umgelegt (Passiv)
      LED.single(9, 255, 0, 0); // rot

    // IR (6)
    if (IR_unrelieable)                                          // wenn IR unzuverlässig
      LED.single(6, 0, 0, 255);                                  // grün
    else if (relativeBallheading == 0)                           // wenn Ball vor mir
      LED.single(6, 0, 255, 0);                                  // blau
    else if (relativeBallheading > 0 && relativeBallheading < 4) // wenn Ball neben mir
      LED.single(6, 255, 0, 85 * relativeBallheading);           // rot bis lila
    else if (relativeBallheading == 4)                           // wenn Ball hinter mir
      LED.single(6, 255, 0, 0);                                  // rot
    else if (relativeBallheading > 4 && relativeBallheading < 8) // wenn ball neben mir
      LED.single(6, 255, 85 * (relativeBallheading - 4), 0);     // rot bis gelb

    // Switchstates: (3, 4)
    if (switchState2 == 0)
      LED.single(3, 0, 255, 0);
    else
      LED.single(3, 0, 0, 255);

    if (stdSpeed == lowSpeed)
      LED.single(4, 0, 50, 255);
    else if (stdSpeed == highSpeed)
      LED.single(4, 0, 200, 200);
    else
      LED.single(4, 255, 255, 255);

    LED.Update(); // anzeigen
  }

  void Update()
  {
    Read.IR();
    Read.Button();
    Read.CalibrationStatus();
    Read.Compass();
    Calculate.LDR();
    Calculate.Ball_Position();
    Calculate.Ball_Anfahrt();
    calibrationGyro_comp = Read.Acceleration();
    db_comp_comp = Read.alround_Compass();
    db_euler_comp = Read.euler_Vector();

    if (switchState1 == 1) // SW1 rot (Debug)
      DisplayRGBValues();
    else
      LED.all(0, 0, 0); // RGBs aus
  }

  void Setup()
  {

    sqrt3 = sqrt(3);

    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, INPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(21, INPUT); // L-Schranke

    // Setup of the Expander
    Wire.begin();
    Wire.beginTransmission(0x20);
    Wire.write(0x03);
    Wire.write(0xFC);
    Wire.endTransmission();
    Wire.beginTransmission(0x20);
    Wire.write(0x01);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.begin();
    Wire.beginTransmission(0x21);
    Wire.write(0x01);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(0x21);
    Wire.write(0x03);
    Wire.write(0x00);
    Wire.endTransmission();

    bno.begin();
    bno.setExtCrystalUse(true);

    /* pin 10-13 SPI with ADC chip select*/
    SPI.begin();

    /* initialize Supersonic*/
    US.init();

    digitalWrite(1, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(23, HIGH);

    // pin 2: Display backlight PWM
    analogWriteFrequency(2, 100);
    analogWrite(2, 255);

    RGBs.begin();
    delay(10);
    RGBs.setBrightness(0);
    RGBs.show();

    analogWriteFrequency(4, MotorFreqency);
    analogWriteFrequency(5, MotorFreqency);
    analogWriteFrequency(6, MotorFreqency);
    analogWriteFrequency(9, MotorFreqency);

    // IR Adressen Katalog
    IR_Adressen[0] = 2048;
    IR_Adressen[1] = 4096;
    IR_Adressen[2] = 6144;
    IR_Adressen[3] = 8192;
    IR_Adressen[4] = 10240;
    IR_Adressen[5] = 12288;
    IR_Adressen[6] = 14336;
    IR_Adressen[7] = 16384;
  }
};

/** Arduino Setup  ********************************************************************************/
MainC Regina;
void Timing_Counter() { timingIndex++; }
void setup()
{
  Regina.Setup();
  myTimer.begin(Timing_Counter, db_ProgrammSpeed * 1000);
}
void loop() { Regina.System(); }