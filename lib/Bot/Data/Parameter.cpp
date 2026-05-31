

#ifndef BOT_H
#include <Bot.h>
#endif

#if false // Defines
// Pin Definitions
#define MAINSWITCH_PORT 10

#define MOT1_IN1_PORT 0
#define MOT1_IN2_PORT 1
#define MOT2_IN1_PORT 2
#define MOT2_IN2_PORT 3
#define MOT3_IN1_PORT 7
#define MOT3_IN2_PORT 8
#define MOT4_IN1_PORT 26
#define MOT4_IN2_PORT 32
#define MOTOR1_PORT_PWM 4
#define MOTOR2_PORT_PWM 5
#define MOTOR3_PORT_PWM 6
#define MOTOR4_PORT_PWM 9

#define SPI_MOSI_PORT 11
#define SPI_MISO_PORT 12
#define SPI_SCK_PORT 13
#define PXY_RX_PORT 14
#define PXY_TX_PORT 15
// 16,17 I2C

#define KICKER_PORT 18

#define LED_PORT_DATA 20
#define LDR_PORT 23

// I2c Addresses
#define SWITCH_PORT_EXPANDER_ADDRESS 0x20

// SPI
#define SPI_DIO0 19
#define SPI_DIO1 31
#define SPI_DIO2 30
#define SPI_DIO3 21
#define SPI_DIO4 22

// Data
#define LED_LENGTH 3
#define I2C_BUS Wire1     // IRL-Ring on Wire1
#define I2C_SPEED 1000000 // 1 MHz

#endif

// Internal
String version = "V0.0.1"; // Version der Software

// Software
float stdSpeed = 0;

// Timing
long unsigned int LoopTiming = 4;       // Zeit in ms wann der Loop durchlaufen wird
long unsigned int LOP_TimerLimit = 8000; // maximale Zeit zwischen Ball erobert und Tor ohne außergewöhnlichen widerstand
long unsigned int LOP_BackTime = 500;    // Zeit um nach hinten zu fahren
long unsigned int LOP_FrontTime = 1100;  // Zeit um nach vorne zu fahren
int KickerExtendTime = 18;               // Zeit die der Kicker ausgefahren ist
int KickerCooldown = 800;                // Zeit die der Kicker hat abzukühlen
int DribbleSpeed = 10;

// IR
int IR_Schwelle = 200; // Schwelle für ir.reliable TBD -> Ab wann zwischen dioden und TSOP umgeschaltet wird
int IR_Range = 8;    // In wie viele Bereiche der IR Ring geteiolt wird // 8, 12, 20
float IR_CircleR = 10;
bool IR_Mode = false;                                                      // true = Tropfen, false = Kreis
// int IR_Adressen[8] = {2048, 4096, 6144, 8192, 10240, 12288, 14336, 16384}; // IR Adressen Katalog
// float Gewicht[8] = {0, 45, 90, 135, 180, -135, -90, -45};                  // IR anordnung in Grad (gemessen zu Fahrtrichtung) Mode 2
// int IR_min[8] = {20, 20, 20, 670, 20, 16, 24, 16};                         // IR Minimalwerte
// int IR_max[8] = {3200, 2500, 3350, 3500, 3400, 1500, 3400, 3600};          // IR Maximalwerte
float IR_front_Drive_Faktor = 1.2;                                         // Faktor mit dem der Ballwinkel multipliziert wird wenn der ball vor dem Roboter Liegt
float IR_Distance_Faktor = 1 / 1100;                                       // Faktor mit dem der Ballabstand multipliziert wird (output = wert zwischen 0 und 1)

// US
float USdivider = 0.8;
float USminSpeed = 40.0;
float USminSpeedH = 30.0;
float USdriveAngle = 1.2;
int US_address[4] = {0x70, 0x71, 0x72, 0x77};

// PID
float PID_P_Multiplier = 0.4;     // 4
float PID_I_Multiplier = 0.07;   // 1.4
float PID_D_Multiplier = 9;    // 15
float PID_Multiplikator = -1; // 0.06
float PIDpa_IlimitMax = 105;
float PIDpa_IlimitMin = -105;
float PIDpa_Imultiplier = 0.00025;
float PID_I_threshhold = 0;    // alt
float PID_Stand_Multiplier = 10; // ignore
float PID_Stand_Speed = 70;

// Motoren
int MotorFreqency = 400;    // PWM Frequenz für die Motoren
int DribblerFreqency = 200; // PWM Frequenz für die Motoren
motor_ports motor1 = {MOT1_IN1_PORT, MOT1_IN2_PORT, MOTOR1_PORT_PWM, MotorFreqency};
motor_ports motor2 = {MOT2_IN1_PORT, MOT2_IN2_PORT, MOTOR2_PORT_PWM, MotorFreqency};
motor_ports motor3 = {MOT3_IN1_PORT, MOT3_IN2_PORT, MOTOR3_PORT_PWM, MotorFreqency};
motor_ports motor4 = {MOT4_IN1_PORT, MOT4_IN2_PORT, MOTOR4_PORT_PWM, MotorFreqency};
motor_ports motoren[4] = {motor1, motor2, motor3, motor4};

// LED
int LEDBrightness = 20;

// LDR
int LDR_Schwelle = 450; // Grenzwert Lichtscharnke
int LDR_Hysterese = 120;

// Pixy
int PixyDistSchwelle = 50; // Pixy Distanzschwelle
int PixyYDumpValue = 250;  // Pixy Y Wert ab dem die Blöcke ignoriert werden
int PixyMaxHeight = 200;   // Pixy Maximalhöhe
int PixyMinHeight = 0;     // Pixy Minimalhöhe
int PixyCamWidth = 320;    // Pixy Cam Width
float PixyDivider = 5;     // Pixy Divider (umso größer umso kleiner die Abweichung bei 0 => umso größer umso genauer)

