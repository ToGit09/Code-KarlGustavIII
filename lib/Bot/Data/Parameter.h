#ifndef PARAMETER_H
#define PARAMETER_H

#ifndef BOT_H
#include <Bot.h>
#endif

#if true // Defines
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
#define I2C_BUS Wire1     // IRL-Ring on Wire1
#define I2C_SPEED 1000000 // 1 MHz

// Definieren ob die Pixy per I2C oder SPI angesprochen wird
#define PixyI2C
// #define PixySPI
#endif

// Internal
extern String version ; // Version der Software

// Software
extern float stdSpeed ;

// Timing
extern long unsigned int LoopTiming ;       // Zeit in ms wann der Loop durchlaufen wird
extern long unsigned int LOP_TimerLimit ; // maximale Zeit zwischen Ball erobert und Tor ohne außergewöhnlichen widerstand
extern long unsigned int LOP_BackTime ;    // Zeit um nach hinten zu fahren
extern long unsigned int LOP_FrontTime ;  // Zeit um nach vorne zu fahren
extern int KickerExtendTime ;               // Zeit die der Kicker ausgefahren ist
extern int KickerCooldown ;                // Zeit die der Kicker hat abzukühlen

// IR
extern int IR_Schwelle ; // Schwelle für ir.reliable TBD -> Ab wann zwischen dioden und TSOP umgeschaltet wird
extern int IR_Range ;    // In wie viele Bereiche der IR Ring geteiolt wird // 8, 12, 20
extern float IR_CircleR ;
extern bool IR_Mode ;                                                      // true = Tropfen, false = Kreis
// int IR_Adressen[8] = {2048, 4096, 6144, 8192, 10240, 12288, 14336, 16384}; // IR Adressen Katalog
// float Gewicht[8] = {0, 45, 90, 135, 180, -135, -90, -45};                  // IR anordnung in Grad (gemessen zu Fahrtrichtung) Mode 2
// int IR_min[8] = {20, 20, 20, 670, 20, 16, 24, 16};                         // IR Minimalwerte
// int IR_max[8] = {3200, 2500, 3350, 3500, 3400, 1500, 3400, 3600};          // IR Maximalwerte
extern float IR_front_Drive_Faktor ;                                         // Faktor mit dem der Ballwinkel multipliziert wird wenn der ball vor dem Roboter Liegt
extern float IR_Distance_Faktor ;                                       // Faktor mit dem der Ballabstand multipliziert wird (output = wert zwischen 0 und 1)

// US
extern float USdivider ;
extern float USminSpeed;
extern float USminSpeedH;
extern float USdriveAngle ;
extern int US_address[4] ;

// Dribbler
extern int DribbleSpeed; // Dribbler Geschwindigkeit 0-100
extern bool Dribbler_dir ;

// PID
extern float PID_P_Multiplier ;     // 4
extern float PID_I_Multiplier;   // 1.4
extern float PID_D_Multiplier;    // 15
extern float PID_Multiplikator; // 0.06
extern float PIDpa_IlimitMax ;
extern float PIDpa_IlimitMin ;
extern float PIDpa_Imultiplier ;
extern float PID_I_threshhold ;
extern float PID_Stand_Multiplier ;
extern float PID_Stand_Speed ;

// Motoren
extern int MotorFreqency ;    // PWM Frequenz für die Motoren
extern int DribblerFreqency; // PWM Frequenz für die Motoren
extern motor_ports motor1;
extern motor_ports motor2;
extern motor_ports motor3 ;
extern motor_ports motor4 ;
extern motor_ports motoren[4] ;

// LED
extern int LEDBrightness ;

// LDR
extern int LDR_Schwelle ; // Grenzwert Lichtscharnke
extern int LDR_Hysterese ;

// Pixy
extern int PixyDistSchwelle ; // Pixy Distanzschwelle
extern int PixyYDumpValue ;  // Pixy Y Wert ab dem die Blöcke ignoriert werden
extern int PixyMaxHeight ;   // Pixy Maximalhöhe
extern int PixyMinHeight ;     // Pixy Minimalhöhe
extern int PixyCamWidth ;    // Pixy Cam Width
extern float PixyDivider;     // Pixy Divider (umso größer umso kleiner die Abweichung bei 0 => umso größer umso genauer)


#endif