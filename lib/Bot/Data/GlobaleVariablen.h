#ifndef BOT_H
#include <Bot.h>
#endif

#ifndef GLOBALEVARIABLEN_H
#define GLOBALEVARIABLEN_H

#define DOGV_CPP

// Sensoren neu

// Libraries
extern Adafruit_BNO055 BNO;

extern SRF08Sensor sensorVorne;
extern SRF08Sensor sensorHinten;
extern SRF08Sensor sensorLinks;
extern SRF08Sensor sensorRechts;
extern SRF08Manager sonar;


extern ESCC DRIBBLER;

extern float sqrt3;

extern int US_raw[4] ; // Ausgelesene Ultraschallwerte

// PID
extern float PID ;
extern float PID_P ;
extern float PID_I ;
extern float PID_D ;


// Taktische Variablen
extern float GoalTargetAngel; // überdenken

// System Variablen
extern int InitT;
extern int lastLoopTime;
extern int lastUpdateTime;
extern float IR_Direction_Mod; // Richtung des IR Sensors
extern bool DribblerState;            // Dribblerstatus aktuell
extern int LoopError;
extern float RawOrbit;
extern int blocks;
extern int HomeSign;      // Pixy Own goal Signature
extern int EnemSign;      // Pixy Enemy Signature
extern int BallSign;      // Pixy Ball Signature
extern int PixyX[3];      // PixyX-coordinate
extern int PixyY[3];      // PixyY-coordinate
extern int PixyW[3];      // PixyWidth
extern int PixyH[3];      // PixyHeight
extern bool PixyS[3];     // PixySees
extern int Pixy_GoalDist; // Pixy Distanz zum Tor

#endif