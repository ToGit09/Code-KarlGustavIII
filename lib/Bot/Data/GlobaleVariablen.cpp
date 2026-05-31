#ifndef BOT_H
#include <Bot.h>
#endif

// Sensoren neu

// Libraries

Adafruit_BNO055 BNO = Adafruit_BNO055(55, 0x28, &Wire1); // Sensor ID 55(Standard), 0x28 I2c Addresse (BNO055_ADDRESS_A), Wire1

SRF08Sensor sensorVorne(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor sensorHinten(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor sensorLinks(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor sensorRechts(SRF08_ADDR(0xEA), SRF08_RANGE_2M, SRF08_GAIN_MID);

SRF08Manager sonar;

ESCC DRIBBLER;

// IR-Ring setup
// I2C_bus i2c1(I2C_BUS);
// Device_handle ir_ring_handle = {i2c1, I2C_target{0x0A}};
// IR_ring ir_ring(ir_ring_handle);

float sqrt3;

int US_raw[4]; // Ausgelesene Ultraschallwerte

// PID
float PID = 0;
float PID_P = 0;
float PID_I = 0;
float PID_D = 0;

// Taktische Variablen
float GoalTargetAngel = 0; // überdenken

// System Variablen

int InitT = -1;
int lastLoopTime = 0;
int lastUpdateTime = 0;
float IR_Direction_Mod = 0; // Richtung des IR Sensors
bool DribblerState = false; // Dribblerstatus aktuell
int LoopError = 0;
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
int Pixy_GoalDist = 0;                 // Pixy Distanz zum Tor
