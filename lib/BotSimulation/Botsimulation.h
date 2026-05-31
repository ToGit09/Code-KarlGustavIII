#include <Arduino.h>

#ifndef BOT_H
#include <Bot.h>
#endif

#ifndef BOTSIMULATION_H

#define BOTSIMULATION_H
#define BOTSIMULATION_VERSION "0.0.1"
#define Simulation
#define IR_Headings 8


bool DribblerAktiv = false;
bool kicker_extended = false;
float simuPID_I = 0.0;
float simuPIDKorrektur_memory = 0.0;

struct telegram
{
    char signature;  // a = ask - nomessage; t = tell; v = verify
    byte message[4]; // message
    // a:
    //   first Byte is Sensorsignature
    //      B00000001 -> IR Sensor
    //      B00000010 -> US Sensor
    //      B00000100 -> Compass Sensor
    //      B00001000 -> Switches/LDR
    //      B00010000 -> Camera (wird nicht simuliert)
    // t:
    //   first Byte is Motorsignature
	//		Bff ff ff 00 -> M1
	// 		Bff ff 00 ff -> M2
	// 		Bff 00 ff ff -> M3
	// 		Bf0 ff ff ff -> Dribbler an/aus
	// 		B0f ff ff ff -> kicker
	//   next bytes ar Motorspeeds
};

struct recievedTelegram
{
    byte message[6]; // message received
    // first Byte is Sensorsignature
    //      B00000001 -> IR Sensor
    //      B00000010 -> US Sensor
    //      B00000100 -> Compass Sensor
    //      B00001000 -> LDR Sensor
    //      B00010000 -> Switches
    //      B00100000 -> Camera (wird nicht simuliert)
    //      B10000001 -> IR Unreliable
    // next four bytes are data
    // if message[i] == 0 -> no following data
    // sensorvalue = message[i] - 1
};

void sendTelegram(telegram t);
recievedTelegram readTelegram(void);

class SimuRead
{
public:
    void init(void);
    ir_sensor_event IR(void);
    us_sensor_event US(compass_sensor_event compass);
    compass_sensor_event Compass(void);
    ldr_sensor_event LDR(void);
    camera_event Pixy(void);
    switches_event Switches(void);
    void calibratePixy(void); // Dump
};

class SimuAction
{
public:
    float calculatePID(compass_sensor_event compass, movement_event movement);
    void move(movement_event movement, float PID);
    void brake(void);
    void dribbler(bool an, bool dir);
    void kicker_reset(void);
    void kick(void);
};

#endif // BOTSIMULATION_H