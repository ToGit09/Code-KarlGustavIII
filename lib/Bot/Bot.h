#pragma once

#ifndef BOT_H
#define BOT_H
#define BOT_VERSION "1.1.1"

#include <Arduino.h>

#define LED_LENGTH 19

struct motor_ports
{
    int IN1;
    int IN2;
    int PWM;
    int frequenz;
};

struct RGB_event
{
    int length;
    int r[LED_LENGTH];
    int g[LED_LENGTH];
    int b[LED_LENGTH];
    int brightness;
};

struct ir_sensor_event
{
    bool reliable;     // Zuverlässigkeit des IR Sensors
    int direction;     // Richtung des IR Sensors
    int Orbit_direction;     // Richtung des IR Sensors zum Orbit
    int distance;      // Abstand zum Ball des IR Sensors (0-1)
    int heading;       // Ungefähres Areal IR Sensors
    float ballanfahrt; // Winkel der Ballanfahr
    float ballanfahrt_neuW;
    float ballanfahrt_neuSF;
    uint16_t rawdata[16];
    bool TSOP[8];
};

struct us_sensor_event
{
    float dist_f;  // Distanz nach vorne
    float dist_b;  // Distanz nach hinten
    float dist_l;  // Distanz nach links
    float dist_r;  // Distanz nach rechts
    float offsetx; // US Offset X (parallel zum Tor)
    float offsety; // US Offset Y (senkrecht zum Tor)
    bool onLeftSide;
    bool positionedForGoal;
    bool reliable;
    int raw[4];
};

struct compass_sensor_event
{
    float orbitDirection; // Richtung des Tores
    int raw;              // Rohwert des Kompasses
    uint8_t calibrationGyro;
    uint8_t calibrationMag;
};

struct switches_event
{
    bool MainSwitch; // Start/Stopp Switch

    bool SWI1; // Schalter
    bool SWI2;
    bool SWI3;
    bool SWI4;
    bool SWI5;
    bool SWI6;
    bool SWI7;
    bool SWI8;

    bool BTN1; // Taster
    bool BTN2;
    bool BTN3;
    bool BTN4;
    bool BTN5;
    bool BTN6;
    bool BTN7;
    bool BTN8;
};

struct ldr_sensor_event
{
    bool ballda; // LDR
    int value; // Wert
};

struct camera_event
{
    bool goal_visible;  // Tor sichtbar
    float goal_heading; // Tor Richtung
};

struct movement_event
{
    float angle;         // Aktueller Winkel
    float speed;         // Aktuelle Geschwindigkeit
    float AngleOfAttack; // Angle of Attack
    float currentCompassAngle;
    bool dribbler;
};

struct motor_event
{
    int speeds[3];
    bool dirs[3];
    bool dribbler;
};

#include "includes.h"

#endif