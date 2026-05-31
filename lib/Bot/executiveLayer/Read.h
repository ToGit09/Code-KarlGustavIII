#ifndef READ_H
#define READ_H
#ifndef BOT_H
#include <Bot.h>
#endif

#include <IR.h>

// Rechnet die Rohen Werte in nutzbare werte um
class CodeCalculate
{
private:
    int US_L_Old[5] = {100, 100, 100, 100, 100};
    int US_R_Old[5] = {100, 100, 100, 100, 100};
    int US_L_idx = 0;
    int US_R_idx = 0;
    int IR_dist_Max = 0;

public:
    // verarbeitet die ausgelesenen IR Sensoren
    ir_sensor_event IR_Calc(ir_sensor_event ir, compass_sensor_event compass, us_sensor_event us);

    // verarbeitet die ausgelesenen US Sensoren
    us_sensor_event US_Calc(us_sensor_event us, compass_sensor_event compass);
};

// Liest die Sensoren aus
class CodeRead : private CodeCalculate
{
public:
    float KompassCliValue = 0;
    int16_t IR_offsets[16] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000}; // alle hoch damit werte akzeptiert werden
    uint16_t IR_gains[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                                                  // alle klein damit werte akzeptiert werden
    // IRC IR_Ring_Lib;

    // liest IR Sensoren aus und übergibt an IR_Calc
    ir_sensor_event IR(compass_sensor_event compass, us_sensor_event us, bool readRawData); // IR_Ring auslesen

    // liest die UltraschallSensoren aus
    us_sensor_event US(compass_sensor_event compass);

    // Liest die Kompass Werte aus
    compass_sensor_event Compass(void);

    // liest die Lichtschranke aus
    ldr_sensor_event LDR(void);

    void calibrateLDR(void);

    // liest die schalter und Taster aus
    switches_event Switches(void);

    // Kalibriert IR-Ring
    void calibrateIR(ir_sensor_event IR_Data);
};
#endif