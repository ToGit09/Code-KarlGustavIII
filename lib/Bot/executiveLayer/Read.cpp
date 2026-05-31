#ifndef READ_H
#include "Read.h"
#endif

#ifndef READ_CPP
#define READ_CPP

// Code Calculate
ir_sensor_event CodeCalculate::IR_Calc(ir_sensor_event ir, compass_sensor_event compass, us_sensor_event us)
{
    ir.ballanfahrt_neuW = 0;
    ir.ballanfahrt_neuSF = 1;

    float Divisor = 2.6f + (abs(ir.Orbit_direction) / 19.3f);

    ir.ballanfahrt_neuW = powf(float(abs(ir.Orbit_direction) / Divisor), 2);

    if (ir.Orbit_direction > 0)
        ir.ballanfahrt_neuW = ir.ballanfahrt_neuW * -1;

    /*if (abs(ir.Orbit_direction) < 100 && ir.distance < 200)
        ir.ballanfahrt_neuW = ir.Orbit_direction * -1.4;

    else if (abs(ir.Orbit_direction) >= 100 && ir.distance < 180)
        ir.ballanfahrt_neuW = ir.Orbit_direction * -1.2;*/

    if (abs(ir.Orbit_direction) < 20)
        ir.ballanfahrt_neuW = ir.Orbit_direction * -1.05;

    return ir;
}

us_sensor_event CodeCalculate::US_Calc(us_sensor_event us, compass_sensor_event compass)
{

    // tatsächliche Abstände Berechnen
    /* us.dist_r = us.dist_r * (1 + co-.,mmnba  QQWEDFV sf(compass.orbitDirection / 180 * PI));
    us.dist_l = us.dist_l * (1 + cosf(compass.orbitDirection / 180 * PI));
    us.dist_f = us.dist_f * (1 + sinf(compass.orbitDirection / 180 * PI));
    us.dist_b = us.dist_b * (1 + sinf(compass.orbitDirection / 180 * PI)); */

    float rechts = us.dist_r * cosf(compass.orbitDirection / 180 * PI);
    float links = us.dist_l * cosf(compass.orbitDirection / 180 * PI);
    float vorne = us.dist_f * cosf(compass.orbitDirection / 180 * PI);
    float hinten = us.dist_b * cosf(compass.orbitDirection / 180 * PI);

    // Offset berechnen
    us.offsetx = (us.dist_r - us.dist_l) / 2;
    us.offsety = (us.dist_f - us.dist_b) / 2;

    US_L_Old[US_L_idx] = us.dist_l;
    US_R_Old[US_R_idx] = us.dist_r;

    us.dist_l = (US_L_Old[0] + US_L_Old[1] + US_L_Old[2] + US_L_Old[3] + US_L_Old[4]) / 5;
    us.dist_r = (US_R_Old[0] + US_R_Old[1] + US_R_Old[2] + US_R_Old[3] + US_R_Old[4]) / 5;

    US_L_idx++;
    US_R_idx++;

    if (US_L_idx > 4)
        US_L_idx = 0;
    if (US_R_idx > 4)
        US_R_idx = 0;

    if (compass.orbitDirection > -20 && compass.orbitDirection < 20)
    {
        us.reliable = true; // wenn wir auf das Tor ausgerichtet sind, sind die US Werte zuverlässig
        us.onLeftSide = (/*Links ist kleiner als rechts*/ links < rechts);
        us.positionedForGoal = ((/*Wir sind mittig*/ (us.dist_l > 30 && us.dist_l < 70) && (us.dist_r > 30 && us.dist_r < 70)) && (/*wir sind nicht ganz hinten*/ us.dist_b > 20));
    }
    else
    {
        us.reliable = false; // wenn wir nicht auf das Tor ausgerichtet sind, sind die US Werte nicht zuverlässig
        us.onLeftSide = (/*Links ist kleiner als rechts*/ links < rechts);
        us.positionedForGoal = ((/*Wir sind mittig*/ (links > 30 && links < 70) && (rechts > 30 && rechts < 70)) && (/*wir sind nicht ganz hinten*/ hinten > 20));
    }

    return us;
}

camera_event CodeCalculate::Pixy_Calc(void)
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

    camera_event event;

    // EnemyGoal
    if (!(PixyY[EnemSign] > PixyYDumpValue))
    {
        Pixy_GoalDist = map(PixyY[EnemSign], PixyMinHeight, PixyMaxHeight, 100, 0); // Distance to goal

        //---

        int pixyXoffset = PixyX[EnemSign] + (PixyW[EnemSign] / 2);                                // X offset
        pixyXoffset = map(pixyXoffset, 0, PixyCamWidth, -1 * PixyCamWidth / 2, PixyCamWidth / 2); // Mapping to -160 to 160

        // pixyXoffset : Abweichung zur Mitte

        if (pixyXoffset < -PixyCamWidth / PixyDivider)
            event.goal_heading = -1; // Links
        else if (pixyXoffset > PixyCamWidth / PixyDivider)
            event.goal_heading = 1; // Rechts
        else
            event.goal_heading = 0; // Mitte
    }

    event.goal_visible = PixyS[EnemSign];

    return event;
}

// Code Read

ir_sensor_event CodeRead::IR(compass_sensor_event compass, us_sensor_event us, bool readRawData)
{
    IRh.read(readRawData);

    ir_sensor_event event;

    event.direction = IRh.Angle;                                      // relative Richtung zum Ball
    event.Orbit_direction = event.direction + compass.orbitDirection; // absoluter Winkel zum Ball (wird größtenteils benutzt)
    event.distance = IRh.Distance_raw;

    event.reliable = (event.distance > IR_Schwelle);
    event.heading = IRh.TSSP;
    IRh.GetTSSP(event.TSOP);

    if (readRawData)
        for (int i = 0; i < 16; i++)
            event.rawdata[i] = IRh.IR_Values[i];

    return IR_Calc(event, compass, us);
    // return event;
}

us_sensor_event CodeRead::US(compass_sensor_event compass)
{

    Wire1.endTransmission();
    us_sensor_event event;
    if (sensorHinten.update())
        event.dist_b = sensorHinten.getDistance(), sensorHinten.startRanging();

    if (sensorVorne.update())
        event.dist_f = sensorVorne.getDistance(), sensorVorne.startRanging();

    if (sensorLinks.update())
        event.dist_l = sensorLinks.getDistance(), sensorLinks.startRanging();

    if (sensorRechts.update())
        event.dist_r = sensorRechts.getDistance(), sensorRechts.startRanging();

    event.dist_f = sensorVorne.getDistance();
    event.dist_b = sensorHinten.getDistance();
    event.dist_l = sensorLinks.getDistance();
    event.dist_r = sensorRechts.getDistance();

    return US_Calc(event, compass);

    uint16_t distance = 0;
    { // get
        Wire1.endTransmission();

        Wire1.beginTransmission(US_address[0]);
        Wire1.write(0x02);                   // select first echo high-byte register
        Wire1.endTransmission(false);        // switch to read direction
        Wire1.requestFrom(US_address[0], 2); // Start a read access and expect 17 range words
        if (Wire1.available() >= 2)
        {
            distance = Wire1.read() << 8; // get high byte
            distance += Wire1.read();     // get low byte
            Wire1.endTransmission();
        }

        US_raw[0] = int(distance);

        distance = 0;

        Wire1.beginTransmission(US_address[1]);
        Wire1.write(0x02);                   // select first echo high-byte register
        Wire1.endTransmission(false);        // switch to read direction
        Wire1.requestFrom(US_address[1], 2); // Start a read access and expect 17 range words
        if (Wire1.available() >= 2)
        {
            distance = Wire1.read() << 8; // get high byte
            distance += Wire1.read();     // get low byte
            // Wire1.endTransmission();
        }
        US_raw[1] = int(distance);

        distance = 0;

        Wire1.beginTransmission(US_address[2]);
        Wire1.write(0x02);                   // select first echo high-byte register
        Wire1.endTransmission(false);        // switch to read direction
        Wire1.requestFrom(US_address[2], 2); // Start a read access and expect 17 range words
        if (Wire1.available() >= 2)
        {
            distance = Wire1.read() << 8; // get high byte
            distance += Wire1.read();     // get low byte
            // Wire1.endTransmission();
        }
        US_raw[2] = int(distance);

        distance = 0;

        Wire1.beginTransmission(US_address[3]);
        Wire1.write(0x02);                   // select first echo high-byte register
        Wire1.endTransmission(false);        // switch to read direction
        Wire1.requestFrom(US_address[3], 2); // Start a read access and expect 17 range words
        if (Wire1.available() >= 2)
        {
            distance = Wire1.read() << 8; // get high byte
            distance += Wire1.read();     // get low byte
            // Wire1.endTransmission();
        }
        US_raw[3] = int(distance);
    }

    { // send Read command
        Wire1.beginTransmission(US_address[0]);
        Wire1.write(0x00); // select version/command register
        Wire1.write(0x51); // send command: read in cm
        Wire1.endTransmission();

        Wire1.beginTransmission(US_address[1]);
        Wire1.write(0x00); // select version/command register
        Wire1.write(0x51); // send command: read in cm
        Wire1.endTransmission();

        Wire1.beginTransmission(US_address[2]);
        Wire1.write(0x00); // select version/command register
        Wire1.write(0x51); // send command: read in cm
        Wire1.endTransmission();

        Wire1.beginTransmission(US_address[3]);
        Wire1.write(0x00); // select version/command register
        Wire1.write(0x51); // send command: read in cm
        Wire1.endTransmission();
    }

    // us_sensor_event event;
    event.dist_f = US_raw[2];
    event.dist_b = US_raw[0];
    event.dist_l = US_raw[1];
    event.dist_r = US_raw[3];

    return US_Calc(event, compass);
}

compass_sensor_event CodeRead::Compass(void)
{
    compass_sensor_event Cevent;

    Cevent.raw = 0;
    Cevent.calibrationMag = -1;
    Cevent.calibrationGyro = -1;

    // Calibration
    uint8_t dump1, dump2;
    BNO.getCalibration(&dump1, &Cevent.calibrationGyro, &dump2, &Cevent.calibrationMag);
    // return Cevent;
    // Direction
    sensors_event_t event;
    BNO.getEvent(&event);
    float KompassOutput = roundf(event.orientation.x); // 0 - 359

    RawOrbit = KompassOutput;
    Cevent.raw = int(KompassOutput);

    Cevent.orbitDirection = KompassOutput - KompassCliValue;

    if (Cevent.orbitDirection < -180)
        Cevent.orbitDirection += 360;
    else if (Cevent.orbitDirection > 180)
        Cevent.orbitDirection -= 360;

    return Cevent;
}

ldr_sensor_event CodeRead::LDR(void)
{
    ldr_sensor_event event;
    int LDRval = analogRead(LDR_PORT); // LDR auslesen
    event.value = LDRval;
    event.ballda = (LDRval > LDR_Schwelle); // Lichtschranke auswerten

    /* if (LDRval < LDR_Schwelle - LDR_Hysterese) // LDR Schwelle anpassen
    {
        LDR_Schwelle = LDRval + LDR_Hysterese;
    } */

    return event;
}

void CodeRead::calibrateLDR(void)
{
    int LDRval = analogRead(LDR_PORT); // LDR auslesen

    LDR_Schwelle = LDRval + LDR_Hysterese;
}

switches_event CodeRead::Switches(void)
{
    Wire1.beginTransmission(0x20); // address second port expander
    Wire1.write(0x00);             // select input register
    Wire1.endTransmission(false);  // send repeated start instead of stop
    Wire1.requestFrom(0x20, 1);    // Start a read access and expect one byte
    byte last = Wire1.read();      // read Register
    Wire1.endTransmission();       // end of transmittion

    switches_event event;
    event.MainSwitch = digitalRead(MAINSWITCH_PORT);
    event.BTN1 = !(last & 1);
    event.BTN2 = !(last & (1 << 1));
    event.BTN3 = !(last & (1 << 2));
    event.BTN4 = !(last & (1 << 3));
    event.BTN5 = false;
    event.SWI1 = !(last & (1 << 4));
    event.SWI2 = !(last & (1 << 5));
    event.SWI3 = !(last & (1 << 6));
    event.SWI4 = false;
    event.SWI5 = false;
    event.SWI6 = false;

    return event;
}

camera_event CodeRead::Pixy(void)
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

    return Pixy_Calc();
}

void CodeRead::calibratePixy(void)
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

void CodeRead::calibrateIR(ir_sensor_event IR_Data)
{
    for (int i = 0; i < 16; i++)
    {
        if (IR_Data.rawdata[i] > IR_gains[i])
        {
            IR_gains[i] = IR_Data.rawdata[i];
            Serial.print("IR_Gain i: ");
            Serial.print(i);
            Serial.print(" v: ");
            Serial.println(IR_Data.rawdata[i]);
        }

        if (IR_Data.rawdata[i] < IR_offsets[i])
        {
            IR_offsets[i] = IR_Data.rawdata[i];
            Serial.print("IR_Offset i: ");
            Serial.print(i);
            Serial.print(" v: ");
            Serial.println(IR_Data.rawdata[i]);
        }
    }

    IRh.writeGains(IR_gains);
    IRh.writeOffsets(IR_offsets);
}
#endif