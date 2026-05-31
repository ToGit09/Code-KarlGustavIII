#ifndef BOTSIMULATION_H
#include "Botsimulation.h"
#endif

// Telegramme über serielle Schnittstelle senden
void sendTelegram(telegram t)
{
    Serial.print(t.signature);
    for (int i = 0; i < 4; i++)
    {
        if (t.message[i] == 0)
            break;
        Serial.print(t.message[i]);
    }
    Serial.println();
}

// Telegramme von serielle Schnittstelle empfangen
recievedTelegram recieveTelegram(void)
{
    recievedTelegram rt;
    while (Serial.available() < 5)
        ; // Warten bis 5 Bytes empfangen wurden

    for (int i = 0; i < 4; i++)
    {
        rt.message[i] = Serial.read();
    }

    Serial.print("v"); // Bestätigung senden
    for (int i = 0; i < 4; i++)
    {
        if (rt.message[i] == 0)
            break;
        Serial.print(rt.message[i]);
    }

    return rt;
}

// Initialisierung des Teensys im Simulationsmodus
void SimuRead::init(void)
{
    Serial.begin(9600);

    // Initialisierung der Ports
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

    digitalWrite(1, LOW);
    digitalWrite(2, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(23, LOW);

    sqrt3 = sqrtf(3);

    //warten bis connected

    while(!Serial.available())
        ;

    while (Serial.available())
    {
        Serial.read();
    }



}

// Sensoren

// IR Sensor simulieren
// per Telegramm anfragen und verarbeiten
ir_sensor_event SimuRead::IR(void)
{
    // Simulate IR sensor reading
    // telegram vorbereiten
    telegram t;
    t.signature = 'a';        // tell
    t.message[0] = B00000001; // B00000001 um IR Sensor anzufragen
    t.message[1] = 0;         // Platzhalter
    t.message[2] = 0;         // Platzhalter
    t.message[3] = 0;         // Platzhalter

    sendTelegram(t);

    while (!Serial.available())
        ;

    recievedTelegram rt = recieveTelegram();
    // rt.message[0] = Signature / Reliable(fist bit)
    // rt.message[1] = positive Direction 0 -  180
    // rt.message[2] = negative Direction 0 -  180
    // Direction = (rt.message[2] - 1) - (rt.message[3] - 1)
    // rt.message[3] = Distance 0 - 255

    ir_sensor_event event;

    if (rt.message[0] == B00000001)
    {
        // Verarbeite die empfangenen IR-Daten
        // Beispiel: rt.message[0] = IR_Heading, rt.message[1] = IR_Distance
        event.reliable = true;
        int posDir = (int)rt.message[1] - 1;
        int negDir = (int)rt.message[2] - 1;
        event.direction = posDir - negDir; // Zusammensetzug der Richtung
        event.distance = (int)rt.message[3] - 1;

        event.heading = round((event.direction + 180) / (360 / IR_Headings));
        event.ballanfahrt = event.direction * 1.6;
    }
    else if (rt.message[0] == B10000001)
    {
        event.reliable = false;
        event.direction = 0;
        event.distance = 0;
        event.heading = 0;
        event.ballanfahrt = event.direction * 1.6;
    }

    return event;
}

// US Sensor simulieren
// per Telegramm anfragen und verarbeiten
us_sensor_event SimuRead::US(compass_sensor_event compass)
{
    // Simulate US sensor reading
    // telegram vorbereiten
    telegram t;
    t.signature = 'a';        // ask
    t.message[0] = B00000010; // B00000001 um US Sensor anzufragen
    t.message[1] = 0;         // Platzhalter
    t.message[2] = 0;         // Platzhalter
    t.message[3] = 0;         // Platzhalter

    sendTelegram(t); // Telegramm senden

    while (!Serial.available()) // Auf Antwort warten
        ;

    recievedTelegram rt = recieveTelegram(); // Telegramm empfangen

    us_sensor_event event; // US Sensor Event erstellen
    // rt.message[0] = Signature
    // rt.message[1] = dist_f (vorne)
    // rt.message[2] = dist_b (hinten)
    // rt.message[3] = dist_l (links)
    // rt.message[4] = dist_r (rechts)

    if (rt.message[0] == B00000010)
    {
        // keine Normierung der Wert nötig da simulierte werte bereits in cm parallel zum Tor sind
        event.dist_f = (float)rt.message[1];
        event.dist_b = (float)rt.message[2];
        event.dist_l = (float)rt.message[3];
        event.dist_r = (float)rt.message[4];
        event.offsetx = (event.dist_r - event.dist_l) / 2.0;
        event.offsety = (event.dist_f - event.dist_b) / 2.0;
    }

    return event;
}

// Kompass simulieren
// per Telegramm anfragen und verarbeiten
compass_sensor_event SimuRead::Compass(void)
{
    // Simulate Compass sensor reading
    // telegram vorbereiten
    telegram t;
    t.signature = 'a';        // ask
    t.message[0] = B00000100; // B00000100 um Kompass Sensor anzufragen
    t.message[1] = 0;         // Platzhalter
    t.message[2] = 0;         // Platzhalter
    t.message[3] = 0;         // Platzhalter

    sendTelegram(t); // Telegramm senden

    while (!Serial.available()) // Auf Antwort warten
        ;

    recievedTelegram rt = recieveTelegram(); // Telegramm empfangen
    // rt.message[0] = Signature
    // rt.Message[1] = Calibration (B0000 1111 _> Gyro = 1 Mag = 0)
    // rt.Message[2] = positive OrbitDirection 0 - 180
    // rt.Message[3] = negative OrbitDirection 0 - 180
    // OrbitDirection = (rt.message[3] - 1) - (rt.message[2] - 1)

    compass_sensor_event event; // Kompass Sensor Event erstellen
    event.raw = 0;

    if (rt.message[0] == B00000100)
    {
        // Calibration auswerten
        event.calibrationGyro = (uint8_t)rt.message[1] & B00001111;
        event.calibrationMag = (uint8_t)(rt.message[1] & B11110000) >> 4;

        int posDir = (int)rt.message[2] - 1;
        int negDir = (int)rt.message[3] - 1;
        event.orbitDirection = posDir - negDir; // Zusammensetzug der Richtung
        event.raw = (int)roundf(event.orbitDirection);
    }

    return event;
}

// LDR Sensor simulieren
// per Telegramm anfragen und verarbeiten
ldr_sensor_event SimuRead::LDR(void)
{
    // Simulate Switches/LDR sensor reading
    // telegram vorbereiten
    telegram t;
    t.signature = 'a';        // ask
    t.message[0] = B00001000; // B00000001 um Switches/LDR Sensor anzufragen
    t.message[1] = 0;         // Platzhalter
    t.message[2] = 0;         // Platzhalter
    t.message[3] = 0;         // Platzhalter

    sendTelegram(t);

    while (!Serial.available())
        ;

    recievedTelegram rt = recieveTelegram();
    // rt.message[0] = Signature
    // rt.message[1] = Switches as byte
    //     B00000001 -> SWI1
    //     B00000010 -> SWI2
    //     B00000100 -> SWI3
    //     B00001000 -> SWI4
    //     B00010000 -> SWI5
    //     B00100000 -> SWI6
    //     B01000000 -> SWI7
    //     B10000000 -> SWI8
    // rt.message[2] = Buttons as byte
    //     B00000001 -> BTN1
    //     B00000010 -> BTN2
    //     B00000100 -> BTN3
    //     B00001000 -> BTN4
    //     B00010000 -> BTN5
    //     B00100000 -> BTN6
    //     B01000000 -> BTN7
    //     B10000000 -> BTN8
    // rt.message[3] = MainSwitch/LDR
    //    B00000001 -> MainSwitch
    //    B00000010 -> LDR

    ldr_sensor_event event;
    if (rt.message[0] == B00001000)
    {
        byte ldrValueRead = rt.message[3];

        event.ballda = (bool)(ldrValueRead & B00000010) >> 1;
    }

    return event;
}

// Schalterabfrage simulieren
// per Telegramm anfragen und verarbeiten
switches_event SimuRead::Switches(void)
{
    // Simulate Switches/LDR sensor reading
    // telegram vorbereiten
    telegram t;
    t.signature = 'a';        // ask
    t.message[0] = B00001000; // B00000001 um Switches/LDR Sensor anzufragen
    t.message[1] = 0;         // Platzhalter
    t.message[2] = 0;         // Platzhalter
    t.message[3] = 0;         // Platzhalter

    sendTelegram(t);

    while (!Serial.available())
        ;

    recievedTelegram rt = recieveTelegram();
    // rt.message[0] = Signature
    // rt.message[1] = Switches as byte
    //     B00000001 -> SWI1
    //     B00000010 -> SWI2
    //     B00000100 -> SWI3
    //     B00001000 -> SWI4
    //     B00010000 -> SWI5
    //     B00100000 -> SWI6
    //     B01000000 -> SWI7
    //     B10000000 -> SWI8
    // rt.message[2] = Buttons as byte
    //     B00000001 -> BTN1
    //     B00000010 -> BTN2
    //     B00000100 -> BTN3
    //     B00001000 -> BTN4
    //     B00010000 -> BTN5
    //     B00100000 -> BTN6
    //     B01000000 -> BTN7
    //     B10000000 -> BTN8
    // rt.message[3] = MainSwitch/LDR
    //    B00000001 -> MainSwitch
    //    B00000010 -> LDR

    switches_event event;

    if (rt.message[0] == B00001000)
    {
        byte switchValuesRead = rt.message[1];
        byte buttonValuesRead = rt.message[2];
        byte mainSwitchRead = rt.message[3];

        event.SWI1 = (bool)switchValuesRead & B00000001;
        event.SWI2 = (bool)(switchValuesRead & B00000010) >> 1;
        event.SWI3 = (bool)(switchValuesRead & B00000100) >> 2;
        event.SWI4 = (bool)(switchValuesRead & B00001000) >> 3;
        event.SWI5 = (bool)(switchValuesRead & B00010000) >> 4;
        event.SWI6 = (bool)(switchValuesRead & B00100000) >> 5;
        event.SWI7 = (bool)(switchValuesRead & B01000000) >> 6;
        event.SWI8 = (bool)(switchValuesRead & B10000000) >> 7;

        event.BTN1 = (bool)buttonValuesRead & B00000001;
        event.BTN2 = (bool)(buttonValuesRead & B00000010) >> 1;
        event.BTN3 = (bool)(buttonValuesRead & B00000100) >> 2;
        event.BTN4 = (bool)(buttonValuesRead & B00001000) >> 3;
        event.BTN5 = (bool)(buttonValuesRead & B00010000) >> 4;
        event.BTN6 = (bool)(buttonValuesRead & B00100000) >> 5;
        event.BTN7 = (bool)(buttonValuesRead & B01000000) >> 6;
        event.BTN8 = (bool)(buttonValuesRead & B10000000) >> 7;

        event.MainSwitch = mainSwitchRead & B00000001;
    }

    return event;
}

// Kamera nicht simuliert
camera_event SimuRead::Pixy(void)
{
    camera_event event;
    event.goal_visible = false;
    event.goal_heading = 0.0;
    return event;
}

// Aktuatoren

// Move Funktion simulieren
void SimuAction::move(movement_event event, float PID)
{
    // Parameter
    int PID_Stand_Speed = 70;       // Parameter für Stand-PID-Speed
    float PID_Multiplikator = 0.06; // PID Multiplikator (anpassen nach Bedarf)
    int DribbleSpeed = 100;         // Dribbler Geschwindigkeit
    bool Dribbler_dir = true;       // Dribbler Richtung

    // Calculate Motorspeeds and directions
    // Winkel in x und y Bewegung umrechnen
    float angleRad = event.angle * PI / 180; // Winkel in Radiant umrechnen
    float x = cosf(angleRad) * event.speed;  // x Bewegung
    float y = sinf(angleRad) * event.speed;  // y Bewegung

    // PID Regler berechnen
    // float angleOfAttack = event.AngleOfAttack; // Anstellwinkel fällt raus da ganzes movement event an PID übergeben wird
    float turn = PID * PID_Multiplikator;

    // Movement berechnen
    int Motor0 = (int)(x + turn);                            // Motor 0
    int Motor1 = (int)((-0.5 * x) - (sqrt3 / 2 + y) + turn); // Motor 1
    int Motor2 = (int)((-0.5 * x) + (sqrt3 / 2 + y) + turn); // Motor 2

    // Standbewegung
    if (event.speed == 0)
    {
        Motor0 = (turn + PID_Stand_Speed);
        Motor1 = (turn + PID_Stand_Speed);
        Motor2 = (turn + PID_Stand_Speed);
        event.speed = PID_Stand_Speed;

        // alt
        // Motor0 = (turn * PID_Stand_Multiplier);
        // Motor1 = (turn * PID_Stand_Multiplier);
        // Motor2 = (turn * PID_Stand_Multiplier);
    }

    // Richtungen bercehnen
    // int dir0 = Motor0 > 0 ? 1 : 2;
    // int dir1 = Motor1 > 0 ? 1 : 2;
    // int dir2 = Motor2 > 0 ? 1 : 2;

    // Mapping
    float mapping = 1;

    if (Motor0 > Motor1 && Motor0 > Motor2)
        mapping = event.speed / abs(Motor0);
    else if (Motor1 > Motor0 && Motor1 > Motor2)
        mapping = event.speed / abs(Motor1);
    else
        mapping = event.speed / abs(Motor2);

    Motor0 = Motor0 * mapping;
    Motor1 = Motor1 * mapping;
    Motor2 = Motor2 * mapping;

    // Motoren Setzen
    uint16_t speeds[4] =
        {
            (uint16_t)abs(Motor0 * 256 / 100),
            (uint16_t)abs(Motor1 * 256 / 100),
            (uint16_t)abs(Motor2 * 256 / 100),
            (uint16_t)abs(DribbleSpeed * 256 / 100) // Dribbler immer volle Geschwindigkeit
        };

    bool dirs[4] =
        {
            Motor0 >= 0,
            Motor1 >= 0,
            Motor2 >= 0,
            Dribbler_dir // Dribbler Richtung
        };

    if (!DribblerAktiv) // Wenn dribbler aus Speed auf 0 setzen
        speeds[3] = 0;

    uint8_t ports = 0x00; // ports variable initialisieren
    // ports = 0x00 = b00 00 00 00

    byte motors[3];

    for (int i = 0; i < 3; i++)
    {
        motors[i] = (byte)speeds[i];

        if (dirs[i]) // vorwärts
        {
            /*
            0x01 => b00 00 00 01
            0x01 << 0 = 0000 0001  (1 dezimal)
            0x01 << 2 = 0000 0100  (4 dezimal)
            0x01 << 4 = 0001 0000  (16 dezimal)
            odern schreibt nur in die entsprechenden Bits
            */
            ports |= (0x01 << (i * 2)); // setze die entsprechenden Bits auf 0 1 0x01 = b00 00 00 01
        }
        else // rückwärts
        {
            /*
            0x02 => b00 00 00 10
            0x02 << 0 = 0000 0010  (2 dezimal)
            0x02 << 2 = 0000 1000  (8 dezimal)
            0x02 << 4 = 0010 0000  (32 dezimal)
            odern schreibt nur in die entsprechenden Bits
            */
            ports |= (0x02 << (i * 2)); // setze die entsprechenden Bits auf 1 0 0x02 = b00 00 00 10
        }

        if (speeds[i] == 0)
        {
            /*
            0x03 => b00 00 00 11
            0x03 << (i * 2) verschiebt die Bits entsprechend
            ~(0x03 << (i * 2)) => negiert die Bits (zB b00 00 00 11 wird zu b11 11 11 00)
            das unden setzt die entsprechenden Bits auf 00 (zb b00 10 01 11 wird zu b00 10 00 11 wenn i = 1 ist)
            */
            ports &= ~(0x03 << (i * 2));
        }
    }

    // Dribbler setzen
    ports |= (DribblerAktiv ? (0x01 << (3 * 2)) : 0x00); // Dribbler an/aus // setze die entsprechenden Bits auf 0 1 0x01 => b01 00 00 00

    // Kicker setzen
    if (kicker_extended)
    {
        ports |= 0x80; // Kicker extended Bit 0 auf 1 setzen -> b1000 0000
    }
    else
    {
        ports &= ~0x80; // Kicker extended Bit 0 auf 0 setzen -> b0111 1111
    }

    // Simulate movement command
    telegram t;
    t.signature = 't'; // tell
    t.message[0] = ports;
    t.message[1] = motors[0];
    t.message[2] = motors[1];
    t.message[3] = motors[2];

    sendTelegram(t);
}

// Dribbler an/aus
void SimuAction::dribbler(bool an, bool dir)
{
    DribblerAktiv = an;
}

// Kicker aufladen
void SimuAction::kicker_reset(void)
{
    kicker_extended = false;
}

// Kicker schießen
void SimuAction::kick(void)
{
    kicker_extended = true;
}

// Bremsen simulieren
void SimuAction::brake(void)
{
    // Simulate brake command
    telegram t;
    t.signature = 't';        // tell
    t.message[0] = B00000000; // alle Motoren aus
    t.message[1] = 0;         // Motor 0 Speed
    t.message[2] = 0;         // Motor 1 Speed
    t.message[3] = 0;         // Motor 2 Speed

    sendTelegram(t);
}

// PID Regler simulieren
float SimuAction::calculatePID(compass_sensor_event compass, movement_event movement)
{
    // Parameter
    float PID_P_Multiplier = 0.5;  // P Multiplikator
    float PID_I_Multiplier = 0.01; // I Multiplikator
    float PID_D_Multiplier = 0.2;  // D Multiplikator
    float PIDpa_IlimitMax = 50.0;  // I Wert nach oben begrenzen
    float PIDpa_IlimitMin = -50.0; // I Wert nach unten begrenzen
    float PID_I_threshhold = 0.5;  // I Wert Schwelle um gegen 0 zu gehen

    float PIDKorrektur = compass.orbitDirection - movement.AngleOfAttack; // Abweichung berechnen

    simuPID_I = simuPID_I + PIDKorrektur;

    if (PIDKorrektur > -(PID_I_threshhold) && PIDKorrektur < PID_I_threshhold) // wenn I gegen null geht auf 0 setzen
        simuPID_I = 0;

    if (simuPID_I > PIDpa_IlimitMax) // I nach oben begrenzen
        simuPID_I = PIDpa_IlimitMax;
    else if (simuPID_I < PIDpa_IlimitMin) // I nach unten begrenzen
        simuPID_I = PIDpa_IlimitMin;

    float PID_P = PIDKorrektur * PID_P_Multiplier; // Berechnung P wert

    simuPID_I = simuPID_I * PID_I_Multiplier; // Berechnung I wert

    float PID_D = (PIDKorrektur - simuPIDKorrektur_memory) * PID_D_Multiplier; // Berechnung D wert

    float internPID = PID_P + simuPID_I + PID_D; // PID Wert berechnen
    simuPIDKorrektur_memory = PIDKorrektur;      // PIDKorrektur speichern für D wert
    return internPID;
}