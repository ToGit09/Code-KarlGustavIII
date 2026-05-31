#ifndef ACTION_H
#include "Action.h"
#endif

#ifndef ACTION_CPP
#define ACTION_CPP

RGB_event emptyrgb = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};

float Codeaction::calculatePID(compass_sensor_event compass, movement_event movement)
{
    // VERALTET UND NICHT MEHR BENUTZT
    float PIDKorrektur = compass.orbitDirection - movement.AngleOfAttack; // Abweichung berechnen

    PID_I = PID_I + PIDKorrektur;

    if (PIDKorrektur > -(PID_I_threshhold) && PIDKorrektur < PID_I_threshhold) // wenn Nach vorne ausgerichtet I zurücksetzen
        PID_I = 0;

    if (PID_I > PIDpa_IlimitMax) // I nach oben begrenzen
        PID_I = PIDpa_IlimitMax;
    else if (PID_I < PIDpa_IlimitMin) // I nach unten begrenzen
        PID_I = PIDpa_IlimitMin;

    PID_P = PIDKorrektur * PID_P_Multiplier; // Berechnung P wert

    PID_I = PID_I * PID_I_Multiplier; // Berechnung I wert

    PID_D = (PIDKorrektur - PIDKorrektur_memory) * PID_D_Multiplier; // Berechnung D wert

    float internPID = PID_P + 0 * PID_I + PID_D; // PID Wert berechnen
    PIDKorrektur_memory = PIDKorrektur;          // PIDKorrektur speichern für D wert
    return internPID;
}

float Codeaction::newPID(movement_event movement)
{
    int PID_now = micros();
    float diffTime = float(PID_now - PID_last) / 1000;
    PID_last = PID_now;

    float PIDKorrektur = movement.currentCompassAngle - movement.AngleOfAttack; // Abweichung berechnen

    PID_P = PIDKorrektur * PID_P_Multiplier; // Berechnung P wert

    PID_I += PIDKorrektur * diffTime * PIDpa_Imultiplier;

    if (PID_I > PIDpa_IlimitMax) // I nach oben begrenzen
        PID_I = PIDpa_IlimitMax;
    else if (PID_I < PIDpa_IlimitMin) // I nach unten begrenzen
        PID_I = PIDpa_IlimitMin;

    if (PID_I > 0 && PID_P < 0)
    {
        PID_I = 0;
    } // wenn I falsches Vorzeichen auf null setzen

    if (PID_I < 0 && PID_P > 0)
    {
        PID_I = 0;
    } // wenn I falsches Vorzeichen auf null setzen7

    if ((PIDKorrektur > -3 && PIDKorrektur < 3) && PID_I > 0) // wenn Nach vorne ausgerichtet I zuruecksetzen
        PID_I -= 0.08;

    if ((PIDKorrektur > -3 && PIDKorrektur < 3) && PID_I < 0)
        PID_I += 0.08;

    PID_D = (PIDKorrektur - PIDKorrektur_memory) / diffTime; // Berechnung D wert

    float internPID = PID_P * PID_P_Multiplier + PID_I * PID_I_Multiplier + PID_D * PID_D_Multiplier; // PID Wert berechnen

    // float temp = diffTime;
    /*
    Serial.print(" PID: time: ");
    Serial.print(diffTime);
    Serial.print(" PID: P: ");
    Serial.print(PID_P * PID_P_Multiplier);
    Serial.print(" PID: I: ");
    Serial.print(PID_I * PID_I_Multiplier);
    Serial.print(" PID: D: ");
    Serial.println(PID_D * PID_D_Multiplier);*/

    // internPID = (internPID + 180) / 360; // PID Werte in -1 bis 1 umrechnen

    if (PIDKorrektur > -1 && PIDKorrektur < 1)
        internPID = 0;

    PIDKorrektur_memory = PIDKorrektur; // PIDKorrektur speichern für D wert
    return internPID;
}

void Codeaction::move(movement_event event)
{
    // Winkel in x und y Bewegung umrechnen
    float driveAngleInRadians = (event.angle + event.currentCompassAngle) * PI / 180; // Winkel in Radiant umrechnen
    float y = cosf(driveAngleInRadians) * event.speed;                                // y Bewegung (Hier ist y parallel zum Tor)
    float x = sinf(driveAngleInRadians) * event.speed;                                // x Bewegung

    // PID Regler berechnen
    float turn = newPID(event) * PID_Multiplikator;

    /* Serial.print ("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Turn: ");
    Serial.println(turn);
     */

    int Motor1;
    int Motor0;
    int Motor2;

    // Standbewegung
    if (event.speed == 0)
    {
        Motor0 = (turn * PID_Stand_Multiplier);
        Motor1 = (turn * PID_Stand_Multiplier);
        Motor2 = (turn * PID_Stand_Multiplier);
    }
    else
    {
        turn *= PID_Stand_Multiplier;
        // Movement berechnen
        Motor1 = (int)(x + turn);                            // Hinten
        Motor0 = (int)((-0.5 * x) + (sqrt3 / 2 * y) + turn); // Vorne Links
        Motor2 = (int)((-0.5 * x) - (sqrt3 / 2 * y) + turn); // Vorne Rechts

        // Mapping
        float mapping = 1;

        if (abs(Motor0) > abs(Motor1) && abs(Motor0) > abs(Motor2))
            mapping = event.speed / abs(Motor0);
        else if (abs(Motor1) > abs(Motor0) && abs(Motor1) > abs(Motor2))
            mapping = event.speed / abs(Motor1);
        else
            mapping = event.speed / abs(Motor2);

        Motor0 = Motor0 * mapping;
        Motor1 = Motor1 * mapping;
        Motor2 = Motor2 * mapping;
    }

    Serial.print("Motor0 VL1: ");
    Serial.print(Motor0);
    Serial.print(" Motor1 H2: ");
    Serial.print(Motor1);
    Serial.print(" Motor2 VR3: ");
    Serial.println(Motor2);

    // Motoren setzen
    setMotors(Motor0 * -1, Motor1 * -1, Motor2, 0); // Motor 4 ist nicht angeschlossen also immer aus
}

void Codeaction::setMotors(int Motor1, int Motor2, int Motor3, int Motor4)
{
    int speeds[4] =
        {
            int(roundf(Motor1 * 65535.f / 100.f)),
            int(roundf(Motor2 * 65535.f / 100.f)),
            int(roundf(Motor3 * 65535.f / 100.f)),
            int(roundf(Motor4 * 65535.f / 100.f))};

    for (int i = 0; i < 4; i++)
    {
        if (speeds[i] == 0)
        {
            analogWrite(motoren[i].PWM, 0);
            digitalWrite(motoren[i].IN1, 0);
            digitalWrite(motoren[i].IN2, 0);
        }

        if (speeds[i] > 0) // Motor rechts rum
        {
            analogWrite(motoren[i].PWM, speeds[i]);
            digitalWrite(motoren[i].IN1, 0);
            digitalWrite(motoren[i].IN2, 1);
        }

        if (speeds[i] < 0) // Motor links rum
        {
            analogWrite(motoren[i].PWM, int(speeds[i] * -1)); // abs() macht die Geschwindigkeit positiv
            digitalWrite(motoren[i].IN1, 1);
            digitalWrite(motoren[i].IN2, 0);
        }
    }
}

void Codeaction::brake(void)
{
    setMotors(0, 0, 0, 0);
}

void Codeaction::dribbler(bool an)
{
    // ACHTUNG! Dribbler kann nur eine Richtung, dir wird auf true gesetzt
    if (an)
        // dribbler(DribblerStdSpeed, DribblerStdDir);
        dribbler(DribblerStdSpeed, true);
    else
        // dribbler(0, DribblerStdDir);
        dribbler(0, true);
}

void Codeaction::dribbler(int Speed, bool dir)
{

    dir = true; // ACHTUNG! Dribbler kann nur eine Richtung
    return;     // MACHT AKTUELL NICHTS WEIL NOCH KEIN TREIBER

    int actualSpeed = int(roundf(Speed * 256 / 100));
    analogWrite(33, actualSpeed);

    // Werte als neuen Default speichern
    DribblerStdSpeed = Speed;
    // DribblerStdDir = dir;
    return;

    // Aktuell wird die Richtung nicht genutzt, da der Dribbler nur eine Richtung kann
    {
        if (Speed == 0)
        {
            analogWrite(33, 0); // Zeit sparen und werte nicht speichern
            return;
        }

        if (Speed < 0) // Wenn negativer Speed dann Motor andersrum drehen
            dir = !dir, Speed *= -1;

        if (dir != DribblerStdDir)
        {
            // veraltet -> I2C Kommunikation nur wenn sich richtung ändert -> Zeit sparen

            // Digitalen PIN nutzen
            // digitalWrite(27, dir);
        }
        // Werte als neuen Default speichern
        DribblerStdSpeed = Speed;
        DribblerStdDir = dir;
    }
}

void Codeaction::kicker_reset(void)
{
    if ((int)KickerTimer > (int)KickerExtendTime) // Zeit die der Kicker an ist
    {
        digitalWrite(KICKER_PORT, LOW);
        Serial.print("kicker reset ");
        Serial.println((int)KickerTimer);
    }
}

void Codeaction::kick(void)
{
    if ((int)KickerTimer > (int)KickerExtendTime + KickerCooldown) // Zeit die der Kicker zum Abkühlen hat
    {
        digitalWrite(KICKER_PORT, HIGH);
        KickerTimer = 0;
    }

    Serial.println("kick");
}

void Codeaction::initRGBs(void)
{
    RGBs.begin();
    delay(10);
    RGBs.setBrightness(0);
    RGBs.show();

    // Hochfahr Animation
    RGBs.clear();
    RGBs.setBrightness(LEDBrightness);
    for (int i = 0; i < 19; i++)
    {
        RGBs.setPixelColor(i, 255, 0, 0);
    }
    RGBs.show();

    rgb.brightness = LEDBrightness;
    rgb.length = 19;
    for (int i = 0; i < rgb.length; i++)
    {
        rgb.r[i] = 255;
        rgb.g[i] = 0;
        rgb.b[i] = 0;
    }

    RGBTimer = 0;
}

RGB_event Codeaction::setRGB(int i, int r, int g, int b)
{
    if (i < 0 || i >= LED_LENGTH)
        return rgb;
    if (r >= 0)
        rgb.r[i] = r;
    if (g >= 0)
        rgb.g[i] = g;
    if (b >= 0)
        rgb.b[i] = b;
    return rgb;
}

void Codeaction::renderRGBs(RGB_event RGBinput /* = emptyrgb */)
{
    if (RGBTimer > 1000 / LED_FPS)
    {
        if (RGBinput.length != emptyrgb.length)
        {
            rgb = RGBinput;
        }

        RGBs.clear();
        RGBs.setBrightness(LEDBrightness);

        for (int i = 0; i < rgb.length; i++)
        {
            RGBs.setPixelColor(i, RGBs.Color(rgb.r[i], rgb.g[i], rgb.b[i]));
        }

        RGBs.show();

        RGBTimer = 0;
    }
}

#endif
