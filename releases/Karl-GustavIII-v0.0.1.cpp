/** Port Belegung v4.5 ********************************************************************/
// Pin       Funktion          Verfahren      Angeschlossen
// 0         MOT1_IN1          GPIO           Motor 1 Richtung
// 1         MOT1_IN2          GPIO           Motor 1 Richtung
// 2         MOT2_IN1          GPIO           Motor 2 Richtung
// 3         MOT2_IN2          GPIO           Motor 2 Richtung
// 4         MOT1_PWM          PWM            Motor 1 Geschwindigkeit
// 5         MOT2_PWM          PWM            Motor 2 Geschwindigkeit
// 6         MOT3_PWM          PWM            Motor 3 Geschwindigkeit
// 7         MOT3_IN1          GPIO           Motor 3 Richtung
// 8         MOT3_IN2          GPIO           Motor 3 Richtung
// 9         MOT4_PWM          PWM            Motor 4 Geschwindigkeit
// 10        START_SWITCH      GPIO           Start/Stopp Switch (MAIN SWITCH)
// 11        MOSI              SPI            ???
// 12        MISO              SPI            "
// 13        SCK               SPI            "
// 14        TNS_TX/PIXY_RX    ??             Kamera
// 15        TNS_RX/PIXY_TX    ??             Kamera
// 16        I2C_SCL_3V3       ??             I2C 3V3 (Portexpander Switches)
// 17        I2C_SDA_3V3       ??             I2C 3V3
// 18        Kicker            GPIO           Kicker trigger
// 19        SPI_DIO0          ??             ???
// 20        RGB_DATA_3V3      GPIO           RGB LEDs
// 21        SPI_DIO3          ??             ???
// 22        SPI_DIO4          ??             ???
// 23        LIGHT_BARRIER     GPIO           LDR Input
// --------------------------------------------------------------------------------------- //
// 24        UART1_TNS_TX      UART           ???
// 25        UART1_TNS_RX      UART           ???
// 26        MOT4_IN1          GPIO           Motor 4 Richtung
// 27        EMPTY             -              -
// 24        UART2_TNS_TX      UART           ???
// 25        UART2_TNS_RX      UART           ???
// 30        SPI_DIO2          ??             ???
// 31        SPI_DIO1          ??             ???
// 32        MOT4_IN2          GPIO           Motor 4 Richtung
// 33        BLDC_DRIBBLER_PWM PWM            Brushless Dribbler Geschwindigkeit
// -------------------------------------------------------------------------------------- //
/** Motor Belegung ************************************************************************/
// Motor 0/1 = hinten
// Motor 1/2 = links
// Motor 2/3 = rechts
// Motor 3/4 = Dribbler
/******************************************************************************************/

/** Setup *********************************************************************************/
#include <Arduino.h>
#include <Bot.h> // Bot
// #include <BotSimulation.h> // Bot Simulation

// #define DEBUG // Debug aktivieren
#undef Simulation // Simulation ausschalten

/** Highlevel *****************************************************************************/

// Code für die Taktiken
class CodeTactics
{
public:
#ifdef Simulation
    SimuAction action;
#else
    Codeaction action;
#endif

    // vor und zurück fahren wenn ein Lack of progress erkannt wird
    void LOP(void)
    {
        return; // SICHERUNG

        action.dribbler(true, true);

        // nur rt_ Variablen verwenden
        if (LOPTimer < LOP_TimerLimit + LOP_BackTime)
            move.angle = 180, move.speed = 30, move.AngleOfAttack = 0; // LOP_BackTime millisekunden Rückwärts fahren
        else if (LOPTimer < LOP_TimerLimit + LOP_BackTime + LOP_FrontTime)
            move.angle = 0, move.speed = 100, move.AngleOfAttack = 0; // LOP_FrontTime millisekunden volle kanne vorwärts donnern
        else
            LOPTimer = 0; // LOP Timer zuruecksetzen

        // action.move(move); // wird in game gemacht
    }

    // Ins eigene Tor fahren wenn der Ball nicht gesehen wird
    void homing(void)
    {
        return; // SICHERUNG

        // nur rt_ Variablen verwenden
        action.dribbler(false, true);

        float tmp_offX = US.offsetx;
        float tmp_offY = US.offsety - 40;                                          // US Offset formatieren
        float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                      // Richtung berechnen
        float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeedH; // Speed berechnen

        if (USspeed > stdSpeed - 20) // wenn speed zu groß
            USspeed = stdSpeed - 20; // Speed mappen

        move.angle = sy_dir * USdriveAngle, move.speed = USspeed, move.AngleOfAttack = 0;
        // action.move(move); // wird in game gemacht
    }

    // Den Ball anfahren
    void ballanfahrt(void)
    {
        return; // SICHERUNG
        // nur rt_ Variablen verwenden

        action.dribbler(true, true);

        if (true)
        {
            move.angle = IR.ballanfahrt, move.speed = stdSpeed, move.AngleOfAttack = 0;
            // action.move(move); // wird in game gemacht
            return;
        }

        // Alte Ballanfahrtsfunktion

        if (IR.heading == 4) // genau hinter mir
        {
            if (US.offsetx > 0)
                move.angle = 125, move.speed = stdSpeed, move.AngleOfAttack = 0; // seitlich nach hinten fahren
            else
                move.angle = -125, move.speed = stdSpeed, move.AngleOfAttack = 0; // seitlich nach hinten fahren
        }
        else if (IR.heading == 3)                                             // schräg hinter mir
            move.angle = 180, move.speed = stdSpeed, move.AngleOfAttack = 0;  // Schräg nach hinten fahren
        else if (IR.heading == 5)                                             // schräg hinter mir
            move.angle = -180, move.speed = stdSpeed, move.AngleOfAttack = 0; // schräg nach hinten fahren
        else if (IR.heading == 2)                                             // neben mir
            move.angle = 150, move.speed = stdSpeed, move.AngleOfAttack = 0;  // fast seitlich fahren
        else if (IR.heading == 6)                                             // neben mir
            move.angle = -150, move.speed = stdSpeed, move.AngleOfAttack = 0; // fast seitlich fahren
        else if (IR.heading == 1)                                             // vor und neben mir
            move.angle = 80, move.speed = stdSpeed, move.AngleOfAttack = 0;   // seitlich nach vorne fahren (mit offset)
        else if (IR.heading == 7)                                             // vor und neben mir
            move.angle = -80, move.speed = stdSpeed, move.AngleOfAttack = 0;  // seitlich nach vorne fahren (mit offset)
        else                                                                  // Ball vor mir (0)
            move.angle = 0, move.speed = stdSpeed, move.AngleOfAttack = 0;

        // action.move(move); // wird in game gemacht
    }

    // Mit dem Ball ins Tor fahren / Schiessen
    // @param PixyDrive true = nach Pixy fahren, false = nach Kompass und US fahren
    void toranfahrt(bool PixyDrive)
    {
        return; // SICHERUNG
        action.dribbler(true, true);

        if (PixyDrive) // Nach Pixy fahren
        {
            if (cam.goal_heading == 0 && Pixy_GoalDist > PixyDistSchwelle)
            {
                action.dribbler(true, false);
                action.kick();
            }
            else if (cam.goal_heading == 0 && Pixy_GoalDist < PixyDistSchwelle)
                move.angle = 0, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = 90 * cam.goal_heading, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }
        else // Nach Kompass und US fahren
        {
            if (Compass.orbitDirection >= GoalTargetAngel - 2 && Compass.orbitDirection <= GoalTargetAngel + 2) // wenn ausgerichtet (entweder auf 0 oder aufs Torfür kicker vorbereitung)
            {
                if (US.dist_f > 35) // Wenn weiter weg vom Tor (35 + Zentimeter) ins Tor kicken
                {
                    float tmp_offX = US.offsetx;
                    float tmp_offY = US.offsety + 35;                     // US Offset formatieren
                    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI; // TorRichtung berechnen

                    GoalTargetAngel = sy_dir; // neue GoalTargetDir setzen

                    if (Compass.orbitDirection >= GoalTargetAngel - 5 && Compass.orbitDirection <= GoalTargetAngel + 5) // wenn zum Tor gedreht
                        action.kick();                                                                                  // kicken
                }
                else if (US.dist_f > 10) // Wenn kurz vor Tor (35 - 10 Zentimeter) ins Tor fahren
                {
                    float tmp_offX = US.offsetx;
                    float tmp_offY = US.offsety + 35;                                         // US Offset formatieren
                    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                     // Richtung berechnen
                    float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeed; // Speed berechnen
                    if (USspeed > stdSpeed)                                                   // wenn speed zu groß
                        USspeed = stdSpeed;                                                   // Speed mappen
                    move.angle = sy_dir * USdriveAngle, move.speed = USspeed, move.AngleOfAttack = 0;
                }
                else // Wenn in Ecke oder ganz nah vor dem Gegnerischen Roboter nach Rechts oder links fahren
                {
                    if (US.offsetx > 0)                                                   // rechts
                        move.angle = 95, move.speed = stdSpeed, move.AngleOfAttack = -10; // nach Links fahren
                    else                                                                  // Links
                        move.angle = -95, move.speed = stdSpeed, move.AngleOfAttack = 10; // nach Rechts fahren
                }
            }
            else                                                                      // wenn nicht ausgerichtet
                move.angle = 0, move.speed = 0, move.AngleOfAttack = GoalTargetAngel; // drehen
        }

        // action.move(move); // wird in game gemacht
    }
};

// Der Main Code für den Roboter
class CodeRobot
{
private:
    // events
    ir_sensor_event IR;
    ldr_sensor_event LDR;
    compass_sensor_event Compass;
    us_sensor_event US;
    switches_event switches;
    camera_event cam;
    movement_event move;

    // Timer
    elapsedMillis ReadTimer;     // Read Timer
    elapsedMillis DribblerTimer; // Dribbler Timer
    elapsedMillis LEDTimer;      // RGB Timer
    elapsedMillis totalTime;     // Insgesamte Zeit die verstrichen ist
    elapsedMillis LoopTime;      // Zeit die ein durchlauf braucht
    elapsedMillis UpdateTime;    // Zeit die ein Update durchlauf braucht
    elapsedMillis LoopTimer;     // immer hochzählen bis zehn oder so und dann starten
    elapsedMillis LOPTimer;      // Lack of progress Timer
    elapsedMillis InitTime;      // Zeit die ein initialize durchlauf braucht
    elapsedMillis PIDTimer;      // PID Timer

    // Timings
    int InitT = -1;

public:
#ifdef Simulation
    SimuRead Read;
#else
    CodeRead Read;
#endif

    CodeTactics Tactics;

    // Initialisierung des Roboters
    void initialize(void)
    {
        InitTime = 0; // InitTimer zurücksetzen

#ifdef Simulation
        Read.init();
        return; // In der Simulation nichts initialisieren
#endif

        //-Serial---------------------------------------------------------------------------------//
        Serial.begin(9600);
        //----------------------------------------------------------------------------------------//

        //-PinModes-------------------------------------------------------------------------------//
        pinMode(0, OUTPUT); // MOT1_IN1
        pinMode(1, OUTPUT); // MOT1_IN2
        pinMode(2, OUTPUT); // MOT2_IN1
        pinMode(3, OUTPUT); // MOT2_IN2
        pinMode(4, OUTPUT); // MOT1_PWM
        pinMode(5, OUTPUT); // MOT2_PWM
        pinMode(6, OUTPUT); // MOT3_PWM
        pinMode(7, OUTPUT); // MOT3_IN1
        pinMode(8, OUTPUT); // MOT3_IN2
        pinMode(9, OUTPUT); // MOT4_PWM
        pinMode(10, INPUT); // START_SWITCH
        // pinMode(11, OUTPUT); // MOSI
        // pinMode(12, INPUT);  // MISO
        // pinMode(13, OUTPUT); // SCK
        // pinMode(14, OUTPUT); // TNS_TX/PIXY_RX
        // pinMode(15, INPUT);  // TNS_RX/PIXY_TX
        // pinMode(16, OUTPUT); // I2C_SCL_3V3
        // pinMode(17, OUTPUT); // I2C_SDA_3V3
        pinMode(18, OUTPUT); // Kicker
        pinMode(19, OUTPUT); // SPI_DIO0
        // pinMode(20, OUTPUT); // RGB_DATA_3V3
        pinMode(21, OUTPUT); // SPI_DIO3
        pinMode(22, OUTPUT); // SPI_DIO4
        pinMode(23, INPUT);  // LIGHT_BARRIER
        // pinMode(24, OUTPUT); // UART1_TNS_TX
        // pinMode(25, INPUT);  // UART1_TNS_RX
        pinMode(26, OUTPUT); // MOT4_IN1
        // pinMode(27, INPUT);  // EMPTY
        // pinMode(28, OUTPUT); // UART2_TNS_TX
        // pinMode(29, INPUT);  // UART2_TNS_RX
        pinMode(30, OUTPUT); // SPI_DIO2
        pinMode(31, OUTPUT); // SPI_DIO1
        pinMode(32, OUTPUT); // MOT4_IN2
        pinMode(33, OUTPUT); // BLDC_DRIBBLER_PWM

        //----------------------------------------------------------------------------------------//

        //-IR-Ring--------------------------------------------------------------------------------//
        I2C_BUS.begin();
        I2C_BUS.setClock(1000000);
        //----------------------------------------------------------------------------------------//

        //-RGB LEDs-------------------------------------------------------------------------------//
        // INIT RGBS
        Tactics.action.initRGBs();
        //----------------------------------------------------------------------------------------//

        //-Expander-------------------------------------------------------------------------------//
        /* pin 18,19: I2C to port expanders (switches)                                            */
        Wire1.begin();
        Wire1.beginTransmission(0x20);
        Wire1.write(0x03);
        Wire1.write(0xFF);
        Wire1.endTransmission();
        Wire1.beginTransmission(0x20);
        Wire1.write(0x01);
        Wire1.write(0x00);
        Wire1.endTransmission();

        //----------------------------------------------------------------------------------------//

        //-BNO------------------------------------------------------------------------------------//
        BNO.begin();
        BNO.setExtCrystalUse(true);
        //----------------------------------------------------------------------------------------//

        //-SPI------------------------------------------------------------------------------------//
        /* pin 9-13 SPI with ADC chip select*/
        SPI.begin();
        //----------------------------------------------------------------------------------------//

        //-Pixy2----------------------------------------------------------------------------------//
        // pixy.init();
        // pixy.setLamp(0, 0);   // turn off the lamp
        // pixy.setLED(0, 0, 0); // turn off the LED
        // pixy.setCameraBrightness(91);
        //----------------------------------------------------------------------------------------//

        //-US-------------------------------------------------------------------------------------//
        int idx;                  // zählt Sensoren durch
        int USnum = 0;            // Anzahl der gefundenen Sensoren
        int sensorAddress = 0x70; // I2C Adresse des aktuellen Sensors
        unsigned char rev;        // software revision of the sensor

        // Programm akkzeptiert Sensoren mit der Addresse 0x70 bis 0x77
        for (idx = 0; idx < 8; idx++) // Alle US Sensoren anfragen
        {
            Tactics.action.setRGB(1, 0, 0, 255);
            Tactics.action.renderRGBs();

            /* check for sensor presence by reading its software version */
            Wire1.beginTransmission(sensorAddress);
            Wire1.write(0x00);                   // select version register
            Wire1.endTransmission(false);        // switch to read direction
            Wire1.requestFrom(sensorAddress, 1); // Start a read access and expect one byte
            rev = Wire1.read();                  // get software version
            Wire1.endTransmission();

            if (rev != 0xFF) // anything that is not FF is a present sensor
            {
                Serial.print("Sensor found at ");
                Serial.print(sensorAddress, HEX);
                US_address[USnum] = sensorAddress;
                USnum++;

                Tactics.action.setRGB(1, 0, 255, 0);
            }
            else
            {
                Serial.print("No sensor at ");
                Serial.print(sensorAddress, HEX);
                Serial.println("h");

                Tactics.action.setRGB(1, 255, 0, 0);
            }

            Tactics.action.renderRGBs();

            delay(50);       // wait a bit before next sensor
            sensorAddress++; // goto next sensor
        }
        //----------------------------------------------------------------------------------------//

        //-LDR------------------------------------------------------------------------------------//
        pinMode(24, INPUT);
        //----------------------------------------------------------------------------------------//

        //-Motor PWM------------------------------------------------------------------------------//
        analogWriteFrequency(motor1.PWM, motor1.frequenz);
        analogWriteFrequency(motor2.PWM, motor2.frequenz);
        analogWriteFrequency(motor3.PWM, motor3.frequenz);
        analogWriteFrequency(motor4.PWM, motor4.frequenz);
        //----------------------------------------------------------------------------------------//

        //-Variablen------------------------------------------------------------------------------//
        Tactics.action.sqrt3 = sqrtf(3);
        //----------------------------------------------------------------------------------------//

        InitT = InitTime; // Initialisierungszeit speichern

        //-Optik----------------------------------------------------------------------------------//
        Serial.print(InitTime);

        for (int i = 0; i < 3; i++)
        {
            Tactics.action.setRGB(1, 0, 255, 0);
            Tactics.action.renderRGBs();
            delay(50);
        }

        delay(100);
        //----------------------------------------------------------------------------------------//

        //-Timing---------------------------------------------------------------------------------//
        ReadTimer = 0;
        // KickerTimer = 0;
        LoopTime = 0;
        UpdateTime = 0;
        LoopTimer = 0;
        LOPTimer = 0;
        //----------------------------------------------------------------------------------------//
    }

    // Liest alle Sensoren aus und zeigt die Werte auf dem Display an
    void update(void)
    {
        UpdateTime = 0;
        // Serial.println("update");

        Compass = Read.Compass();
        // IR = Read.IR(Compass);
        LDR = Read.LDR();
        // cam = Read.Pixy(); // keine Pixy angeschlossen
        // PID = Tactics.action.calculatePID(Compass, move); // unnötig geworden
        // Tactics.action.kicker_reset();

        if (ReadTimer % 20 >= 15)
        {
            IR = Read.IR(Compass);
        }

        if (ReadTimer % 50 >= 45)
        {
            switches = Read.Switches();
        }

        if (ReadTimer >= 110)
        {
            US = Read.US(Compass); // keine US angeschlossen
            ReadTimer = 0;
        }

        lastUpdateTime = UpdateTime;
    }

    // Handelt das Timing der Game Funktion und speichert alle sensor Werte
    void system(void)
    {
        if (LoopTimer >= LoopTiming) // Alle 10 ms Ausführen
        {
            // Serial.println("alive");
            LoopError = LoopTimer - LoopTiming; // Fehler im Loop Timer
            LoopTime = 0;                       // Timer der die Länge des Loops misst

            Tactics.action.kicker_reset();

            // Mainswitch aktualisieren um Schneller handeln zu können
            switches.MainSwitch = digitalRead(MAINSWITCH_PORT);

            if (switches.MainSwitch)
            {
                Tactics.action.setRGB(0, 0, 255, 0); // Start lever State

                // Main Switch = Spiel
                // game(); // führt aktuell noch nichts aus

                if (switches.BTN1)
                    Read.KompassCliValue = Compass.orbitDirection + Read.KompassCliValue;

                /*Serial.print(" ");
                Serial.print(US.dist_r);
                Serial.print(" : ");
                Serial.print(US.dist_l);
                Serial.print(" : ");
                Serial.print(US.dist_f);
                Serial.print(" : ");
                Serial.print(US.dist_b);
                Serial.print(" ; ");*/

                // Tactics.action.setMotors( 50, 0, -50, false);
                if (switches.SWI1)
                    move.speed = 0;
                else
                    move.speed = 30;

                if (switches.SWI2)
                    move.angle = 0;
                else
                    move.angle = 90;

                if (switches.SWI3)
                    move.AngleOfAttack = 0;
                else
                    move.AngleOfAttack = 45;

                if (switches.BTN2)
                    Tactics.action.kick();

                // move.speed = 30;

                // move.AngleOfAttack = 0;

                /*Serial.print("Move: ");
                Serial.print(move.speed);
                Serial.print(" : ");
                Serial.print(move.angle);
                Serial.print(" : ");
                Serial.print(move.AngleOfAttack);
                Serial.print(" : ");*/

                // Serial.println(Compass.orbitDirection);

                move.currentCompassAngle = Compass.orbitDirection;
                move.angle = 0*IR.Orbit_direction * -1;
                move.AngleOfAttack = 0;
                move.speed = 30;

                //Tactics.action.move(move);

                Serial.print("LDR: ");
                Serial.print(LDR.ballda);
                Serial.print(" : ");
                Serial.println(LDR.value);

                // analogWrite(motoren[1].PWM, 200);

                // Serial.println(IR.direction);
            }
            else
            {
                Tactics.action.setRGB(0, 0, 0, 255); // Lever State

                if (switches.BTN1)
                    Read.KompassCliValue = Compass.orbitDirection + Read.KompassCliValue;

                if (switches.BTN2)
                    Tactics.action.kick();

                // Serial.print("Kompass: ");
                // Serial.println(Compass.orbitDirection);

                Serial.println("Debug");

                Tactics.action.brake();

                // Serial.print("IR: ");
                // Serial.print(IR.distance);
                // Serial.print(" : ");
                // Serial.println(IR.direction);
            }

            { // RGBS Aktualisieren
                int r = 0, g = 0, b = 0;

                if (switches.SWI1)
                    r = 255;
                else
                    r = 0;

                if (switches.SWI2)
                    g = 255;
                else
                    g = 0;

                if (switches.SWI3)
                    b = 255;
                else
                    b = 0;

                Tactics.action.setRGB(2, r, g, b); // Lever states

                // Calibration anzeigen
                if (Compass.calibrationMag == -1)
                    Tactics.action.setRGB(1, 0, 0, 255);
                else if (Compass.calibrationMag == 0)
                    Tactics.action.setRGB(1, 255, 0, 0);
                else if (Compass.calibrationMag == 1)
                    Tactics.action.setRGB(1, 255, 70, 0);
                else if (Compass.calibrationMag == 2)
                    Tactics.action.setRGB(1, 255, 255, 30);
                else if (Compass.calibrationMag == 3)
                    Tactics.action.setRGB(1, 0, 255, 0);

                Tactics.action.renderRGBs();

            } // RGBs

            lastLoopTime = LoopTime; // letzte Loop Zeit speichern
            LoopTimer = 0;           // System Timing zurücksetzen
        }
    }

    // Verarbeitet die wichtigsten Sensor Werte und entscheidet über die Taktiken
    void game(void)
    {
        return; // SICHERUNG

        bool hasBall = (LDR.ballda && IR.heading == 0 && IR.reliable); // LDR und IR Sensor sehen den Ball

        if (LOPTimer >= LOP_TimerLimit)
            Tactics.LOP();
        else if (cam.goal_visible && hasBall) // Wenn die Pixy dass Tor sieht und der Ball da ist
            Tactics.toranfahrt(true);
        else if (hasBall) // LDR und IR Sensor sehen den Ball
            Tactics.toranfahrt(false);
        else if (IR.reliable) // IR Sensor sieht den Ball
        {
            GoalTargetAngel = 0; // Toranfahrt zurücksetzen
            LOPTimer = 0;        // LOP Timer zuruecksetzen
            Tactics.ballanfahrt();
        }
        else
            Tactics.homing();

        move.currentCompassAngle = Compass.orbitDirection;
        Tactics.action.move(move);
    }
};

/** Main **********************************************************************************/
CodeRobot Karl_GustavIII;

// alle Taktiken sind noch verplombt
// erst alle Sensorwerte ordentlich durchchecken
// dann display ordentlich durchchecken
// und dann die Taktiken einzeln via Debug Switch ordentlich durchchecken.
// dann den Gameloop testen

IntervalTimer InteruptTimer; // 5ms, Robot::update

void InteruptTimerFunc(void)
{
    Karl_GustavIII.update();
}

void setup()
{
    Karl_GustavIII.initialize();
    InteruptTimer.begin(InteruptTimerFunc, 5 * 1000);
    Serial.println("Setup done");
}

void loop()
{
    // analogWrite(6, 255);
    // digitalWrite(8, LOW);
    // digitalWrite(7, HIGH);
    // digitalWrite(0,1);
    // digitalWrite(1,0);
    Karl_GustavIII.system();
}
/******************************************************************************************/