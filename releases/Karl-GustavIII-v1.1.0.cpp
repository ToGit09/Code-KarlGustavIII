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
// 11        MOSI              SPI            SPI-Intercon
// 12        MISO              SPI            SPI-Intercon
// 13        SCK               SPI            SPI-Intercon
// 14        TNS_TX/PIXY_RX    Serial         Kamera
// 15        TNS_RX/PIXY_TX    Serial         Kamera
// 16        I2C_SCL_3V3       I2C            I2C 3V3 (Portexpander Switches)
// 17        I2C_SDA_3V3       I2C            I2C 3V3
// 18        Kicker            GPIO           Kicker trigger
// 19        SPI_DIO0          SPI/GPIO       SPI-Intercon
// 20        RGB_DATA_3V3      GPIO           RGB LEDs
// 21        SPI_DIO3          SPI/GPIO       SPI-Intercon
// 22        SPI_DIO4          SPI/GPIO       SPI-Intercon
// 23        LIGHT_BARRIER     GPIO           LDR Input
// --------------------------------------------------------------------------------------- //
// 24        UART1_TNS_TX      UART           Kommunikation mit Teensy via UART1
// 25        UART1_TNS_RX      UART           Kommunikation mit Teensy via UART1
// 26        MOT4_IN1          GPIO           Motor 4 Richtung
// 27        EMPTY             -              LEER / evtl Dribbler DIR
// 24        UART2_TNS_TX      UART           Kommunikation mit Teensy via UART2
// 25        UART2_TNS_RX      UART           Kommunikation mit Teensy via UART2
// 30        SPI_DIO2          SPI/GPIO       SPI-Intercon
// 31        SPI_DIO1          SPI/GPIO       SPI-Intercon
// 32        MOT4_IN2          GPIO           Motor 4 Richtung
// 33        BLDC_DRIBBLER_PWM PWM            Brushless Dribbler Geschwindigkeit
// -------------------------------------------------------------------------------------- //
/** Motor Belegung ************************************************************************/
// Motor 0/1 = hinten
// Motor 1/2 = links
// Motor 2/3 = rechts
// Motor 3/4 = leer
/******************************************************************************************/

/** Setup *********************************************************************************/
#include <Arduino.h>
#include <Bot.h> // Bot
// #include <BotSimulation.h> // Bot Simulation

// #define DEBUG // Debug aktivieren
#undef Simulation // Simulation ausschalten

/** Highlevel *****************************************************************************/

// Code für die Taktiken
// VARIABLEN SIND NICHT DIE GLEICHEN WIE IM HAUPTKLASSENCODE
class CodeTactics
{
public:
#ifdef Simulation
    SimuAction action;
#else
    Codeaction action;
#endif
    elapsedMillis LOPTimer; // Lack of progress Timer
    int stdSpeed = 50;

    // vor und zurück fahren wenn ein Lack of progress erkannt wird
    // @brief nicht genutzt
    movement_event resolveLOP(movement_event event = movement_event())
    {
        return event; // SICHERUNG

        // action.dribbler(true, true);

        // nur rt_ Variablen verwenden
        if (LOPTimer < LOP_TimerLimit + LOP_BackTime)
            event.angle = 180, event.speed = 30, event.AngleOfAttack = 0; // LOP_BackTime millisekunden Rückwärts fahren
        else if (LOPTimer < LOP_TimerLimit + LOP_BackTime + LOP_FrontTime)
            event.angle = 0, event.speed = 100, event.AngleOfAttack = 0; // LOP_FrontTime millisekunden volle kanne vorwärts donnern
        else
            LOPTimer = 0; // LOP Timer zuruecksetzen

        // action.move(move); // wird in game gemacht
    }

    // Ins eigene Tor fahren wenn der Ball nicht gesehen wird
    // @brief nicht genutzt
    movement_event homing(us_sensor_event US, movement_event move = movement_event())
    {
        return move; // SICHERUNG

        // nur rt_ Variablen verwenden
        action.dribbler(false, true);

        float tmp_offX = US.offsetx;
        float tmp_offY = US.offsety - 40;                                          // US Offset formatieren
        float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;                      // Richtung berechnen
        float USspeed = (abs(tmp_offX) + abs(tmp_offY)) / USdivider + USminSpeedH; // Speed berechnen

        // Speed sollte langsam sein also Maximal Standardspeed - 30 (Langsames fahren vors eigene Tor...)
        if (USspeed > stdSpeed - 30) // wenn speed zu groß
            USspeed = stdSpeed - 30; // Speed mappen

        move.angle = sy_dir * USdriveAngle, move.speed = USspeed, move.AngleOfAttack = 0, move.dribbler = false;

        return move;
    }

    movement_event defend(ir_sensor_event IR_Data, movement_event move = movement_event())
    {
        if (IR_Data.Orbit_direction > 0) // links
            move.angle = -105, move.speed = 100, move.AngleOfAttack = 0;
        else // rechts
            move.angle = 105, move.speed = 100, move.AngleOfAttack = 0;

        return move;
    }

    /**
     * @brief Ball anfahren wenn der Ball gesehen wird
     * @param IR_Data: Der IR sensor Event
     * @param Cornerspeed: Wenn true, wird die Geschwindigkeit an die Corner speed angepasst
     * @param move: Der Movement Event
     * @return Der Movement Event
     *
     * Wenn Cornerspeed == true, wird die Geschwindigkeit an die Corner speed angepasst.
     * Wenn der Ball gesehen wird, wird nach dem Ball gefahren.
     * Wenn der Ball nicht gesehen wird, wird nach Pixy gefahren.
     */
    movement_event ballanfahrt(ir_sensor_event IR_Data, bool Cornerspeed, bool leftcorner, movement_event move = movement_event())
    {

        move.angle = IR_Data.ballanfahrt_neuW, move.AngleOfAttack = 0;

        // in einer Ecke geschwindigkeit anpassen
        if (Cornerspeed)
        {
            move.speed = 30; // Geschwindigkeit begrenzen

            if (leftcorner) // in die Ecke drehen
                move.AngleOfAttack = -15;
            else
                move.AngleOfAttack = 15;
        }
        else
            move.speed = stdSpeed * IR_Data.ballanfahrt_neuSF; // Standardgeschwindigkeit

        // Angle of attack setzen wenn der Ball vor einem ist
        if (IR_Data.Orbit_direction > -35 && IR_Data.Orbit_direction < 35)
            move.AngleOfAttack = IR_Data.Orbit_direction;
        else
            move.AngleOfAttack = 0;

        // Debug
        Serial.print("Ball anfahren : ");
        Serial.print(IR_Data.Orbit_direction);
        Serial.print(" | ");
        Serial.print(move.angle);
        Serial.print(" | ");
        Serial.print(move.speed);
        Serial.print(" | ");
        Serial.println(move.AngleOfAttack);

        return move;
    }

    /**
     * Corner movement function
     *
     * @param isItLeftCorner: Is it the left corner?
     * @param cornerTimerState: The state of the corner timer
     * @param US: The US sensor event
     * @param move: The movement event to be modified
     * @return The modified movement event
     *
     * This function returns a movement event that makes the robot turn left or right depending on the value of isItLeftCorner.
     * If cornerTimerState is greater than or equal to 2000, the speed of the robot will be set to 90.
     * If cornerTimerState is greater than or equal to 4000, the speed of the robot will be set to 100.
     * If the distance in front of the robot is greater than 10, the angle will be set to 0 and the robot will kick the ball.
     */
    movement_event corner(bool isItLeftCorner, int cornerTimerState, bool isCornerUnset, us_sensor_event US, compass_sensor_event comp, movement_event move = movement_event())
    {
        if (isCornerUnset)
        {
            move = toranfahrt(false, US, comp, camera_event(), move);
            return move;
        }

        if (isItLeftCorner)
        {
            move.angle = -60, move.speed = stdSpeed, move.AngleOfAttack = 15; // fast ganz nach rechts fahren
            Serial.println("Left Corner");
        }

        else
        {
            move.angle = 60, move.speed = stdSpeed, move.AngleOfAttack = -25; // fast ganz nach links fahren
            Serial.println("Right Corner");
        }

        // if (cornerTimerState <= 200) // 0.5 sekunden nach hinten fahren um ballsperre zu lösen -> danach langsame ballsuche ! evtl zeit runtersetzen
        // move.angle = 180,
        // move.speed = 30;

        if (cornerTimerState >= 2000) // nach 2 sekunden schneller werden
            move.speed = 90;

        if (cornerTimerState >= 4000) // nach 4 sekunden noch schneller werden
            move.speed = 100;

        if (US.dist_f > 20) // Wenn es vorne Platz hat schießen und vorwärts fahren
        {
            move.angle = 0;
            action.kick();
        }

        Serial.print(" ECKE : WINKEL = ");
        Serial.print(move.angle);
        Serial.print(" AOT = ");
        Serial.println(move.AngleOfAttack);

        return move;
    }

    /**
     * @brief Ball anfahren wenn der Ball gesehen wird
     * @param PixyDrive: Ob der Ball nach Pixy fahren soll
     * @param US: Der US sensor Event
     * @param Compass: Der Kompass Event
     * @param cam: Der Kamera Event
     * @param move: Der Movement Event
     * @return Der Movement Event
     *
     * Wenn PixyDrive == true, wird nach Pixy gefahren.
     * Wenn PixyDrive == false, wird nach Kompass und US gefahren.
     */
    movement_event toranfahrt(bool PixyDrive, us_sensor_event US, compass_sensor_event Compass, camera_event cam, movement_event move = movement_event())
    {
        if (PixyDrive)
            ;
        else
        {
            if (US.dist_f > 35 || US.dist_b < 50) // Platz nach vorne zum schiessen
            {
                Serial.print("Platz nach vorne: ");
                if (US.offsetx < 8 && US.offsetx > -8) // wenn vorm Tor
                {
                    Serial.println("kicken ");
                    action.kick();
                    move.angle = 0, move.speed = stdSpeed, move.AngleOfAttack = 0;
                }
                else // Nicht vorm Tor
                {
                    if (US.offsetx > 0)
                        move.angle = -35, move.speed = stdSpeed, move.AngleOfAttack = 0;
                    else
                        move.angle = 35, move.speed = stdSpeed, move.AngleOfAttack = 0;

                    Serial.print("fahren im Winkel ");
                    Serial.println(move.angle);
                }
            }
            else // kein Platz nach vorne zum Tor fahren
            {
                Serial.print("Kein Platz nach vorne: ");
                if (US.offsetx < 8 && US.offsetx > -8) // wenn vorm Tor
                {
                    Serial.println("gerade aus fahren ");
                    move.angle = 0, move.speed = stdSpeed, move.AngleOfAttack = 0;
                    action.kick();
                }
                else
                {
                    float tmp_offX = US.offsetx;
                    float tmp_offY = US.offsety + 35;                     // US Offset formatieren
                    float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI; // TorRichtung berechnen

                    move.angle = sy_dir * USdriveAngle * -1, move.speed = stdSpeed, move.AngleOfAttack = 0;
                    Serial.print("fahren im Winkel ");
                    Serial.println(sy_dir * USdriveAngle * -1);
                }
            }
        }

        return move;
        /* action.dribbler(true, true);

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
 */
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
    elapsedMillis ReadTimer2;    // Read Timer
    elapsedMillis DribblerTimer; // Dribbler Timer
    elapsedMillis LEDTimer;      // RGB Timer
    elapsedMillis totalTime;     // Insgesamte Zeit die verstrichen ist
    elapsedMillis LoopTime;      // Zeit die ein durchlauf braucht
    elapsedMillis UpdateTime;    // Zeit die ein Update durchlauf braucht
    elapsedMillis LoopTimer;     // immer hochzählen bis zehn oder so und dann starten
    elapsedMillis InitTime;      // Zeit die ein initialize durchlauf braucht
    elapsedMillis PIDTimer;      // PID Timer
    elapsedMillis CornerTimer;   // Corner Timer

    // Timings
    int InitT = -1;
    bool isItLeftCorner = false;
    bool cornerisUnset = true;
    int StatusR = 0;
    int StatusG = 0;
    int StatusB = 0;

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
        // Read.IR_Ring_Lib.init();
        IRh.init();
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
        Tactics.LOPTimer = 0;
        CornerTimer = 10000;
        //----------------------------------------------------------------------------------------//
    }

    // Liest alle Sensoren aus und zeigt die Werte auf dem Display an
    void update(void)
    {
        UpdateTime = 0;

        Compass = Read.Compass();
        IR = Read.IR(Compass, US);
        LDR = Read.LDR();
        // cam = Read.Pixy(); // keine Pixy angeschlossen

        if (ReadTimer >= 100)
        {
            switches = Read.Switches();
            ReadTimer = 0;
        }

        if (ReadTimer2 >= 73) // BIN AUF 80 RUNTER war 110
        {
            US = Read.US(Compass);
            ReadTimer2 = 0;
        }

        lastUpdateTime = UpdateTime;

        /* Serial.print(" Sensorwerte Aktualisiert in : ");
        Serial.println(UpdateTime); */
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
            update(); // Aktualisiert alle Sensoren

            // Mainswitch aktualisieren um Schneller handeln zu können
            switches.MainSwitch = digitalRead(MAINSWITCH_PORT);

            if (switches.MainSwitch)
            {
                Tactics.action.setRGB(0, StatusR, StatusG, StatusB); // Start lever State

                // manual Input Handling
                if (switches.BTN1)
                    Read.KompassCliValue = Compass.orbitDirection + Read.KompassCliValue, Read.calibrateLDR();

                if (switches.BTN2)
                    Tactics.action.kick();

                if (switches.BTN3)
                    Read.calibrateLDR();

                if (switches.BTN4)
                    ;

                // Switches
                if (switches.SWI1) // Speeds
                    Tactics.stdSpeed = 80;
                else
                    Tactics.stdSpeed = 94;

                if (switches.SWI2)
                    ;
                else
                    ;

                if (switches.SWI3)
                    KickerExtendTime = 50;
                else
                    KickerExtendTime = 15;

                // Fahr logik

                gameLogik(); // führt aktuell noch nichts aus

                // --- Debug stuff -------------------------------------------------------------------------//

                // move.speed = 30;

                // move.AngleOfAttack = 0;

                /*Serial.print("Move: ");
                Serial.print(move.speed);
                Serial.print(" : ");
                Serial.print(move.angle);
                Serial.print(" : ");
                Serial.print(move.AngleOfAttack);
                Serial.print(" : ");*/

                /*Serial.print(" ");
                Serial.print(US.dist_r);
                Serial.print(" : ");
                Serial.print(US.dist_l);
                Serial.print(" : ");
                Serial.print(US.dist_f);
                Serial.print(" : ");
                Serial.print(US.dist_b);
                Serial.print(" ; ");*/

                // Serial.println(Compass.orbitDirection);

                /* move.currentCompassAngle = Compass.orbitDirection;
                move.angle = IR.ballanfahrt;
                move.AngleOfAttack = 0; // IR.Orbit_direction;
                move.speed = Tactics.stdSpeed;

                Tactics.action.move(move); */

                /* Serial.print("LDR: ");
                Serial.print(LDR.ballda);
                Serial.print(" : ");
                Serial.println(LDR.value); */

                // analogWrite(motoren[1].PWM, 200);

                // Serial.println(IR.direction);

                /* Serial.print(" US : ");
                Serial.print(US.dist_f);
                Serial.print(" : ");
                Serial.println(US.dist_b);
                Serial.print(" x: ");
                Serial.print(US.offsetx); */
            }
            else
            {
                Tactics.action.setRGB(0, 0, 0, 255); // Lever State
                Tactics.LOPTimer = 0;                // Damit sich der LOP Timer nicht schon vor dem Spiel hochschraubt....
                CornerTimer = 10000;

                if (switches.BTN1)
                    Read.KompassCliValue = Compass.orbitDirection + Read.KompassCliValue;

                if (switches.BTN2)
                    Tactics.action.kick();

                if (switches.BTN3)
                    Read.calibrateLDR();

                if (switches.BTN4)
                    Tactics.action.dribbler(true);
                else
                    Tactics.action.dribbler(false);

                Serial.print("Kompass: ");
                Serial.println(Compass.orbitDirection);

                // Serial.println("Debug");

                Tactics.action.brake();

                /* Serial.print("IR: ");
                Serial.print(IRh.Distance_raw);
                Serial.print(",  Angle : ");
                Serial.println(IRh.Angle);   */

                /* Serial.print(" IR : ");
                Serial.print(IR.direction);
                Serial.print(", Dist : ");
                Serial.print(IR.distance);
                Serial.print(", Ballanfahrt : ");
                Serial.print(IR.ballanfahrt_neuW);
                Serial.print(" ; ");
                Serial.println(IR.ballanfahrt_neuSF); */

                /* Serial.print(" US : ");
                Serial.print(US.dist_f);
                Serial.print(" : ");
                Serial.print(US.dist_b);
                Serial.print(" x: ");
                Serial.println(US.offsetx); */

                Serial.print(" US : ");
                Serial.print(" l : ");
                Serial.print(US.dist_l);
                Serial.print(" r : ");
                Serial.print(US.dist_r);
                Serial.print(" f : ");
                Serial.print(US.dist_f);
                Serial.print(" h : ");
                Serial.println(US.dist_b);

                Serial.print("LDR: ");
                Serial.print(LDR.ballda);
                Serial.print(" : ");
                Serial.println(LDR.value);
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

            // Timing ausgeben
            /* Serial.print("LoopTime: ");
            Serial.print(lastLoopTime);
            Serial.print("; LoopError: (der loop davor war um x zu lang) ");
            Serial.print(LoopError);
            Serial.print("; davon UpdateTime: ");
            Serial.println(lastUpdateTime); */
        }
    }

    // Verarbeitet die wichtigsten Sensor Werte und entscheidet über die Taktiken
    void gameLogik(void)
    {
        bool hasBall = (LDR.ballda && (IR.direction >= -25 && IR.direction <= 25)); // LDR und IR Sensor sehen den Ball
        bool amIinanCorner = false;
        bool IshoouldDefend = (US.dist_b < 20 && (!hasBall) && (IR.Orbit_direction >= 20 || IR.Orbit_direction <= -20) && (IR.Orbit_direction <= 95 && IR.Orbit_direction >= -95));
        // bool amIinanLOP = (Tactics.LOPTimer >= LOP_TimerLimit); // Wenn der Ball nicht erkannt wird nach bestimmter Zeit versuchen durch bewegung den Ball zu erkennen
        // bool PixySeestheGoalandBothastheBall = (cam.goal_visible && hasBall);
        // bool IRSeestheBall = (IR.reliable);

        { // corner detection
            bool cornerdetected = false;

            if ((US.dist_l < 25 || US.dist_r < 25) && US.dist_f < 22)
            {
                cornerdetected = true;
                if (CornerTimer > 6000 || cornerisUnset) // neu erkannt
                // CornerTimer = 0;
                {
                    if (US.onLeftSide)
                        isItLeftCorner = true, Serial.print("LEFT CORNER DETECTED");
                    else
                        isItLeftCorner = false, Serial.print("RIGHT CORNER DETECTED");
                    // isItLeftCorner = (US.offsetx > 0);
                    cornerisUnset = false;

                    Serial.print("### L: ");
                    Serial.print(US.dist_l);
                    Serial.print(" R: ");
                    Serial.print(US.dist_r);
                    Serial.print(" Clock: ");
                    Serial.println(CornerTimer);

                } // Werte zurücksetzen

                if (CornerTimer > 6100) // neu erkannt 100 MS für ecken erkennung
                    CornerTimer = 0, cornerisUnset = true;

                if (cornerdetected || (CornerTimer < 6000))
                    amIinanCorner = true;

                if ((IR.Orbit_direction > 70 || IR.Orbit_direction < -70) || (US.dist_f > 40)) // Wenn der Ball neben oder hinter dem Robooter ist können wir nicht in der Ecke stehen -> Ballanfahrtspeed nicht verkleinern
                    amIinanCorner = false, cornerisUnset = true, CornerTimer = 10000;
            }

            if (!amIinanCorner)
                cornerisUnset = true;
        }

        bool avoidObstacle = (!amIinanCorner && US.dist_f < 20 && hasBall);

        // hasBall = true;

        // MAIN LOGIC
        if (IshoouldDefend)
            move = Tactics.defend(IR, move), StatusR = 255, StatusG = 70, StatusB = 0;
        else if (!hasBall) // Hat keinen Ball
            move = Tactics.ballanfahrt(IR, amIinanCorner, isItLeftCorner), StatusR = 255, StatusG = 0, StatusB = 0;
        else if (amIinanCorner) // Hat ball und ist in einer Ecke
            move = Tactics.corner(isItLeftCorner, CornerTimer, cornerisUnset, US, Compass, move), StatusR = 0, StatusG = 200, StatusB = 255;
        else if (avoidObstacle)
            move.angle = 100 * (US.dist_l < US.dist_r ? -1 : 1), move.speed = Tactics.stdSpeed, move.AngleOfAttack = 0, StatusR = 255, StatusG = 0, StatusB = 255;
        else // hat ball (und ist nicht in der Ecke)
            move = Tactics.toranfahrt(false, US, Compass, cam, move), StatusR = 0, StatusG = 255, StatusB = 0;

        // lila als Avoid

        Serial.print(" Corner : ");
        Serial.print(amIinanCorner);
        Serial.print(" Left : ");
        Serial.print(isItLeftCorner);
        Serial.print(" Defend : ");
        Serial.println(IshoouldDefend);

        // Dribbler Logik
        if (hasBall)
            Tactics.action.dribbler(true);
        else
            Tactics.action.dribbler(false);

        move.currentCompassAngle = Compass.orbitDirection;
        Tactics.action.move(move);

        return; // SICHERUNG

        /*         if (amIinanLOP)
                    Tactics.resolveLOP(move);
                else if (PixySeestheGoalandBothastheBall) // Wenn die Pixy dass Tor sieht und der Ball da ist
                    Tactics.toranfahrt(true, US, Compass, cam, move);
                else if (hasBall) // LDR und IR Sensor sehen den Ball
                    Tactics.toranfahrt(false, US, Compass, cam, move);
                else if (IRSeestheBall) // IR Sensor sieht den Ball
                {
                    GoalTargetAngel = 0;  // Toranfahrt zurücksetzen
                    Tactics.LOPTimer = 0; // LOP Timer zuruecksetzen
                    Tactics.ballanfahrt(IR, false);
                }
                else // IR Sensor sieht keinen Ball
                    Tactics.homing(US);

                move.currentCompassAngle = Compass.orbitDirection;
                Tactics.action.move(move); */
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
    // InteruptTimer.begin(InteruptTimerFunc, 5 * 1000);
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