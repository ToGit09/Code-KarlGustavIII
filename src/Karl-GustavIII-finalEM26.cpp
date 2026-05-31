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

// Zwei Hauptklassen: CodeRobot -> Zentrale Verwaltung des roboters, CodeTactics -> Alle Taktiken und Bewegungsfunktionen

/** Setup *********************************************************************************/
#include <Arduino.h>
#include <Bot.h> // Bot
#include <Servo.h>

/** Highlevel *****************************************************************************/

// Code für die Taktiken
class CodeTactics
{
public:
    Codeaction action;
    elapsedMillis LOPTimer; // Lack of progress Timer
    int stdSpeed = 50;

    /**
     * @brief Verteidigung im Eigenen Tor
     * @param IR_Data IR-Sensordaten
     * @param move Bewegungsbefehl
     * @return Aktualisierter Bewegungsbefehl
     */
    movement_event defend(ir_sensor_event IR_Data, movement_event move = movement_event())
    {
        if (IR_Data.Orbit_direction > 0) // links
            move.angle = -110, move.speed = 100, move.AngleOfAttack = 0;
        else // rechts
            move.angle = 110, move.speed = 100, move.AngleOfAttack = 0;

        return move;
    }

    /**
     * @brief Fährt den Ball an und berücksichtigt dabei Ultraschall-Abstände.
     * @param IR_Data IR-Sensordaten
     * @param US_Data Ultraschall-Sensordaten
     * @param move Bewegungsbefehl
     * @return Aktualisierter Bewegungsbefehl
     */
    movement_event ballanfahrt(ir_sensor_event IR_Data, us_sensor_event US_Data, movement_event move = movement_event())
    {

        move.angle = IR_Data.ballanfahrt_neuW, move.AngleOfAttack = 0;

        if (US_Data.dist_r < 15 && IR_Data.ballanfahrt_neuW > 0) // wenn rechts wenig Platz ist, nicht nach rechts fahren
        {
            if (abs(IR_Data.ballanfahrt_neuW) > 90)
                move.angle = 170, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = 10, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (US_Data.dist_l < 15 && IR_Data.ballanfahrt_neuW < 0)
        {
            if (abs(IR_Data.ballanfahrt_neuW) > 90)
                move.angle = -170, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = -10, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (US_Data.dist_b < 15 && abs(IR_Data.ballanfahrt_neuW) > 90)
        {
            if (IR_Data.ballanfahrt_neuW > 0)
                move.angle = -85, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = 85, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (abs(IR_Data.Orbit_direction) < 25)
            move.AngleOfAttack = IR_Data.Orbit_direction;

        if (abs(move.AngleOfAttack) > 12)
            move.AngleOfAttack = 12 * (move.AngleOfAttack / abs(move.AngleOfAttack)); // wenn der Angle of Attack zu groß ist, auf 5 begrenzen

        move.speed = stdSpeed; //  * IR_Data.ballanfahrt_neuSF; // Standardgeschwindigkeit

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
     * @brief Bewegungsfunktion für Eckensituationen.
     *
     * @param isItLeftCorner Gibt an, ob es die linke Ecke ist.
     * @param US Ultraschall-Sensordaten.
     * @param comp Kompass-Sensordaten.
     * @param move Bewegungs-Event, das angepasst wird.
     * @param ir IR-Sensordaten.
     * @return movement_event Das angepasste Bewegungs-Event.
     *
     * Die Funktion steuert das Verhalten in der Ecke:
     * - Ist der Frontabstand (US.dist_f) größer als 30, fährt der Roboter geradeaus
     *   (Winkel 0, AngleOfAttack 0) mit Standardgeschwindigkeit und löst einen Kick aus.
     * - Andernfalls dreht der Roboter mit Winkel ±115 (abhängig von der Ecke),
     *   ebenfalls mit Standardgeschwindigkeit und AngleOfAttack 0.
     */
    movement_event corner(bool isItLeftCorner, us_sensor_event US, compass_sensor_event comp, movement_event move = movement_event(), ir_sensor_event ir = ir_sensor_event())
    {
        if ((US.dist_f > 30))
            move.angle = 0, move.AngleOfAttack = 0, move.speed = stdSpeed, action.kick();
        else
            move.angle = 115 * (isItLeftCorner ? -1 : 1), move.speed = stdSpeed, move.AngleOfAttack = 0;

        Serial.print(" ECKE : WINKEL = ");
        Serial.print(move.angle);
        Serial.print(" AOT = ");
        Serial.println(move.AngleOfAttack);

        return move;
    }

    /**
     * @brief Ball anfahren, wenn der Ball gesehen wird
     * @param US Der US-Sensor-Event
     * @param Compass Der Kompass-Event
     * @param move Der Movement-Event
     * @return Der angepasste Movement-Event
     *
     * Die Funktion richtet den Roboter anhand des seitlichen Ballabstands aus.
     * Bei mittiger Position wird ein Kick ausgelöst.
     */
    movement_event toranfahrt(us_sensor_event US, compass_sensor_event Compass, movement_event move = movement_event())
    {

        move.AngleOfAttack = 0, move.speed = stdSpeed;

        if (US.offsetx > -20 && US.offsetx < 20) // ungefähr mitte
            action.kick(), move.angle = 0;
        else
            move.angle = 70 * (US.offsetx > 0 ? -1 : 1);

        return move;
    }

    void printGameLogikStatus(bool hasBall, bool amIinanCorner, bool IshoouldDefend, bool avoidObstacle)
    {
        Serial.print(millis());
        Serial.print(" : KG3 ist ");
        Serial.print(amIinanCorner ? "in einer Ecke" : "");
        Serial.print(hasBall ? "am Ball" : "nicht am Ball");
        Serial.print(IshoouldDefend ? " und verteidigt sein Tor" : "");
        Serial.println(avoidObstacle ? " und weicht einem Hindernis aus" : "");
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
    movement_event move;

    // Timer
    elapsedMillis ReadTimer;     // Read Timer
    elapsedMillis ReadTimer2;    // Read Timer
    elapsedMillis ReadTimer3;    // Read Timer
    elapsedMillis DribblerTimer; // Dribbler Timer
    elapsedMillis LEDTimer;      // RGB Timer
    elapsedMillis totalTime;     // Insgesamte Zeit die verstrichen ist
    elapsedMillis LoopTime;      // Zeit die ein durchlauf braucht
    elapsedMillis LoopTimer;     // immer hochzählen bis zehn oder so und dann starten
    elapsedMillis InitTime;      // Zeit die ein initialize durchlauf braucht
    elapsedMillis PIDTimer;      // PID Timer
    elapsedMillis CornerTimer;   // Corner Timer
    elapsedMicros UpdateTime;    // Zeit die ein Update durchlauf braucht

    // Timings
    int InitT = -1;
    bool isItLeftCorner = false;
    bool cornerisUnset = true;
    int StatusR = 0;
    int StatusG = 0;
    int StatusB = 0;
    int lBNOidx = 0;
    int lIRidx = 0;

public:
    CodeRead Read;

    CodeTactics Tactics;

    // Initialisierung des Roboters
    void initialize(void)
    {
        InitTime = 0; // InitTimer zurücksetzen

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

        //-INA------------------------------------------------------------------------------------//
        INA.init();
        //----------------------------------------------------------------------------------------//

        //-US-------------------------------------------------------------------------------------//
        sensorVorne.setEMAAlpha(0.25f);
        sensorHinten.setEMAAlpha(0.25f);
        sensorLinks.setEMAAlpha(0.25f);
        sensorRechts.setEMAAlpha(0.25f);

        sensorVorne.setJumpThreshold(50);
        sensorHinten.setJumpThreshold(50);
        sensorLinks.setJumpThreshold(50);
        sensorRechts.setJumpThreshold(50);

        sensorVorne.begin(Wire1);
        sensorHinten.begin(Wire1);
        sensorLinks.begin(Wire1);
        sensorRechts.begin(Wire1);

        sensorVorne.startRanging();
        sensorHinten.startRanging();
        sensorLinks.startRanging();
        sensorRechts.startRanging();
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

        //-Dribbler-------------------------------------------------------------------------------//

        DRIBBLER.init(33); // Init

        Tactics.action.setRGB(0, 0, 0, 255);
        Tactics.action.setRGB(1, 255, 0, 0);
        Tactics.action.setRGB(2, 0, 0, 255);

        for (int i = 3; i < 19; i++)
            Tactics.action.setRGB(i, 255, 255, 0);
        Tactics.action.renderRGBs();

        while (INA.Voltage_DR() > 4)
        {
            delay(30);
        }

        for (int i = 3; i < 19; i++)
            Tactics.action.setRGB(i, 0, 0, 0);
        Tactics.action.renderRGBs();

        int i = 3;
        int li = 3;

        while (INA.Voltage_DR() < 4)
        {

            Tactics.action.setRGB(li, 0, 0, 0);
            Tactics.action.setRGB(i, 0, 150, 255);

            Tactics.action.renderRGBs();

            li = i;
            i++;
            delay(100);

            if (i < 3)
                i = 3;
            if (i > 18)
                i = 3;
        }

        for (int i = 0; i < 19; i++)
            Tactics.action.setRGB(i, 0, 255, 255);
        Tactics.action.renderRGBs();

        delay(100);

        DRIBBLER.init_Power();

        for (int i = 0; i < 19; i++)
            Tactics.action.setRGB(i, 0, 255, 0);
        Tactics.action.renderRGBs();

        delay(2500);

        for (int i = 0; i < 19; i++)
            Tactics.action.setRGB(i, 0, 0, 0);
        Tactics.action.renderRGBs();

        //----------------------------------------------------------------------------------------//

        //-Variablen------------------------------------------------------------------------------//
        Tactics.action.sqrt3 = sqrtf(3);
        //----------------------------------------------------------------------------------------//

        InitT = InitTime; // Initialisierungszeit speichern

        //-Optik----------------------------------------------------------------------------------//
        Serial.print(InitTime);

        for (int i = 0; i < 19; i++)
        {
            Tactics.action.setRGB(i - 1, 0, 0, 0);
            Tactics.action.setRGB(i, 0, 255, 0);
            Tactics.action.renderRGBs();
            delay(40);
        }

        Tactics.action.setRGB(18, 0, 0, 0);

        Tactics.action.setRGB(0, 0, 255, 0);
        Tactics.action.setRGB(1, 0, 255, 0);
        Tactics.action.setRGB(2, 0, 255, 0);

        delay(200);
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

        if (ReadTimer >= 100)
            switches = Read.Switches(),
            ReadTimer = 0;

        if (ReadTimer2 >= 10)
            US = Read.US(Compass),
            ReadTimer2 = 0;

        if (ReadTimer3 >= 20)
            LDR = Read.LDR(),
            ReadTimer3 = 0;

        IR = Read.IR(Compass, US, switches.BTN4); // IR Sensoren auslesen, abhängig von Kompass und US Werten (für ballanfahrt), und BTN4 (für IR Kalibrierung)

        lastUpdateTime = UpdateTime;

        // Serial.print(" Sensorwerte Aktualisiert in : ");
        // Serial.println(UpdateTime);
    }

    // Handelt das Timing der Game Funktion und speichert alle sensor Werte
    void system(void)
    {
        if (LoopTimer >= LoopTiming)
        {
            LoopError = LoopTimer - LoopTiming; // Fehler im Loop Timer
            LoopTime = 0;                       // Timer der die Länge des Loops misst

            Tactics.action.kicker_reset();
            update();
            Tactics.action.kicker_reset(); // für präzisere Kicks

            switches.MainSwitch = digitalRead(MAINSWITCH_PORT);

            if (switches.MainSwitch)
            {
                Tactics.action.setRGB(0, StatusR, StatusG, StatusB);
                Tactics.action.setRGB(3, StatusR, StatusG, StatusB);

                if (switches.SWI1)
                    Tactics.stdSpeed = 30;
                else
                    Tactics.stdSpeed = 94;

                if (switches.SWI2) // Instantkick / DO NOT CHANGE
                    ;

                if (switches.SWI3)
                    ;
                else
                    ;

                gameLogik();

                Serial.println("----");
                Serial.print(IR.ballanfahrt_neuW);
                Serial.print(" | ");
                Serial.print(IR.ballanfahrt_neuSF);
                Serial.print(" | ");
                Serial.print(IR.Orbit_direction);
                Serial.print(" | ");
                Serial.println(IR.distance);
                // Tactics.action.setMotors(-60, 60, 60, 0);
            }
            else
            {
                Tactics.action.setRGB(0, 0, 0, 255); // Lever State
                Tactics.action.setRGB(3, 0, 0, 255); // Lever State
                Tactics.LOPTimer = 0;                // Damit sich der LOP Timer nicht schon vor dem Spiel hochschraubt....
                Tactics.action.brake();
                CornerTimer = 10000;

                if (switches.BTN1)
                    Read.KompassCliValue = Compass.raw, Read.calibrateLDR();

                if (switches.BTN2)
                {
                    Tactics.action.kick();
                    delay(13);
                    digitalWrite(KICKER_PORT, LOW);
                }

                if (switches.BTN3)
                    Read.calibrateLDR();

                if (switches.BTN4)
                    Read.calibrateIR(IR);

                if (switches.SWI1)
                    DRIBBLER.set(DribbleSpeed);
                else
                    DRIBBLER.set(0);

                if (switches.SWI2)
                    ;

                if (switches.SWI3)
                    ;
            }

            doRGBs();

            lastLoopTime = LoopTime; // letzte Loop Zeit speichern
            LoopTimer = 0;           // System Timing zurücksetzen

            if (millis() % 300 < 12)
                printData();

            // Timing ausgeben
            /*Serial.print("LoopTime: ");
            Serial.print(lastLoopTime);
            Serial.print("; LoopError: (der loop davor war um x zu lang) ");
            Serial.print(LoopError);
            Serial.print("; davon UpdateTime: ");
            Serial.println(lastUpdateTime);*/
        }
    }

    // Verarbeitet die wichtigsten Sensor Werte und entscheidet über die Taktiken
    void gameLogik(void)
    {
        move.angle = 0, move.speed = 0, move.AngleOfAttack = 0; // default Werte

        bool hasBall = (LDR.ballda && (IR.direction >= -12 && IR.direction <= 12)); // LDR und IR Sensor sehen den Ball
        bool amIinanCorner = false;
        bool IshoouldDefend = (US.dist_b < 25 && (!hasBall) && (IR.Orbit_direction >= 20 || IR.Orbit_direction <= -20) && (IR.Orbit_direction <= 95 && IR.Orbit_direction >= -95));

        unsigned long CornerTime = 4500UL;
        unsigned long Timeout = 400UL;

        if (US.dist_f < 25 || CornerTimer < CornerTime + Timeout)
        {
            if ((CornerTimer > CornerTime + Timeout) && (US.dist_l < 35 || US.dist_r < 35)) // wenn eine neue Ecke erkannt wird
                CornerTimer = 0, isItLeftCorner = (US.dist_l < US.dist_r), amIinanCorner = true;
            else if (CornerTimer > CornerTime)
                amIinanCorner = false;
            else
                amIinanCorner = true;
        }

        if (US.dist_f > 40)
            CornerTimer = 10000;

        bool avoidObstacle = (!amIinanCorner && US.dist_f < 30 && hasBall);

        // MAIN LOGIC
        if (IshoouldDefend)
            move = Tactics.defend(IR, move), StatusR = 255, StatusG = 70, StatusB = 0;

        else if (!hasBall) // Hat keinen Ball
            move = Tactics.ballanfahrt(IR, US), StatusR = 255, StatusG = 0, StatusB = 0;

        else if (amIinanCorner) // Hat ball und ist in einer Ecke
            move = Tactics.corner(isItLeftCorner, US, Compass, move, IR), StatusR = 0, StatusG = 200, StatusB = 255;

        else if (avoidObstacle)
            move.angle = 160 * (US.onLeftSide ? -1 : 1), move.speed = Tactics.stdSpeed, move.AngleOfAttack = 10 * (US.onLeftSide ? -1 : 1), StatusR = 255, StatusG = 0, StatusB = 255;

        else // hat ball (und ist nicht in der Ecke)
            move = Tactics.toranfahrt(US, Compass, move), StatusR = 0, StatusG = 255, StatusB = 0;

        if (switches.SWI2 && hasBall && !amIinanCorner && US.dist_f > 25)
            ; // Tactics.action.kick();

        if (hasBall || (abs(IR.Orbit_direction) < 50))
            DRIBBLER.set(DribbleSpeed);
        else
            DRIBBLER.set(0);

        move.currentCompassAngle = Compass.orbitDirection;
        // Tactics.action.move(move);

        Tactics.printGameLogikStatus(hasBall, amIinanCorner, IshoouldDefend, avoidObstacle);
    }

    void printData()
    {
        Serial.print(">");

        Serial.print("time:");
        Serial.print(millis());
        Serial.print(",");

        Serial.print("looptime:");
        Serial.print(lastLoopTime);
        Serial.print(",");

        Serial.print("Kompass:");
        Serial.print(Compass.orbitDirection);
        Serial.print(",");

        Serial.print("ir_dir:");
        Serial.print(IR.Orbit_direction);
        Serial.print(",");

        Serial.print("ir_dist:");
        Serial.print(IR.distance);
        Serial.print(",");

        Serial.print("us_f:");
        Serial.print(US.dist_f);
        Serial.print(",");

        Serial.print("us_b:");
        Serial.print(US.dist_b);
        Serial.print(",");

        Serial.print("us_l:");
        Serial.print(US.dist_l);
        Serial.print(",");

        Serial.print("us_r:");
        Serial.print(US.dist_r);
        Serial.print(",");

        Serial.print("LDR:");
        Serial.print(LDR.ballda);
        Serial.print(",");

        Serial.print("ballanfahrt:");
        Serial.print(IR.ballanfahrt_neuW);
        Serial.print(",");

        Serial.print("ballanfahrtSF:");
        Serial.print(IR.ballanfahrt_neuSF);
        Serial.print(",");

        Serial.print("INA :");
        Serial.print(INA.Current_DR());
        Serial.println();
    }

    void doRGBs()
    {
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

        if (!switches.MainSwitch)
        {
            bool temp = false;
            for (int i = 0; i < 3; i++)
            {
                if (Read.IR_gains[i] == 0)
                    temp = true;
                if (Read.IR_offsets[i] >= 6000)
                    temp = true;
            }

            if (temp)
                Tactics.action.setRGB(2, 255, 0, 0); // IR Kalibrierung
            else
                Tactics.action.setRGB(2, 0, 255, 0); // IR Kalibrierung
        }
        else
            Tactics.action.setRGB(2, r, g, b); // Lever states

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

        // 0 reserviert für Status LEDs

        int BNOidx = 19 - round((Compass.orbitDirection + 180) / 24 + 0.5);
        int IRidx = 3 + round((IR.direction + 180) / 24 + 0.5);

        if (BNOidx < 3)
            BNOidx = 3;
        if (IRidx > 18)
            IRidx = 18;

        Tactics.action.setRGB(lBNOidx, 0, -1, -1);
        Tactics.action.setRGB(BNOidx, 255, -1, -1);

        Tactics.action.setRGB(lIRidx, -1, 0, -1);
        Tactics.action.setRGB(IRidx, -1, 255, -1);

        lIRidx = IRidx;
        lBNOidx = BNOidx;

        if (LDR.ballda)
        {
            Tactics.action.setRGB(10, -1, -1, 255);
            Tactics.action.setRGB(12, -1, -1, 255);
        }
        else
        {
            Tactics.action.setRGB(10, -1, -1, 0);
            Tactics.action.setRGB(12, -1, -1, 0);
        }

        Tactics.action.renderRGBs();
    }
};

/** Main **********************************************************************************/
CodeRobot Karl_GustavIII;

void setup()
{
    Karl_GustavIII.initialize();
    Serial.println("Setup done");
}

void loop()
{
    Karl_GustavIII.system();
}
/******************************************************************************************/