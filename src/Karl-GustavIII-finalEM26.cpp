/**
 * @file Karl-GustavIII-finalEM26.cpp
 * @brief Main control code for Karl-GustavIII robot (EM2026 competition)
 * @version 4.5
 *
 * Pin Configuration (Teensy 3.6):
 * Motor Control:
 *   0-1: Motor 1 direction (MOT1_IN1, MOT1_IN2)
 *   2-3: Motor 2 direction (MOT2_IN1, MOT2_IN2)
 *   4-5: Motor 1,2 PWM speed control
 *   6-9: Motor 3,4 speed & direction control
 *
 * Input Sensors:
 *   10: Main power switch (start/stop)
 *   23: Light barrier (ball detection LDR)
 *   24: ADC input (camera/external sensors)
 *
 * Communication:
 *   11-13: SPI bus (motors, expansion)
 *   14-15: Serial TX/RX (camera UART)
 *   16-17: I2C bus 1 (port expanders, sensors)
 *   28-29: Serial UART2 (Teensy communication)
 *
 * Actuators:
 *   18: Kicker trigger output
 *   19,21-22: SPI data lines
 *   20: RGB LED strip data
 *   30-31: SPI data lines
 *   33: Dribbler motor PWM
 *
 * System Architecture:
 * - Main game logic loop running at LoopTiming (typically 30ms)
 * - Sensor updates at various intervals (10-100ms)
 * - 19-LED RGB ring for status indication
 * - 4-motor omnidirectional drive (omni-wheels)
 * - IR ring for ball detection
 * - Compass for heading tracking
 * - Ultrasonic sensors for obstacle detection
 *
 * Game States (reflected in LED color):
 *   Orange: Defending goal
 *   Red: Seeking ball
 *   Cyan: In corner with ball
 *   Magenta: Obstacle avoidance
 *   Green: Driving to goal
 */

/**
 * Motor Layout:
 *   Motor 0,1: Rear axis (back)
 *   Motor 1,2: Left axis (port)
 *   Motor 2,3: Right axis (starboard)
 *   Motor 3,4: Extra motor (unused)
 */

/** Includes **********************************************************************************/
#include <Arduino.h>
#include <Bot.h> // Bot library
#include <Servo.h>

/** Tactics Class *****************************************************************************/

/**
 * @class CodeTactics
 * @brief High-level tactic implementations for game strategy
 *
 * Implements movement tactics for:
 * - Ball pursuit with obstacle avoidance
 * - Corner handling (rotation to shoot)
 * - Goal approach (centering and scoring)
 * - Defensive positioning
 * - General game state logging
 */
class CodeTactics
{
public:
    Codeaction action;
    elapsedMillis LOPTimer; // Lack of progress timer
    int stdSpeed = 50;

    /**
     * @brief Defensive positioning - moves robot to block goal area
     * @param IR_Data IR sensor data for ball direction
     * @param move Movement event to be modified
     * @return Modified movement event
     *
     * Positions robot at ±110° to the ball direction to block incoming shots.
     */
    movement_event defend(ir_sensor_event IR_Data, movement_event move = movement_event())
    {
        if (IR_Data.Orbit_direction > 0) // ball on left
            move.angle = -110, move.speed = 100, move.AngleOfAttack = 0;
        else // ball on right
            move.angle = 110, move.speed = 100, move.AngleOfAttack = 0;

        return move;
    }

    /**
     * @brief Drives towards the ball with obstacle avoidance and angle of attack control
     * @param IR_Data IR sensor data containing ball direction and distance
     * @param US_Data Ultrasonic sensor data for obstacle detection (left, right, back)
     * @param move The current movement event to be modified
     * @return Modified movement event with calculated angle, speed and angle of attack
     *
     * This function drives the robot towards the ball as determined by IR sensors.
     * It implements obstacle avoidance by adjusting angle if too close to walls.
     * Angle of attack is limited to ±12 degrees for stability.
     */
    movement_event ballanfahrt(ir_sensor_event IR_Data, us_sensor_event US_Data, movement_event move = movement_event())
    {

        move.angle = IR_Data.ballanfahrt_neuW, move.AngleOfAttack = 0;

        if (US_Data.dist_r < 15 && IR_Data.ballanfahrt_neuW > 0) // obstacle on right, steer left
        {
            if (abs(IR_Data.ballanfahrt_neuW) > 90)
                move.angle = 170, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = 10, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (US_Data.dist_l < 15 && IR_Data.ballanfahrt_neuW < 0) // obstacle on left, steer right
        {
            if (abs(IR_Data.ballanfahrt_neuW) > 90)
                move.angle = -170, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = -10, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (US_Data.dist_b < 15 && abs(IR_Data.ballanfahrt_neuW) > 90) // obstacle behind, steer sideways
        {
            if (IR_Data.ballanfahrt_neuW > 0)
                move.angle = -85, move.speed = stdSpeed, move.AngleOfAttack = 0;
            else
                move.angle = 85, move.speed = stdSpeed, move.AngleOfAttack = 0;
        }

        if (abs(IR_Data.Orbit_direction) < 25)
            move.AngleOfAttack = IR_Data.Orbit_direction;

        if (abs(move.AngleOfAttack) > 12)
            move.AngleOfAttack = 12 * (move.AngleOfAttack / abs(move.AngleOfAttack)); // limit angle of attack to ±12°

        move.speed = stdSpeed; // standard driving speed

        // Debug output
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
     * @brief Corner movement with ball handling
     *
     * @param isItLeftCorner True if robot is in left corner, false for right corner
     * @param US Ultrasonic sensor data for distance measurements
     * @param comp Compass sensor data
     * @param move The movement event to be modified
     * @param ir IR sensor data
     * @return Modified movement event for corner navigation
     *
     * If front distance > 30cm: moves forward (angle=0) and kicks the ball.
     * Otherwise: rotates left or right depending on corner side with standard speed.
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
     * @brief Drive towards goal for shooting
     *
     * @param US Ultrasonic sensor data for distance and offset measurements
     * @param Compass Compass sensor data
     * @param move The movement event to be modified
     * @return Modified movement event for goal approach
     *
     * Centers the robot relative to the goal using ultrasonic offset measurement.
     * If centered (offset ±20cm): executes kick and moves forward (angle=0).
     * Otherwise: moves left or right to reach center position with standard speed.
     */
    movement_event toranfahrt(us_sensor_event US, compass_sensor_event Compass, movement_event move = movement_event())
    {

        move.AngleOfAttack = 0, move.speed = stdSpeed;

        if (US.offsetx > -20 && US.offsetx < 20) // approximately centered
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

    // Main robot code
    class CodeRobot
    {
    private:
        // sensor events
        ir_sensor_event IR;
        ldr_sensor_event LDR;
        compass_sensor_event Compass;
        us_sensor_event US;
        switches_event switches;
        movement_event move;

        // timing variables
        elapsedMillis ReadTimer;     // sensor update interval
        elapsedMillis ReadTimer2;    // alternative update interval
        elapsedMillis ReadTimer3;    // alternative update interval
        elapsedMillis DribblerTimer; // dribbler motor timing
        elapsedMillis LEDTimer;      // RGB LED update timing
        elapsedMillis totalTime;     // total elapsed time since start
        elapsedMillis LoopTime;      // duration of current loop iteration
        elapsedMillis LoopTimer;     // loop timing counter
        elapsedMillis InitTime;      // initialization duration
        elapsedMillis PIDTimer;      // PID control timing
        elapsedMillis CornerTimer;   // corner detection and handling
        elapsedMicros UpdateTime;    // sensor update duration

    // Timings and state variables
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

    /**
     * @brief Initialize robot hardware and sensors
     * - Sets up Serial communication (9600 baud)
     * - Configures all motor and sensor GPIO pins
     * - Initializes IR ring, RGB LEDs, I2C port expanders
     * - Initializes compass (BNO055), ultrasonic sensors, current sensor (INA)
     * - Runs startup LED sequence and boot diagnostics
     * - Calibrates PWM frequencies for motor control
     * - Initializes dribbler motor and displays ready status
     */
    void initialize(void)
    {
        InitTime = 0;

        // Serial communication initialization (9600 baud)
        Serial.begin(9600);

        // Configure GPIO pins for motors, switches, and sensors
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

        // IR ring sensor initialization
        IRh.init();

        // RGB LED strip initialization
        Tactics.action.initRGBs();

        // I2C port expanders initialization (address 0x20 for switches)
        Wire1.begin();
        Wire1.beginTransmission(0x20);
        Wire1.write(0x03);
        Wire1.write(0xFF);
        Wire1.endTransmission();
        Wire1.beginTransmission(0x20);
        Wire1.write(0x01);
        Wire1.write(0x00);
        Wire1.endTransmission();

        // Compass (BNO055) initialization
        BNO.begin();
        BNO.setExtCrystalUse(true);

        // SPI bus initialization (for ADC and sensor communication)
        SPI.begin();

        // Current sensor (INA) initialization
        INA.init();

        // Ultrasonic sensors configuration
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

        delay(100);

        // LDR (Light Dependent Resistor) pin configuration
        pinMode(24, INPUT);

        // Motor PWM frequency initialization
        analogWriteFrequency(motor1.PWM, motor1.frequenz);
        analogWriteFrequency(motor2.PWM, motor2.frequenz);
        analogWriteFrequency(motor3.PWM, motor3.frequenz);
        analogWriteFrequency(motor4.PWM, motor4.frequenz);

        // Dribbler motor initialization
        DRIBBLER.init(33);

        // Startup LED sequence - displaying boot status
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
        {
            Tactics.action.setRGB(i, 0, 255, 255);
        }
        Tactics.action.renderRGBs();
        delay(100);

        // Button pressed - continue with dribbler power initialization
        DRIBBLER.init_Power();

        // Ready status - all LEDs green
        for (int i = 0; i < 19; i++)
        {
            Tactics.action.setRGB(i, 0, 255, 0);
        }
        Tactics.action.renderRGBs();
        delay(2500);

        // Clear LEDs
        for (int i = 0; i < 19; i++)
        {
            Tactics.action.setRGB(i, 0, 0, 0);
        }
        Tactics.action.renderRGBs();

        // Initialize constants
        Tactics.action.sqrt3 = sqrtf(3);

        InitT = InitTime; // save initialization time

        // Startup animation - LED sweep
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

        // Initialize timing variables
        ReadTimer = 0;
        LoopTime = 0;
        UpdateTime = 0;
        LoopTimer = 0;
        Tactics.LOPTimer = 0;
        CornerTimer = 10000;
    }

    /**
     * @brief Update sensor readings from all sensors
     * - Reads compass every loop (high frequency)
     * - Reads switches every 100ms
     * - Reads ultrasonic sensors every 10ms
     * - Reads LDR (ball detection) every 20ms
     * - Reads IR sensors every loop
     */
    void update(void)
    {
        UpdateTime = 0;

        Compass = Read.Compass();

        if (ReadTimer >= 100)
        {
            switches = Read.Switches();
            ReadTimer = 0;
        }

        if (ReadTimer2 >= 10)
        {
            US = Read.US(Compass);
            ReadTimer2 = 0;
        }

        if (ReadTimer3 >= 20)
        {
            LDR = Read.LDR();
            ReadTimer3 = 0;
        }

        IR = Read.IR(Compass, US, switches.BTN4); // IR sensor reading with compass/US calibration and calibration button

        lastUpdateTime = UpdateTime;
    }

    /**
     * @brief Main system loop handler - manages timing and game logic execution
     * - Synchronizes execution to LoopTiming (main control frequency)
     * - Handles main switch (robot on/off)
     * - Routes to game logic when enabled
     * - Manages calibration buttons when disabled
     * - Updates RGB LED status display
     */
    void system(void)
    {
        if (LoopTimer >= LoopTiming)
        {
            LoopError = LoopTimer - LoopTiming; // timing error
            LoopTime = 0; // loop iteration timer

            Tactics.action.kicker_reset();
            update();

            switches.MainSwitch = digitalRead(MAINSWITCH_PORT);

            if (switches.MainSwitch)
            {
                // Robot enabled - update status LEDs
                Tactics.action.setRGB(0, StatusR, StatusG, StatusB);
                Tactics.action.setRGB(3, StatusR, StatusG, StatusB);

                // Speed selection via switch SWI1
                if (switches.SWI1)
                    Tactics.stdSpeed = 30; // slow mode
                else
                    Tactics.stdSpeed = 94; // normal mode

                if (switches.SWI2)
                    ; // instant kick mode

                if (switches.SWI3)
                    ; // reserved for future

                gameLogik();

                // Debug output
                Serial.println("----");
                Serial.print(IR.ballanfahrt_neuW);
                Serial.print(" | ");
                Serial.print(IR.ballanfahrt_neuSF);
                Serial.print(" | ");
                Serial.print(IR.Orbit_direction);
                Serial.print(" | ");
                Serial.println(IR.distance);
            }
            else
            {
                // Robot disabled - enable calibration and diagnostics
                Tactics.action.setRGB(0, 0, 0, 255); // standby status LED
                Tactics.action.setRGB(3, 0, 0, 255); // standby status LED
                Tactics.LOPTimer = 0; // reset lack-of-progress timer before game start
                Tactics.action.brake();
                CornerTimer = 10000;

                // Calibration buttons
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
                    ; // reserved

                if (switches.SWI3)
                    ; // reserved
            }

            doRGBs();

            lastLoopTime = LoopTime; // save previous loop duration
            LoopTimer = 0; // reset system timing

            if (millis() % 300 < 12)
                printData();

            // Debug timing output (uncomment for detailed loop timing analysis)
            /*Serial.print("LoopTime: ");
            Serial.print(lastLoopTime);
            Serial.print("; LoopError: (previous loop excess time) ");
            Serial.print(LoopError);
            Serial.print("; of which UpdateTime: ");
            Serial.println(lastUpdateTime);*/
        }
    }

    /**
     * @brief Main game logic - decides robot actions based on sensor input
     *
     * State machine logic:
     * 1. Defend - if ball approaching from rear without possession
     * 2. Drive to ball - if no ball possession
     * 3. Corner handling - if in corner with ball (rotates to shoot)
     * 4. Obstacle avoidance - if ball ahead and front obstacle
     * 5. Drive to goal - if ball possessed and not in corner
     *
     * Dribbler control:
     * - Enabled if ball possession OR ball within 50° orbit
     * - Disabled otherwise
     *
     * Corner detection: Uses CornerTimer and side distances (±35cm threshold)
     */
    void gameLogik(void)
    {
        move.angle = 0, move.speed = 0, move.AngleOfAttack = 0; // default values

        bool hasBall = (LDR.ballda && (IR.direction >= -12 && IR.direction <= 12)); // LDR and IR sensors confirm ball possession
        bool amIinanCorner = false;
        bool IshoouldDefend = (US.dist_b < 25 && (!hasBall) && (IR.Orbit_direction >= 20 || IR.Orbit_direction <= -20) && (IR.Orbit_direction <= 95 && IR.Orbit_direction >= -95));

        unsigned long CornerTime = 4500UL;
        unsigned long Timeout = 400UL;

        // Corner detection logic
        if (US.dist_f < 25 || CornerTimer < CornerTime + Timeout)
        {
            if ((CornerTimer > CornerTime + Timeout) && (US.dist_l < 35 || US.dist_r < 35)) // new corner detected
                CornerTimer = 0, isItLeftCorner = (US.dist_l < US.dist_r), amIinanCorner = true;
            else if (CornerTimer > CornerTime)
                amIinanCorner = false;
            else
                amIinanCorner = true;
        }

        if (US.dist_f > 40)
            CornerTimer = 10000; // reset corner timer if free space ahead

        bool avoidObstacle = (!amIinanCorner && US.dist_f < 30 && hasBall);

        // State machine: execute appropriate tactic based on current state
        if (IshoouldDefend)
            move = Tactics.defend(IR, move), StatusR = 255, StatusG = 70, StatusB = 0; // orange: defending

        else if (!hasBall) // no ball possession
            move = Tactics.ballanfahrt(IR, US), StatusR = 255, StatusG = 0, StatusB = 0; // red: seeking ball

        else if (amIinanCorner) // ball possession in corner
            move = Tactics.corner(isItLeftCorner, US, Compass, move, IR), StatusR = 0, StatusG = 200, StatusB = 255; // cyan: corner mode

        else if (avoidObstacle)
            move.angle = 160 * (US.onLeftSide ? -1 : 1), move.speed = Tactics.stdSpeed, move.AngleOfAttack = 10 * (US.onLeftSide ? -1 : 1), StatusR = 255, StatusG = 0, StatusB = 255; // magenta: obstacle avoidance

        else // ball possession, not in corner
            move = Tactics.toranfahrt(US, Compass, move), StatusR = 0, StatusG = 255, StatusB = 0; // green: driving to goal

        if (switches.SWI2 && hasBall && !amIinanCorner && US.dist_f > 25)
            ; // SWI2: instant kick mode

        // Dribbler control
        if (hasBall || (abs(IR.Orbit_direction) < 50))
            DRIBBLER.set(DribbleSpeed); // enable dribbler if ball possession or close to ball
        else
            DRIBBLER.set(0); // disable dribbler

        move.currentCompassAngle = Compass.orbitDirection;
        Tactics.action.move(move);

        Tactics.printGameLogikStatus(hasBall, amIinanCorner, IshoouldDefend, avoidObstacle);
    }

    /**
     * @brief Outputs telemetry data via serial at periodic intervals (~300ms)
     * Format: time, looptime, compass, IR direction, IR distance, US distances, LDR, ball approach angles, current draw
     */
    void printData()
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

    /**
     * @brief Updates RGB LED display for status indication
     *
     * LED layout (19 LEDs total):
     * - LED 0,3: Game status color (orange=defend, red=seek, cyan=corner, magenta=avoid, green=shoot)
     * - LED 1: Compass/magnetometer calibration status
     *   - Blue: not calibrated (-1)
     *   - Red: low calibration (0)
     *   - Orange: medium calibration (1)
     *   - Yellow: good calibration (2)
     *   - Green: fully calibrated (3)
     * - LED 2: IR ring calibration status (red=uncalibrated, green=calibrated)
     * - LEDs 3-18 (16-LED ring): 
     *   - Red channel: compass heading direction
     *   - Green channel: IR ball direction
     *   - Blue channels 10,12: ball detection (on when LDR active)
     * - SWI1-3: reflected in LED colors during standby mode
     */
    void doRGBs()
    {
        // Lever state colors (SWI1-3 reflected when enabled)
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

        // IR calibration status display
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
                Tactics.action.setRGB(2, 255, 0, 0); // red: IR not calibrated
            else
                Tactics.action.setRGB(2, 0, 255, 0); // green: IR calibrated
        }
        else
            Tactics.action.setRGB(2, r, g, b); // show lever states during game

        // Compass calibration status
        if (Compass.calibrationMag == -1)
            Tactics.action.setRGB(1, 0, 0, 255); // blue: not calibrated
        else if (Compass.calibrationMag == 0)
            Tactics.action.setRGB(1, 255, 0, 0); // red: low
        else if (Compass.calibrationMag == 1)
            Tactics.action.setRGB(1, 255, 70, 0); // orange: medium
        else if (Compass.calibrationMag == 2)
            Tactics.action.setRGB(1, 255, 255, 30); // yellow: good
        else if (Compass.calibrationMag == 3)
            Tactics.action.setRGB(1, 0, 255, 0); // green: fully calibrated

        // LED 0 reserved for status indicator

        // Calculate LED indices for compass and IR direction (16-LED ring, 24° per LED)
        int BNOidx = 19 - round((Compass.orbitDirection + 180) / 24 + 0.5);
        int IRidx = 3 + round((IR.direction + 180) / 24 + 0.5);

        if (BNOidx < 3)
            BNOidx = 3;
        if (IRidx > 18)
            IRidx = 18;

        // Update compass direction LED (red channel)
        Tactics.action.setRGB(lBNOidx, 0, -1, -1); // turn off previous position
        Tactics.action.setRGB(BNOidx, 255, -1, -1); // turn on current position

        // Update IR direction LED (green channel)
        Tactics.action.setRGB(lIRidx, -1, 0, -1); // turn off previous position
        Tactics.action.setRGB(IRidx, -1, 255, -1); // turn on current position

        lIRidx = IRidx;
        lBNOidx = BNOidx;

        // Ball detection indication (blue channel on LEDs 10 and 12)
        if (LDR.ballda)
        {
            Tactics.action.setRGB(10, -1, -1, 255); // ball detected
            Tactics.action.setRGB(12, -1, -1, 255); // ball detected
        }
        else
        {
            Tactics.action.setRGB(10, -1, -1, 0); // no ball
            Tactics.action.setRGB(12, -1, -1, 0); // no ball
        }

        Tactics.action.renderRGBs();
    }
};

// Main robot instance
CodeRobot Karl_GustavIII;

/**
 * @brief Arduino setup function - runs once at startup
 */
void setup()
{
    Karl_GustavIII.initialize();
    Serial.println("Setup done");
}

/**
 * @brief Arduino main loop - runs continuously
 */
void loop()
{
    Karl_GustavIII.system();
}
// End of Karl-GustavIII-finalEM26