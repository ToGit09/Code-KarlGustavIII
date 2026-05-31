#ifndef ACTION_H
#define ACTION_H

#ifndef BOT_H
#include <Bot.h>
#endif

#define LED_BRIGHTNESS 8
#define LED_FPS 24

extern RGB_event emptyrgb;

class Codeaction
{
public:
    Adafruit_NeoPixel RGBs = Adafruit_NeoPixel(LED_LENGTH, LED_PORT_DATA, NEO_GRB + NEO_KHZ800);
    RGB_event rgb;
    elapsedMillis RGBTimer;
    elapsedMillis KickerTimer; // Kicker Timer
    float PIDKorrektur_memory;
    float PID_last_Abweichung = 0;
    float PID_last = 0;
    float sqrt3 = sqrtf(3.0);

    int DribblerStdSpeed = 80;
    bool DribblerStdDir = true;

    /**
     * @deprecated
     * Die Funktion ist veraltet und soll nicht mehr benutzt werden.
     * Sie wird durch newPID ersetzt.
     */
    float calculatePID(compass_sensor_event compass, movement_event movement);

    /**
     * Berechnet den PID-Wert basierend auf der Abweichung
     * zwischen der aktuellen Kompasswinkel und der Zielwinkel.
     *
     * @param movement   Ein Objekt vom Typ movement_event,
     *                     welches die aktuellen Kompasswinkel und die Zielwinkel
     *                     enthält.
     *
     * @return          Ein float-Wert, der den PID-Wert darstellt.
     */
    float newPID(movement_event movement);

    /**
     * Berechnet die Motorgeschwindigkeiten anhand des Movement-Events
     * @param event: Movement-Event mit Winkel, Geschwindigkeit und Anstellwinkel
     */
    void move(movement_event event);

    /**
     * @brief Setzt die Motoren
     * @param Motor1: Geschwindigkeit Motor 1 (-100 bis 100)
     * @param Motor2: Geschwindigkeit Motor 2 (-100 bis 100)
     * @param Motor3: Geschwindigkeit Motor 3 (-100 bis 100)
     * @param Motor3: Geschwindigkeit Motor 4 (-100 bis 100)
     *
     * Setzt die Motoren auf die gegebenen Geschwindigkeiten.
     *
     * @note Die Geschwindigkeiten werden in eine positive Zahl umgerechnet (0-100) und die Richtung wird automatisch berechnet.
     */
    void setMotors(int Motor1, int Motor2, int Motor3, int Motor4);

    /**
     * Alle Motoren(FAHREN) stoppen
     *
     * Diese Funktion sollt aufgerufen werden, wenn der Roboter
     * anhalten soll oder wenn er nicht mehr fahren soll.
     * sie setzt alle Motoren auf 0 und ignoriert den Dribbler.
     */
    void brake(void);

    /** 
     * @brief setzt den Dribbler BLDC-Motor über den Dribbler Stecker
     * 
     * @param speed: Setzt die Gschwindigkeit des Dribblers (0-100 %)
     * @param dir: Setzt die Richtung des Dribblers (True = Ball wird angezogen)
     * @note Dribbler kann nur eine Richtung, dir wird auf true gesetzt
     * @note Aktuell macht die Funktion nichts, da der Dribbler Treiber noch nicht implementiert ist
     */
    void dribbler(int speed, bool dir = true);

    /**
     * @brief macht den Dribbler mit den letzten gesetzten Werten an/aus 
     * @note Wenn dribbler(int, bool) noch nicht benutzt wurde werden standard werte benutzt
     */
    void dribbler(bool an);

    // lädt den kicker auf
    void kicker_reset(void);

    // Schießt den Ball
    void kick(void);

    // Initialisiert the RGB strip and runs a short animation.
    // This function should be called once after the Codeaction object has been created.
    void initRGBs(void);

    // RGBs setzen
    RGB_event setRGB(int i, int r, int g, int b);

    // RGBs rendern
    // @param RGBinput: optional: RGB_event mit den RGB Werten die gesetzt werden sollen
    void renderRGBs(RGB_event RGBinput = emptyrgb);
};

#endif