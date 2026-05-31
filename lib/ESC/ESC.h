#pragma once
#include <Arduino.h>

// Klasse zur Steuerung eines sensorlosen Brushless-ESC über PWM.
//
// PWM-Format (Standard-Servo-Protokoll):
//   ESC_MIN_US  = 1000 µs → Motorstopp / Minimum
//   ESC_MAX_US  = 2000 µs → Vollgas    / Maximum
//   Frequenz    = 50 Hz
//
// init() führt automatisch eine einmalige Kalibrierung durch und
// armt den ESC anschließend. Danach kann set() direkt verwendet werden.

#define ESC_MIN_US   1000   // µs – Stopp / Arming-Wert
#define ESC_MAX_US   2000   // µs – Vollgas
#define ESC_FREQ_HZ    50   // PWM-Frequenz in Hz

// Pulse-Breiten für 16-bit analogWrite bei 50 Hz:
//   Periode = 1/50 Hz = 20 000 µs
//   Wert = (Pulsbreite / 20000) * 65535
#define ESC_TICKS_MIN  ((uint32_t)((ESC_MIN_US / 20000.0f) * 65535))  // ≈ 3277
#define ESC_TICKS_MAX  ((uint32_t)((ESC_MAX_US / 20000.0f) * 65535))  // ≈ 6554

class ESCC {
    private:
        uint8_t _pin;
        bool    _armed = false;

        // Schreibt einen rohen Tick-Wert direkt auf den PWM-Ausgang.
        void writeTicks(uint32_t ticks);

    public:
        // Initialisiert den PWM-Pin, kalibriert den ESC einmalig und armt ihn.
        // pin: der Teensy-Pin, an dem das ESC-Signalkabel hängt.
        // Nach init() ist der ESC bereit und gestoppt (set(0)).
        void init(uint8_t pin);
        void init_Power();

        // Setzt die Motorgeschwindigkeit.
        // speed: 0 (Stopp) bis 100 (Vollgas), Werte außerhalb werden begrenzt.
        void set(int speed);

        // Stoppt den Motor sofort (entspricht set(0)).
        void stop();

        // Gibt zurück, ob der ESC erfolgreich gearmt wurde.
        bool armed();
};
