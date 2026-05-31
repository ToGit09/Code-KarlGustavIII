#include "ESC.h"

ESCC ESC;

// ─── Private ─────────────────────────────────────────────────

void ESCC::writeTicks(uint32_t ticks) {
    analogWrite(_pin, ticks);
}

// ─── Public ──────────────────────────────────────────────────
void ESCC::init_Power(){
    writeTicks(ESC_TICKS_MAX);
    delay(3000);   // Wartezeit: ESC einschalten und Pieptöne abwarten

    // Phase 2: Auf Minimum wechseln → ESC speichert Min/Max und bestätigt
    //          per Ton. Gleichzeitig wird der ESC damit gearmt.
    writeTicks(ESC_TICKS_MIN);
    delay(2000);   // Bestätigungston abwarten

    // Kalibrierung + Arming abgeschlossen
    _armed = true;
}


void ESCC::init(uint8_t pin) {
    _pin   = pin;
    _armed = false;

    pinMode(_pin, OUTPUT);
    analogWriteFrequency(_pin, ESC_FREQ_HZ);
    analogWriteResolution(16);
}

void ESCC::set(int speed) {
    if (!_armed) return;

    speed = constrain(speed, 0, 100);

    // Lineares Mapping: 0 → ESC_TICKS_MIN, 100 → ESC_TICKS_MAX
    uint32_t ticks = map(speed, 0, 100, ESC_TICKS_MIN, ESC_TICKS_MAX);
    writeTicks(ticks);
}

void ESCC::stop() {
    if (!_armed) return;
    writeTicks(ESC_TICKS_MIN);
}

bool ESCC::armed() {
    return _armed;
}
