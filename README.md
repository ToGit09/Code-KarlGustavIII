# Karl-Gustav III

*Das Readme wurde von KI erstellt und kann fehler enthalten/veraltet sein*
*In den Code schauen lohnt sich also :)*

## 1. Projektstruktur

- `src/` – Hauptcode der Firmware
- `lib/` – Projektbibliotheken
- `test/` – TestCode und Ablage
- `platformio.ini` – Build-/Board-Konfiguration


## 2. Mathematische Grundlagen

### 2.1 Omnidirektionale Bewegung

Die Bewegungssteuerung basiert auf einem dreieckigen Antriebssystem mit 120° Motoranordnung:

```cpp
/**
 * Berechnung der Motorgeschwindigkeiten für omnidirektionale Bewegung
 * @param angle: Bewegungsrichtung in Grad (-180 bis 180)
 * @param speed: Geschwindigkeit (0-100)
 * @param aoa: Anstellwinkel (-180 bis 180)
 */
void move(float angle, float speed, float aoa) {
    // Winkel in Radiant umrechnen
    float angleRad = angle * PI / 180;
    
    // Vektorzerlegung in x/y Komponenten
    float x = cosf(angleRad) * speed;
    float y = sinf(angleRad) * speed;
    
    // PID für Orientierung
    angleOfAttack = aoa;
    float turn = PID * PID_Multiplikator;
    
    // Motorgeschwindigkeiten berechnen
    int Motor0 = (int)(x + turn);                            // 180° (hinten)
    int Motor1 = (int)((-0.5 * x) - (sqrt3 / 2 * y) + turn); // -60° (links vorne)
    int Motor2 = (int)((-0.5 * x) + (sqrt3 / 2 * y) + turn); // 60° (rechts vorne)
}
```

Die mathematische Herleitung:
1. Der Einheitsvektor für jeden Motor:
   - Motor0: [1, 0]
   - Motor1: [-0.5, -√3/2]
   - Motor2: [-0.5, √3/2]

2. Die Projektion des Bewegungsvektors [x, y] auf diese Einheitsvektoren ergibt die Motorgeschwindigkeiten.

### 2.2 PID-Regelung

Der PID-Regler für die Orientierungskontrolle:

```cpp
void calculatePID(void) {
    // Aktueller Fehler
    float PIDKorrektur = OrbitDirection - angleOfAttack;
    
    // Integral berechnen
    PID_I = PID_I + PIDKorrektur;
    
    // Anti-Windup
    if (PIDKorrektur > -(PID_I_threshhold) && 
        PIDKorrektur < PID_I_threshhold)
        PID_I = 0;
    
    if (PID_I > PIDpa_IlimitMax)
        PID_I = PIDpa_IlimitMax;
    else if (PID_I < PIDpa_IlimitMin)
        PID_I = PIDpa_IlimitMin;
    
    // PID Komponenten
    PID_P = PIDKorrektur * PID_P_Multiplier;  // Proportional
    PID_I = PID_I * PID_I_Multiplier;         // Integral
    PID_D = (PIDKorrektur - PIDKorrektur_memory) 
            * PID_D_Multiplier;                // Differential
    
    // Gesamtregelung
    PID = PID_P + PID_I + PID_D;
}
```

Mathematische Details:
1. P-Anteil:
   - Direkt proportional zum Fehler
   - P = Fehler * P_Multiplier (typisch 4.0)
   - Schnelle Reaktion, aber mögliches Überschwingen

2. I-Anteil:
   - Summiert den Fehler über Zeit
   - Begrenzt auf [-105, 105]
   - Eliminiert bleibende Regelabweichung
   - I_Multiplier typisch 1.4

3. D-Anteil:
   - Rate der Fehleränderung
   - Dämpft Oszillationen
   - D_Multiplier typisch 15.0

### 2.3 Ultraschall-Navigation

Die Positionsbestimmung verwendet trigonometrische Korrekturen:

```cpp
void US_Calc(void) {
    // Kompasskorrektur der Messwerte
    US_Rechts = US_Rechts * cosf(OrbitDirection * 180 / PI);
    US_Links = US_Links * cosf(OrbitDirection * 180 / PI);
    US_Vorne = US_Vorne * sinf(OrbitDirection * 180 / PI);
    US_Hinten = US_Hinten * sinf(OrbitDirection * 180 / PI);
    
    // Relative Position berechnen
    US_offsetx = (US_Rechts - US_Links) / 2;
    US_offsety = (US_Vorne - US_Hinten) / 2;
}
```

Die Positionsberechnung berücksichtigt:
1. Kompasswinkel-Korrektur:
   - Messwerte werden entsprechend der aktuellen Orientierung projiziert
   - Verwendung von sin/cos für Vektorzerlegung

2. Relative Position:
   - X-Offset: Mittelwert der seitlichen Differenz
   - Y-Offset: Mittelwert der vor/zurück Differenz
