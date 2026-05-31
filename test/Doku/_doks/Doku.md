# Karl-Gustav III v1.1.1 - Technische Dokumentation

*Letzte Aktualisierung: 26.02.2026*

## 1. Überblick
Der Karl-Gustav III ist ein autonomer Roboterfußball-Roboter. Der Code steuert sämtliche Sensoren und Aktoren und implementiert verschiedene Spielstrategien.

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

### 2.2 IR-Ballortung

Die Ballposition wird durch Vektoraddition der IR-Sensorwerte ermittelt:

```cpp
void IR_Calc(void) {
    float Ballx = 0;
    float Bally = 0;
    
    // Vektoraddition aller Sensorwerte
    for (int i = 0; i < 8; i++) {
        if (IR_values[i] >= IR_Schwelle) {
            // Umrechnung in kartesische Koordinaten
            Bally += cosf((i * 45) * PI / 180) * IR_values[i];
            Ballx += sinf((i * 45) * PI / 180) * IR_values[i];
        }
    }
    
    // Ballrichtung berechnen
    IR_Direction = atan2f(Ballx, Bally) * 180 / PI;
    
    // Ballsektor bestimmen
    IR_Heading = int(round((IR_Direction + 180) / (360 / IR_Range)));
}
```

Mathematische Schritte:
1. Für jeden Sensor i (0-7):
   - Position: 45° * i
   - x_i = IR_values[i] * sin(45° * i)
   - y_i = IR_values[i] * cos(45° * i)

2. Vektoraddition:
   - x_gesamt = Σ x_i
   - y_gesamt = Σ y_i

3. Winkelberechnung:
   - Ballrichtung = atan2(y_gesamt, x_gesamt)
   - Normalisierung auf -180° bis 180°

### 2.3 PID-Regelung

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

### 2.4 Ultraschall-Navigation

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

## 3. Hauptklassen und ihre Funktionen

### 3.1 CodeCalculate
Verarbeitet die Rohdaten der Sensoren:

```cpp
class CodeCalculate {
public:
    void IR_Calc(void) {
        // Berechnet Ballposition aus IR-Sensorwerten
        float Ballx = 0;
        float Bally = 0;
        
        // Direction berechnen
        for (int i = 0; i < 8; i++) {
            if (IR_values[i] >= IR_Schwelle) {
                Bally += cosf((i * 45) * PI / 180) * IR_values[i];
                Ballx += sinf((i * 45) * PI / 180) * IR_values[i];
            }
        }
        
        IR_Direction = atan2f(Ballx, Bally) * 180 / PI;
    }
}
```

### 3.2 CodeAction
Steuert die Bewegungen des Roboters:

```cpp
class Codeaction {
public:
    void move(float angle, float speed, float aoa) {
        // Winkel in x und y Bewegung umrechnen
        float angleRad = angle * PI / 180;
        float x = cosf(angleRad) * speed;
        float y = sinf(angleRad) * speed;
        
        // Movement berechnen
        int Motor0 = (int)(x + turn);
        int Motor1 = (int)((-0.5 * x) - (sqrt3 / 2 + y) + turn);
        int Motor2 = (int)((-0.5 * x) + (sqrt3 / 2 + y) + turn);
    }
}
```

### 3.3 CodeTactics
Implementiert die Spielstrategien:

```cpp
class CodeTactics {
public:
    void ballanfahrt(void) {
        if (rt_IR_Heading == 4) // Ball genau hinter dem Roboter
        {
            if (rt_US_offsetx > 0)
                action.move(125, stdSpeed, 0);
            else
                action.move(-125, stdSpeed, 0);
        }
    }
    
    void toranfahrt(void) {
        if (rt_OrbitDirection >= GoalTargetAngel - 2 && 
            rt_OrbitDirection <= GoalTargetAngel + 2) 
        {
            if (rt_US_Vorne > 35) {
                // Wenn weiter weg vom Tor, zum Tor ausrichten und schießen
                float tmp_offX = rt_US_offsetx;
                float tmp_offY = rt_US_offsety + 35;
                float sy_dir = atan2f(tmp_offX, tmp_offY) * 180 / PI;
                
                if (rt_OrbitDirection >= GoalTargetAngel - 5 && 
                    rt_OrbitDirection <= GoalTargetAngel + 5)
                    action.kick();
            }
        }
    }
}
```

## 4. PID-Regelung

Die Orientierungsregelung erfolgt über einen PID-Regler:

```cpp
void calculatePID(void) {
    float PIDKorrektur = OrbitDirection - angleOfAttack;
    
    PID_I = PID_I + PIDKorrektur;
    
    if (PIDKorrektur > -(PID_I_threshhold) && 
        PIDKorrektur < PID_I_threshhold)
        PID_I = 0;
        
    PID_P = PIDKorrektur * PID_P_Multiplier;
    PID_I = PID_I * PID_I_Multiplier;
    PID_D = (PIDKorrektur - PIDKorrektur_memory) * PID_D_Multiplier;
    
    PID = PID_P + PID_I + PID_D;
}
```

## 5. Hauptschleife

Der Roboter arbeitet in drei Hauptschleifen:

1. Update (5ms):
```cpp
void update(void) {
    Read.IR();
    Read.Compass();
    Read.LDR();
    Tactics.action.calculatePID();
    Tactics.action.kicker_reset();
}
```

2. System (10ms):
```cpp
void system(void) {
    if (LoopTimer >= LoopTiming) {
        // Sensordaten kopieren
        // Benutzereingaben verarbeiten
        // Debug-Funktionen
    }
}
```

3. Spiellogik:
```cpp
void game(void) {
    if (LOPTimer >= LOP_TimerLimit)
        Tactics.LOP();
    else if (rt_LDR_Ballda && rt_IR_Heading == 0 && !rt_IR_unreliable)
        Tactics.toranfahrt();
    else if (!rt_IR_unreliable)
        Tactics.ballanfahrt();
    else
        Tactics.homing();
}
```

## 6. Debug-Funktionen

Der Code enthält umfangreiche Debug-Möglichkeiten:

```cpp
void doSerial(bool debug = false) {
    if (!debug) {
        Serial.println("Bodensee Devils - Karl_GustavIII - " + version);
        Serial.print("TimeStamp: ");
        Serial.print(totalTime);
        // ... weitere Statusausgaben
    }
}
```

RGB-LEDs zeigen wichtige Statusinformationen:
- Kalibrierungsstatus (LEDs 0,1)
- Kompassrichtung (LED 7)
- Systemstatus (LED 8)
- Ballbesitz (LED 9)
- IR-Status (LED 6)