# Funktionsübersicht Karl-Gustav III v1.1.1

*Letzte Aktualisierung: 26.02.2026*

## Klassen und Funktionen

| Klasse | Funktionsname | Parameter | Beschreibung der Funktion | Rückgabewert |
|--------|--------------|-----------|--------------------------|--------------|
| CodeCalculate | IR_Calc | void | Verarbeitet die IR-Sensordaten und berechnet Ballposition mittels Vektoraddition | void |
| CodeCalculate | US_Calc | void | Verarbeitet die Ultraschall-Sensordaten | void |
| CodeRead | IR | void | Liest die IR-Sensoren aus | void |
| CodeRead | US | void | Liest die Ultraschallsensoren aus | void |
| CodeRead | Compass | void | Liest den Kompasssensor aus | void |
| CodeRead | LDR | void | Liest die Lichtschranke aus | void |
| CodeRead | Switches | void | Liest die Schalter und Taster aus | void |
| CodeOptical | LED | bool doRGB | Steuert die RGB-LEDs basierend auf Systemstatus | void |
| CodeOptical | doSerial | bool debug | Gibt Debuginformationen über Serial aus | void |
| Codeaction | calculatePID | void | Berechnet die PID-Regelung für die Orientierung | void |
| Codeaction | setMotor | int i, int dir, int speed | Steuert einen einzelnen Motor | void |
| Codeaction | move | float angle, float speed, float aoa | Steuert die omnidirektionale Bewegung | void |
| Codeaction | brake | void | Stoppt alle Motoren | void |
| Codeaction | dribbler | bool an, bool dir | Steuert den Dribbler | void |
| Codeaction | kicker_reset | void | Setzt den Kicker zurück | void |
| Codeaction | kick | void | Löst einen Schuss aus | void |
| CodeTactics | LOP | void | Behandelt Lack of Progress Situationen | void |
| CodeTactics | homing | void | Fährt zurück ins eigene Tor | void |
| CodeTactics | ballanfahrt | void | Steuert die Bewegung zum Ball | void |
| CodeTactics | toranfahrt | void | Steuert die Bewegung zum gegnerischen Tor | void |
| CodeRobot | initialize | void | Initialisiert den Roboter | void |
| CodeRobot | update | void | Aktualisiert Sensordaten (5ms-Zyklus) | void |
| CodeRobot | system | void | Verarbeitet Systemfunktionen (10ms-Zyklus) | void |
| CodeRobot | game | void | Implementiert die Spiellogik | void |

## Parameter-Details

### setMotor
- i: Motorindex (0=hinten, 1=links, 2=rechts, 3=Dribbler)
- dir: Richtung (0=Freilauf, 1=Rechts, 2=Links, 3=Bremsen)
- speed: Geschwindigkeit (0-100)

### move
- angle: Bewegungsrichtung in Grad (-180 bis 180)
- speed: Geschwindigkeit (0-100)
- aoa: Anstellwinkel (-180 bis 180)

### dribbler
- an: Dribbler aktivieren/deaktivieren
- dir: true=Ball anziehen, false=Ball abstoßen