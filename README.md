# Karl-Gustav III

Kurzübersicht für den Einstieg in den Code

## Projektstruktur

- `src/` – Hauptcode der Firmware
- `include/` – Header und zentrale Definitionen
- `lib/` – Projektbibliotheken
- `test/` – Tests und testnahe Quellen
- `platformio.ini` – Build-/Board-Konfiguration (`esp32dev`)

## Wichtige Befehle

```bash
pio run
pio run -t upload
pio test
```

## Einstieg im Code

1. Starte bei `src/main.cpp`.
2. Folge von dort den Includes in `include/` und Modulen in `lib/`.
3. Nutze `test/` für Referenz- und Testcode.
