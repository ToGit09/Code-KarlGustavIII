#include <Bot.h>

// I2C Ziel-ID (Beispielwert aus dem Datenblatt)
const int M_ADDR = 0x60;

// Funktion zum Schreiben eines 32-Bit Registers (Shadow-Register 0x80 - 0xAE)
void writeReg32(uint16_t regAddr, uint32_t data)
{
    Wire1.beginTransmission(M_ADDR);

    // 24-Bit Control Word erstellen [6, 7]
    // Bit 23: 0 (Write), Bit 22: 0 (No CRC), Bits 21-20: 01 (32-bit Data)
    Wire1.write(0x10);
    delayMicroseconds(100); // 100µs Verzögerung zwischen Bytes erforderlich [1]

    // Bits 15-8 des Control Words (Memory Page 0) [8]
    Wire1.write(0x00);
    delayMicroseconds(100);

    // Bits 7-0 des Control Words (Register Adresse) [8]
    Wire1.write(regAddr & 0xFF);
    delayMicroseconds(100);

    // Datenbytes senden (LSB zuerst!) [2]
    for (int i = 0; i < 4; i++)
    {
        Wire1.write((data >> (8 * i)) & 0xFF);
        delayMicroseconds(100);
    }

    Wire1.endTransmission();
}

void setup()
{
    Wire1.begin();
    Wire1.setClock(1000000);

    Serial.begin(9600);
    Serial.println("Konfiguriere MCF8316C...");

    // Schritt 1: Grundkonfiguration mit empfohlenen (KI) Standardwerten [9, 10]
    // Diese Werte sind für einen zuverlässigen Start optimiert.
    writeReg32(0x80, 0x64738CA0); // ISD_CONFIG (Start-up Verhalten)
    writeReg32(0x84, 0x0B6807D0); // MOTOR_STARTUP1
    writeReg32(0x88, 0x113181B8); // CLOSED_LOOP1 (PWM Freq, FG etc.)

    // Schritt 2: Maximale Motordrehzahl setzen (Essenzieller Parameter) [4, 11]
    // Register 0x8E (CLOSED_LOOP4). Hier wird z.B. MAX_SPEED definiert.
    // writeReg32(0x8E, 0x000004B0); // MAX_SPEED = 1200 RPM (Beispielwert, anpassen je nach Motor)
    writeReg32(0x8E, 0x00000320); // MAX_SPEED = 800 RPM (Beispielwert, anpassen je nach Motor)
}