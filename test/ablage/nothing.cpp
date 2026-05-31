#include <Arduino.h>
#include <Bot.h>

void setup() {
    Serial.begin(9600);
}

void loop() {
    Serial.print("Alive");
    delay(100);
}