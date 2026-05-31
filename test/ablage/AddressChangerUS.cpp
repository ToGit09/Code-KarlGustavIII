#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Bot.h>
#define DEFAULT_I2C Wire1

int currentAddress = 0x70;
int newAddress = 0x75;

void changeAddress() {
    DEFAULT_I2C.beginTransmission(currentAddress);
    DEFAULT_I2C.write(0x00);
    DEFAULT_I2C.write(0xA0);
    DEFAULT_I2C.endTransmission();
    DEFAULT_I2C.beginTransmission(currentAddress);
    DEFAULT_I2C.write(0x00);
    DEFAULT_I2C.write(0xAA);
    DEFAULT_I2C.endTransmission();
    DEFAULT_I2C.beginTransmission(currentAddress);
    DEFAULT_I2C.write(0x00);
    DEFAULT_I2C.write(0xA5);
    DEFAULT_I2C.endTransmission();
    DEFAULT_I2C.beginTransmission(currentAddress);
    DEFAULT_I2C.write(0x00);
    DEFAULT_I2C.write(newAddress << 1);
    DEFAULT_I2C.endTransmission();
}

void setup() {
    delay(300);
    Serial.begin(115200);
    DEFAULT_I2C.begin();
    DEFAULT_I2C.beginTransmission(currentAddress);
    int result = DEFAULT_I2C.endTransmission();
    if(result == 0) {
        Serial.println("Sensor found!");
        changeAddress();
    }else if(result == 4) {
        Serial.println("Unknown Error!");
    }else {
        Serial.println("No Sensor found!");
    }
}

void loop() {

}