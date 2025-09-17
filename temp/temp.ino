#include "max6675.h"

// Define digital pins for software SPI
#define SCK 6   // Clock
#define SO  4   // Data
#define CS1 5   // Chip Select for first MAX6675
#define CS2 3   // Chip Select for second MAX6675

// Define sensor pin for soil moisture sensor
const int sensorPin = A0;

// Create MAX6675 objects with manual pin control
MAX6675 thermo1(SCK, CS1, SO);
MAX6675 thermo2(SCK, CS2, SO);

unsigned long previousMillis = 0;
const long interval = 500; // Interval for sensor reading

void setup() {
    Serial.begin(9600);
    Serial.println("MAX6675 Dual Thermocouple & Soil Moisture Sensor Test");
    delay(1000);
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Read temperatures from both sensors
        float temp1 = thermo1.readCelsius();
        float temp2 = thermo2.readCelsius();
        
        // Read soil moisture sensor value
        int sensorValue = analogRead(sensorPin);
        
        // Print readings
        Serial.print("Thermocouple 1: ");
        Serial.print(temp1);
        Serial.print(" °C  |  ");
        
        Serial.print("Thermocouple 2: ");
        Serial.print(temp2);
        Serial.print(" °C  |  ");
        
        Serial.print("Soil Moisture Value: ");
        Serial.println(sensorValue);
    }
}


