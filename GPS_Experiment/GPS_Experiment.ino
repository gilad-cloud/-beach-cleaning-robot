#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>

QMC5883LCompass compass;
TinyGPSPlus gps;

// Target GPS coordinates
float targetLat = 32.001010;
float targetLon = 34.770870;

float currentLat = 0.0, currentLon = 0.0;
int targetAzimuth = 0;

#define PWM 255
#define enA 2
#define enB 4
#define enC 5
#define enD 3
#define enE 6
#define inA1 50
#define inA2 51
#define inD3 48
#define inD4 49
#define inB3 12
#define inB4 13
#define inC1 11
#define inC2 10
#define inE1 44
#define inE2 42

unsigned long lastGPSUpdate = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(9600);           
  Serial2.begin(9600);          
  compass.init();
  compass.setCalibration(-1455, 923, -1058, 1523, -856, 1012);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(enE, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inD3, OUTPUT);
  pinMode(inD4, OUTPUT);
  pinMode(inB3, OUTPUT);
  pinMode(inB4, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(inE1, OUTPUT);
  pinMode(inE2, OUTPUT);

  analogWrite(enA, PWM);
  analogWrite(enD, PWM);
  analogWrite(enB, PWM);
  analogWrite(enC, PWM);
}

void loop() {
  // Read data from GPS
  while (Serial2.available()) gps.encode(Serial2.read());

  unsigned long currentTime = millis();

  // Update location and bearing every second
  if (gps.location.isValid() && currentTime - lastGPSUpdate > 1000) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    targetAzimuth = calculateBearing(currentLat, currentLon, targetLat, targetLon);
    lastGPSUpdate = currentTime;
  }

  // Stop if target is reached
  float dist = haversine(currentLat, currentLon, targetLat, targetLon);
  if (dist < 5.0) {
    stop();
    Serial.println("✅ Arrived at destination!");
    return;
  }

  // Drive forward
  forward();

  // Debug print every second
  if (currentTime - lastPrintTime >= 1000) {
    compass.read();
    int currentAzimuth = compass.getAzimuth();
    if (currentAzimuth < 0) currentAzimuth += 360;

    Serial.print("GPS: ");
    Serial.print(currentLat, 6);
    Serial.print(", ");
    Serial.print(currentLon, 6);
    Serial.print(" | Distance: ");
    Serial.print(dist);
    Serial.print(" | Target Bearing: ");
    Serial.print(targetAzimuth);
    Serial.print(" | Compass Heading: ");
    Serial.println(currentAzimuth);
    lastPrintTime = currentTime;
  }
}

// Drive forward
void forward() {
  digitalWrite(inA1, HIGH); digitalWrite(inA2, LOW);
  digitalWrite(inD3, HIGH); digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW);  digitalWrite(inC2, HIGH);
  digitalWrite(inB3, LOW);  digitalWrite(inB4, HIGH);
}

// Stop all motors
void stop() {
  digitalWrite(inA1, LOW); digitalWrite(inA2, LOW);
  digitalWrite(inD3, LOW); digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW); digitalWrite(inC2, LOW);
  digitalWrite(inB3, LOW); digitalWrite(inB4, LOW);
  digitalWrite(inE1, LOW); digitalWrite(inE2, LOW);
}

// Calculate distance between two GPS points (in meters)
float haversine(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1); lat2 = radians(lat2);
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// Calculate bearing (azimuth) from current to target point
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  float bearing = degrees(atan2(y, x));
  return fmod((bearing + 360.0), 360.0);
}
