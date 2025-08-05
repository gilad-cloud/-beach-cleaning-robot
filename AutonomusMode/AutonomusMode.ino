#include <QMC5883LCompass.h>
QMC5883LCompass compass;
// Define PWM and motor control pins
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
// Define ultrasonic sensor pins
#define trigLeft 8
#define echoLeft 9
#define trigRight 7
#define echoRight 32
// Obstacle avoidance states
#define AVOID_TURNING 0
#define AVOID_STRAIGHT_DRIVE 1
#define AVOID_RETURN 2

// PID Variables
float kp = 15.0, ki = 100.0, kd = 0.0;
float error = 0, error_sum = 0, prev_error = 0;
float P = 0, I = 0, D = 0, PID_PWM = 0;
float I_max = 100, max_sum = 0;
unsigned long lastPIDTime = 0, lastPrintTime = 0;
int leftPWM = 0, rightPWM = 0;
const unsigned long printInterval = 50;

// Obstacle detection and avoidance variables
int avoidState = AVOID_TURNING, clearCounter = 0, obstacleCounter = 0;
unsigned long avoidStartTime = 0, detourStartTime = 0, obstacleStartTime = 0;
int avoidStartAzimuth = 0, avoidReturnAzimuth = 0;

// General control and state variables
String command = "";
unsigned long now_time = 0, start_time = 0, phase_start_time = 0, last_receive_time = 0, return_drive_start_time = 0;
int finalRotation = 0, startAzimuth = 0, returnAzimuth = 0,total_cycles = 0, current_cycle = 0;

enum Phase {
  IDLE,
  DRIVE_1,
  ROTATE_1,
  DRIVE_2,
  ROTATE_180,
  DRIVE_3,
  ROTATE_90,
  DRIVE_4,
  RETURN_TO_START,
  OBSTACLE_AVOIDANCE
  };
Phase current_phase = IDLE, preObstaclePhase = IDLE;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  compass.init();
  // Compass calibration values
  compass.setCalibration(-1455, 923, -1058, 1523, -856, 1012);
  pinMode(trigLeft, OUTPUT); 
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  analogWrite(enA, PWM);
  analogWrite(enB, PWM);
  analogWrite(enC, PWM);
  analogWrite(enD, PWM);
  analogWrite(enE, PWM);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);
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
}

void loop() {
  // Append received character from Serial1 to command string
  if (Serial1.available()) {
    command += (char)Serial1.read();
    last_receive_time = millis();
  }
   // Parse command when complete (e.g., "3000,2") and idle timeout passed
  if (command.length() > 3 && millis() - last_receive_time > 200) {
    if (sscanf(command.c_str(), "%lu,%d", &now_time, &total_cycles) == 2) {
      finalRotation = 90; // Desired final rotation         
      start_time = millis();
      current_cycle = 0;
      current_phase = DRIVE_1;
      compass.read();
      startAzimuth = compass.getAzimuth();
      if (startAzimuth < 0) startAzimuth += 360;
      Serial.print("Starting autonomous cycle | Drive Time: ");
      Serial.print(now_time);
      Serial.print(" ms | Cycles: ");
      Serial.println(total_cycles);
      roller(); // Start roller motor
    }
    command = ""; // Reset command string
  }
   // If all cycles finished, stop roller and exit
  if (current_cycle >= total_cycles){
    stopRoller();
    return;
  }
  // Read compass heading
  compass.read();
  int currentAzimuth = compass.getAzimuth();
  if (currentAzimuth < 0) currentAzimuth += 360;
  // Read ultrasonic distances from both sides
  int distLeft = readUltrasonic(trigLeft, echoLeft);
  int distRight = readUltrasonic(trigRight, echoRight);
  // Check for obstacles only during driving phases
  if (current_phase >= DRIVE_1 && current_phase <= DRIVE_4) {
    if (distLeft < 20 || distRight < 20) obstacleCounter++;
    else obstacleCounter = 0;
    if (obstacleCounter >= 3) {
      Serial.println("Obstacle detected 3 times in a row — entering OBSTACLE_AVOIDANCE mode.");
      obstacleStartTime = millis();
      preObstaclePhase = current_phase;
      current_phase = OBSTACLE_AVOIDANCE;
      stop(); // Stop all motors
      clearCounter = 0; 
      detourStartTime = millis();
      obstacleCounter = 0;
      compass.read();  // Get heading to return to
      avoidStartAzimuth = compass.getAzimuth();
      if (avoidStartAzimuth < 0) avoidStartAzimuth += 360;
      return;
    }
  }
  // Drive sequence of the robot, phase by phase
  switch (current_phase) {
    case DRIVE_1:
      runPIDDrive(startAzimuth, now_time, start_time, ROTATE_1);
      break;
    case ROTATE_1:
      Serial.println("Starting ROTATE_1");
      if (rotateToTarget(currentAzimuth, (startAzimuth + finalRotation) % 360)) {
        phase_start_time = millis();
        current_phase = DRIVE_2;
      }
      break;
    case DRIVE_2:
      if (millis() - phase_start_time < 1000) forward();
      else {
        stop();
        current_phase = ROTATE_180;
        }
      break;
    case ROTATE_180:
      Serial.println("Starting ROTATE_180");
      if (rotateToTarget(currentAzimuth, (startAzimuth + 180) % 360)) {
        return_drive_start_time = millis();
        compass.read();
        returnAzimuth = compass.getAzimuth();
        if (returnAzimuth < 0) returnAzimuth += 360;
        current_phase = DRIVE_3;
      }
      break;
    case DRIVE_3:
      runPIDDrive(returnAzimuth, now_time - 1000, return_drive_start_time, ROTATE_90);
      break;
    case ROTATE_90:
      Serial.println("Starting ROTATE_90");
      if (rotateToTarget(currentAzimuth, (startAzimuth + 90) % 360)) {
        phase_start_time = millis(); current_phase = DRIVE_4;
      }
      break;
    case DRIVE_4:
      if (millis() - phase_start_time < 1000) forward();
      else {
        stop();
        current_phase = RETURN_TO_START;
        }
      break;
    case RETURN_TO_START:
      Serial.println("Starting RETURN_TO_START");
      if (rotateToTarget(currentAzimuth, startAzimuth)) {
        current_cycle++;
        if (current_cycle < total_cycles) {
          current_phase = DRIVE_1;
          start_time = millis();
          compass.read(); startAzimuth = compass.getAzimuth();
          if (startAzimuth < 0) startAzimuth += 360;
        } else {
          current_phase = IDLE;
          Serial.println("All cycles finished!");
        }
      }
      break;
    case OBSTACLE_AVOIDANCE:
      obstacleAvoidance(currentAzimuth);
      break;
  }
}

void runPIDDrive(int refAzimuth, unsigned long duration, unsigned long startTime, Phase nextPhase) {
  if (millis() - startTime < duration) {
    // Get current azimuth.
    compass.read();
    int currentAzimuth = compass.getAzimuth();
    if (currentAzimuth < 0) currentAzimuth += 360;
    // Calculate error
    error = refAzimuth - currentAzimuth;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    if (dt < 0.001) dt = 0.001;

    error_sum += error;
    max_sum = (I_max / (ki * dt));
    if (error_sum > max_sum) error_sum = max_sum;
    
    P = kp * error;
    I = ki * error_sum * dt;
    D = kd * (error - prev_error) / dt;
    PID_PWM = constrain(P + I + D, -120, 120);

    prev_error = error; lastPIDTime = now;
    
    // Apply PID correction to motor speeds
    int basePWM = 170;
    leftPWM = constrain(basePWM + PID_PWM, 0, 255);
    rightPWM = constrain(basePWM - PID_PWM, 0, 255);
    // Drive motors forward with PID compensation
    analogWrite(enA, rightPWM); analogWrite(enD, rightPWM);
    analogWrite(enB, leftPWM); analogWrite(enC, leftPWM);
    digitalWrite(inA1, HIGH); digitalWrite(inA2, LOW);
    digitalWrite(inD3, HIGH); digitalWrite(inD4, LOW);
    digitalWrite(inC1, LOW); digitalWrite(inC2, HIGH);
    digitalWrite(inB3, LOW); digitalWrite(inB4, HIGH);
    // Debug output for tuning PID
    if (now - lastPrintTime >= printInterval) {
      lastPrintTime = now;
      Serial.print("[PID] Time: "); Serial.print(now);
      Serial.print(" ms | Azimuth: "); Serial.print(currentAzimuth);
      Serial.print(" | Error: "); Serial.print(error);
      Serial.print(" | P: "); Serial.print(P);
      Serial.print(" | I: "); Serial.print(I);
      Serial.print(" | D: "); Serial.print(D);
      Serial.print(" | PWM L: "); Serial.print(leftPWM);
      Serial.print(" | PWM R: "); Serial.println(rightPWM);
    }
  } else {
    stop();
    error_sum = 0;
    current_phase = nextPhase;
  }
}
// Obstacle avoidance state machine
void obstacleAvoidance(int currentAzimuth) {
  switch (avoidState) {
    case AVOID_TURNING:
      // Turn until path ahead is clear (detour turning phase)
      analogWrite(enA, 255); analogWrite(enB, 255);
      analogWrite(enC, 255); analogWrite(enD, 255);
      if (readUltrasonic(trigLeft, echoLeft) < 20 || readUltrasonic(trigRight, echoRight) < 20) {
        clearCounter = 0; right();
      } else {
        clearCounter++;
        if (clearCounter >= 3) {
          stop(); // Path is clear
          avoidState = AVOID_STRAIGHT_DRIVE;
          avoidStartTime = millis();
        }
      }
      break;
    case AVOID_STRAIGHT_DRIVE:
      // Drive straight forward for a fixed time after detour
      analogWrite(enA, 255); analogWrite(enB, 255);
      analogWrite(enC, 255); analogWrite(enD, 255);
      if (millis() - avoidStartTime < 1000) forward();
      else {
        stop(); avoidReturnAzimuth = avoidStartAzimuth;
        avoidState = AVOID_RETURN;
      }
      break;
    case AVOID_RETURN:
      // Re-align to original azimuth after avoidance
      compass.read(); currentAzimuth = compass.getAzimuth();
      if (currentAzimuth < 0) currentAzimuth += 360;
      if (rotateToTarget(currentAzimuth, avoidReturnAzimuth)) {
        stop();
        // Compensate timing so path remains accurate
        unsigned long avoidDuration = millis() - obstacleStartTime;
        start_time += avoidDuration;
        phase_start_time += avoidDuration; // Resume previous phase
        return_drive_start_time += avoidDuration;
        current_phase = preObstaclePhase;
        avoidState = AVOID_TURNING; obstacleCounter = 0;
      }
      break;
  }
}
// Ultrasonic distance measurement in cm
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration == 0 ? 999 : duration * 0.034 / 2;
}

bool rotateToTarget(int currentAzimuth, int targetAzimuth) {
  // Calculate azimuth error
  int error = targetAzimuth - currentAzimuth;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  // If error is large enough, keep rotating
  if (abs(error) > 2) {
    analogWrite(enA, 255); analogWrite(enD, 255);
    analogWrite(enB, 255); analogWrite(enC, 255);
    (error > 0) ? right() : left(); // Rotate right if error positive, left if negative
    // Not yet at the target, return false
    return false;
  } else {
    stop();
    Serial.print("Finished rotation. Current Azimuth: ");
    Serial.println(currentAzimuth);
    return true;
  }
}

void forward() {
  analogWrite(enA, 200); analogWrite(enD, 200);
  analogWrite(enB, 200); analogWrite(enC, 200);
  digitalWrite(inA1, HIGH); digitalWrite(inA2, LOW);
  digitalWrite(inD3, HIGH); digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW);  digitalWrite(inC2, HIGH);
  digitalWrite(inB3, LOW);  digitalWrite(inB4, HIGH);
}
void left() {
  digitalWrite(inA1, HIGH); digitalWrite(inA2, LOW);
  digitalWrite(inD3, HIGH); digitalWrite(inD4, LOW);
  digitalWrite(inC1, HIGH); digitalWrite(inC2, LOW);
  digitalWrite(inB3, HIGH); digitalWrite(inB4, LOW);
}
void right() {
  digitalWrite(inA1, LOW); digitalWrite(inA2, HIGH);
  digitalWrite(inD3, LOW); digitalWrite(inD4, HIGH);
  digitalWrite(inC1, LOW); digitalWrite(inC2, HIGH);
  digitalWrite(inB3, LOW); digitalWrite(inB4, HIGH);
}
void stop() {
  digitalWrite(inA1, LOW); digitalWrite(inA2, LOW);
  digitalWrite(inD3, LOW); digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW); digitalWrite(inC2, LOW);
  digitalWrite(inB3, LOW); digitalWrite(inB4, LOW);
}
void roller() {
  digitalWrite(inE1, LOW); 
  digitalWrite(inE2, HIGH);
}
void stopRoller() {
  digitalWrite(inE1, LOW); 
  digitalWrite(inE2, LOW);
}
