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

String command = ""; 

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
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
  char serial_val;
  
  if (Serial1.available()) {
    serial_val = Serial1.read();
    Serial.print("READING: ");
    Serial.println(serial_val);

    if (serial_val >= 'A' && serial_val <= 'Z') {
      command += serial_val;
    }

    if (command.length() == 4) {
      Serial.print("Received Command: ");
      Serial.println(command);
      
      switch (command.charAt(0)) { 
        case 'S': 
          if (command == "STOP") {
            stop();
          }
          break;
        case 'F': 
          if (command == "FWRD") {
            forward();
            roller();
          }
          break;
        case 'B': 
          if (command == "BKWD") {
            backward();
            roller();
          }
          break;
        case 'L': 
          if (command == "LEFT") {
            left();
            roller();
          }
          break;
        case 'R': 
          if (command == "RGHT") {
            right();
            roller();
          }
          break;
      }
      command = "";
    }
  }
}

void forward() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inD3, HIGH);
  digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, HIGH);
}

void backward() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inD3, LOW);
  digitalWrite(inD4, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inB3, HIGH);
  digitalWrite(inB4, LOW);
}

void left() {
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inD3, HIGH);
  digitalWrite(inD4, LOW);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, LOW);
  digitalWrite(inB3, HIGH);
  digitalWrite(inB4, LOW);
}

void right() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inD3, LOW);
  digitalWrite(inD4, HIGH);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, HIGH);
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, HIGH);
}

void stop() {
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inD3, LOW);
  digitalWrite(inD4, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, LOW);
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, LOW);
  digitalWrite(inE1, LOW);
  digitalWrite(inE2, LOW);
}
void roller() {
  digitalWrite(inE1, LOW);
  digitalWrite(inE2, HIGH);
}
