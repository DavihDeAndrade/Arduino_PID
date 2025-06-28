#include <PID_v1.h>

// Commands to change parameters in serial monitor
// kp 8.5
// ki 0.3
// kd 1.1
// set 15.0

// Sensor pins
const int TRIG_UPPER = 9;
const int ECHO_UPPER = 10;

// H-Bridge
const int PUMP_IN1 = 5;
const int PUMP_IN2 = 6;
const int PUMP_ENA = 3;

// PID variables
double currentLevel = 0;
double setpoint = 10.0;  // Desired level in cm
double controlSignal = 0;

// PID gains (adjustable)
double Kp = 10.0, Ki = 0.5, Kd = 1.0;
PID pid(&currentLevel, &controlSignal, &setpoint, Kp, Ki, Kd, DIRECT);

// Time control
unsigned long lastReading = 0;
const unsigned long interval = 100;

void setup() {
  Serial.begin(9600);
  // Sensor
  pinMode(TRIG_UPPER, OUTPUT);
  pinMode(ECHO_UPPER, INPUT);
  // H-Bridge
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(PUMP_ENA, OUTPUT);
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_ENA, 0);
  // PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  pid.SetSampleTime(interval);
  Serial.println("Type commands: kp 10.0 | ki 0.5 | kd 1.0 | set 15");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReading >= interval) {
    lastReading = now;
    currentLevel = readLevel(TRIG_UPPER, ECHO_UPPER);
    pid.Compute();
    applyPWM(controlSignal);
    Serial.print("Level: ");
    Serial.print(currentLevel, 1);
    Serial.print(" cm | Output: ");
    Serial.print(controlSignal, 1);
    Serial.print(" | Kp=");
    Serial.print(Kp);
    Serial.print(" Ki=");
    Serial.print(Ki);
    Serial.print(" Kd=");
    Serial.println(Kd);
  }
  readSerialCommands();
}

// Read level with ultrasonic sensor
float readLevel(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  float distance = duration * 0.0343 / 2.0;       // in cm
  return distance;
}

// Apply control to pump
void applyPWM(double pwm) {
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_ENA, (int)pwm);
}

// Interpret Serial commands
void readSerialCommands() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("kp ")) {
      Kp = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("ki ")) {
      Ki = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("kd ")) {
      Kd = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("set ")) {
      setpoint = line.substring(4).toFloat();
    } else {
      Serial.println("Valid commands: kp [value], ki [value], kd [value], set [value]");
    }
  }
}
