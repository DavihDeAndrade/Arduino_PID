#include <PID_v1.h>

// Sensor pins
const int TRIG_UPPER = 9;
const int ECHO_UPPER = 10;
const int TRIG_BELOW = 7;  // Trigger for lower tank
const int ECHO_BELOW = 8;  // Echo for lower tank

// H-Bridge
const int PUMP_IN1 = 5;
const int PUMP_IN2 = 6;
const int PUMP_ENA = 3;

// PID variables
double currentLevel = 0;        // Current sensor reading in cm (upper tank)
double setpointPercent = 0.0;   // Desired level as percentage (0-100%)
double setpointDistance = 0;    // Converted setpoint for sensor distance
double controlSignal = 0;

// Tank configuration
const float TANK_HEIGHT = 15.0;      // Total tank height in cm
const float SENSOR_OFFSET = 1.3;     // Sensor mounting depth from top (cm)
const float MIN_WATER_HEIGHT = 2.5;  // Minimum water level in cm
const float MAX_WATER_HEIGHT = 10.0; // Maximum water level in cm

// Calculated sensor ranges
const float SENSOR_TO_BOTTOM = TANK_HEIGHT - SENSOR_OFFSET; // 13.7cm
const float SENSOR_TO_EMPTY = SENSOR_TO_BOTTOM - MIN_WATER_HEIGHT; // 11.2cm
const float SENSOR_TO_FULL = SENSOR_TO_BOTTOM - MAX_WATER_HEIGHT;  // 3.7cm

// PID gains (adjustable)
double Kp = 15.0, Ki = 1.0, Kd = 6.0;
PID pid(&currentLevel, &controlSignal, &setpointDistance, Kp, Ki, Kd, REVERSE);

// Time control
unsigned long lastReading = 0;
const unsigned long interval = 100;

// Interlock system
bool setpointReceived = false;  // Flag for first setpoint

void setup() {
  Serial.begin(9600);
  
  // Clear serial buffer
  while(Serial.available() > 0) Serial.read();
  
  // Ultrasonic sensors
  pinMode(TRIG_UPPER, OUTPUT);
  pinMode(ECHO_UPPER, INPUT);
  pinMode(TRIG_BELOW, OUTPUT);
  pinMode(ECHO_BELOW, INPUT);
  
  // H-Bridge
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(PUMP_ENA, OUTPUT);
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_ENA, 0);  // Pump off initially
  
  // PID configuration - start in MANUAL mode
  pid.SetMode(MANUAL);
  pid.SetOutputLimits(0, 255);  // PID internal works from 0-255
  pid.SetSampleTime(interval);
  
  // Initialize setpoint distance
  updateSetpoint();
}

void loop() {
  
  delay(500);
  
  unsigned long now = millis();
  
  if (now - lastReading >= interval) {
    lastReading = now;
    
    // Read both sensors regardless of interlock
    float upperSensorReading = readLevel(TRIG_UPPER, ECHO_UPPER);
    float lowerSensorReading = readLevel(TRIG_BELOW, ECHO_BELOW);
    
    int pumpPWM = 0;  // Default to 0 (pump off)
    
    // Only run PID if setpoint received
    if (setpointReceived) {
      currentLevel = upperSensorReading;  // For PID calculation
      pid.Compute();
      pumpPWM = map(controlSignal, 0, 255, 16, 50);
      pumpPWM = constrain(pumpPWM, 16, 50);
      applyPWM(pumpPWM);
    } else {
      // Ensure pump is off while waiting
      analogWrite(PUMP_ENA, 0);
    }
    
    // Send RAW data to Python (distances in cm, PWM as integer)
    Serial.print(upperSensorReading, 1);
    Serial.print(",");
    Serial.print(lowerSensorReading, 1);
    Serial.print(",");
    Serial.println(pumpPWM);
  }
  
  readSerialCommands();
}

// Convert percentage (0-100%) to sensor distance
float percentToSensor(float percent) {
  // Constrain percentage to valid range
  percent = constrain(percent, 0, 100);
  
  // Calculate sensor distance for given percentage
  // 0% = EMPTY = 11.2cm (SENSOR_TO_EMPTY)
  // 100% = FULL = 3.7cm (SENSOR_TO_FULL)
  return SENSOR_TO_EMPTY - (percent / 100.0) * (SENSOR_TO_EMPTY - SENSOR_TO_FULL);
}

// Update setpoint distance based on percentage
void updateSetpoint() {
  setpointDistance = percentToSensor(setpointPercent);
}

// Read level with ultrasonic sensor
float readLevel(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout 30ms
  float distance = duration * 0.0343 / 2.0;       // distance in cm
  
  // Handle invalid readings by returning previous value
  static float lastGoodValue = 0;
  if (distance > 0 && distance < 100) {
    lastGoodValue = distance;
  }
  return lastGoodValue;
}

// Apply control to pump - PWM limited between 16 and 50
void applyPWM(int pwm) {
  // Ensure PWM is within limits (16-50)
  pwm = constrain(pwm, 16, 50);
  
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  analogWrite(PUMP_ENA, pwm);
}

// Interpret Serial commands
void readSerialCommands() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("SP:")) {
      // Setpoint command from Python
      setpointPercent = line.substring(3).toFloat();
      setpointPercent = constrain(setpointPercent, 0, 100);
      updateSetpoint();
      
      if (!setpointReceived) {
        // First setpoint received - release interlock
        setpointReceived = true;
        pid.SetMode(AUTOMATIC);
        Serial.println("INTERLOCK RELEASED");
      }
    } else if (line.startsWith("kp ")) {
      Kp = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("ki ")) {
      Ki = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("kd ")) {
      Kd = line.substring(3).toFloat();
      pid.SetTunings(Kp, Ki, Kd);
    } else if (line.startsWith("set ")) {
      setpointPercent = line.substring(4).toFloat();
      setpointPercent = constrain(setpointPercent, 0, 100);
      updateSetpoint();
    }
  }
}
