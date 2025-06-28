void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(3, OUTPUT);

  digitalWrite(5, HIGH);  // IN1
  digitalWrite(6, LOW);   // IN2

  analogWrite(3, 0);    // ENA com 100/255 (~39% PWM)
}

void loop() {
  // nada
}
