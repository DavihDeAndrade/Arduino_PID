// PID coefficients
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.1;
// Internal PID variables
float erro_anterior = 0;
float erro_integral = 0;
unsigned long tempo_anterior = 0;
const float dt = 0.1;  // 100 ms
// Setpoint (desired level) and current reading
float setpoint = 10.0;  // for example, 10 cm
float nivel_atual;      // value read from sensor
void setup() {
  Serial.begin(9600);
  // Initialize pump, sensor etc.
}
void loop() {
  unsigned long agora = millis();
  if ((agora - tempo_anterior) >= dt * 1000) {
    tempo_anterior = agora;
    // Level sensor reading
    nivel_atual = lerNivel();
    // Error calculation
    float erro = setpoint - nivel_atual;
    // Integral and derivative
    erro_integral += erro * dt;
    float erro_derivativo = (erro - erro_anterior) / dt;
    // PID
    float controle = Kp * erro + Ki * erro_integral + Kd * erro_derivativo;
    // Saturate output (for example, between 0 and 255 for PWM)
    controle = constrain(controle, 0, 255);
    // Send signal to pump
    analogWrite(PIN_BOMBA, (int)controle);
    // Update previous error
    erro_anterior = erro;
    // (Optional) print for monitoring
    Serial.print("Nivel: ");
    Serial.print(nivel_atual);
    Serial.print(" | Controle: ");
    Serial.println(controle);
  }
}
float lerNivel() {
  // Here adapt for a real sensor
  int leitura = analogRead(A0);
  float nivel_cm = map(leitura, 0, 1023, 0, 20);  // example: 0–20 cm
  return nivel_cm;
}
