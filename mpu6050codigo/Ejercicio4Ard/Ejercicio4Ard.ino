const uint8_t  ALGO = 1;
const float    dt   = 0.01f;
const uint32_t dt_ms = uint32_t(dt * 1000.0f);

float Kp = 1.0f;
float Ki = 0.5f;
float Kd = 0.1f;

float setpoint = 0.0f;
float measured_value = 0.0f;
float output = 0.0f;

// ALGO 1
float previous_error = 0.0f;
float integral = 0.0f;

inline void pid1() {
  float error = setpoint - measured_value;
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
}

// ALGO 2
float A0, A1, A2;
float errorA[3] = {0.0f, 0.0f, 0.0f};

inline void pid2() {
  errorA[2] = errorA[1];
  errorA[1] = errorA[0];
  errorA[0] = setpoint - measured_value;
  output = A0 * errorA[0] + A1 * errorA[1] + A2 * errorA[2];
}

// ALGO 3
float A0_PI, A1_PI;
float A0d, A1d, A2d;
float alpha_1, alpha_2;
float errorB[3] = {0.0f, 0.0f, 0.0f};
float d0 = 0.0f, d1 = 0.0f, fd0 = 0.0f, fd1 = 0.0f;

inline void pid3() {
  errorB[2] = errorB[1];
  errorB[1] = errorB[0];
  errorB[0] = setpoint - measured_value;
  output += A0_PI * errorB[0] + A1_PI * errorB[1];
  d1 = d0;
  d0 = A0d * errorB[0] + A1d * errorB[1] + A2d * errorB[2];
  fd1 = fd0;
  fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;
  output += fd0;
}

void setup() {
  A0 = Kp + Ki * dt + Kd / dt;
  A1 = -Kp - 2.0f * Kd / dt;
  A2 = Kd / dt;

  A0_PI = Kp + Ki * dt;
  A1_PI = -Kp;

  A0d = Kd / dt;
  A1d = -2.0f * Kd / dt;
  A2d = Kd / dt;

  float N = 5.0f;
  float tau = Kd / (Kp * N);
  float alpha = dt / (2.0f * tau);

  alpha_1 = alpha / (alpha + 1.0f);
  alpha_2 = (alpha - 1.0f) / (alpha + 1.0f);
}

void loop() {
  // measured_value = ...;
  
  switch (ALGO) {
    case 1: pid1(); break;
    case 2: pid2(); break;
    case 3: pid3(); break;
  }

  // aplicar output al actuador

  delay(dt_ms);
}
