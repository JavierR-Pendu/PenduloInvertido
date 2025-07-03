#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

// Pines corregidos (sin conflictos)
const int pinPWMA = 16;
const int pinAIN2 = 17;
const int pinAIN1 = 18;
const int pinPWMB = 25;
const int pinBIN1 = 19;
const int pinBIN2 = 26;
const int pinSTBY = 23;

// Variables del sensor MPU6050
int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

// Variables PID
float Kp = 0;
float Ki = 0;
float Kd = 0;
float integral = 0.0;
float derivative = 0.0;
float previous_error = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Pines I2C correctos

  sensor.initialize();

  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinSTBY, OUTPUT);

  // Configuración PWM para ESP32
  ledcSetup(0, 1000, 8);  // canal 0, frecuencia 1kHz, resolución 8 bits
  ledcAttachPin(pinPWMA, 0);

  ledcSetup(1, 1000, 8);  // canal 1
  ledcAttachPin(pinPWMB, 1);
}

void loop() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  float accel_ang_x = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / PI);
  float accel_ang_y = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / PI);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  float desired_angle = 0.0;
  float error = desired_angle - ang_x;

  integral += error * dt;
  derivative = (error - previous_error) / dt;

  float motor_speed = Kp * error + Ki * integral + Kd * derivative;

  enableMotors();
  moveMotor(pinPWMA, pinAIN1, pinAIN2, motor_speed, 0);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, motor_speed, 1);

  Serial.print(" Rotación en X: ");
  Serial.print(ang_x);
  Serial.print(", Error: ");
  Serial.print(error);
  Serial.print(", Velocidad: ");
  Serial.println(motor_speed);

  previous_error = error;

  delay(1);
}

// Función adaptada para PWM en ESP32
void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed, int channel) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    ledcWrite(channel, speed);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    ledcWrite(channel, -speed);
  }
}

void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}

