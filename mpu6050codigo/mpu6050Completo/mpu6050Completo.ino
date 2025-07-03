/*
 * MPU‑6050 con ESP32 | Cálculo de ángulos con filtro complementario
 * SDA → GPIO 21
 * SCL → GPIO 22
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 sensor;  // Dirección por defecto 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long tiempo_prev = 0;
float dt = 0.0f;

float ang_x = 0.0f, ang_y = 0.0f;
float ang_x_prev = 0.0f, ang_y_prev = 0.0f;

void setup() {
  Serial.begin(115200);      // Velocidad más alta recomendada para ESP32
  Wire.begin(21, 22);        // SDA = 21, SCL = 22

  sensor.initialize();
  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
    while (true);  // detiene ejecución si falla
  }

  tiempo_prev = millis();
}

void loop() {
  // 1. Leer acelerómetro y giroscopio
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // 2. Calcular Δt
  unsigned long tiempo_act = millis();
  dt = (tiempo_act - tiempo_prev) / 1000.0f;
  tiempo_prev = tiempo_act;

  // 3. Ángulos desde acelerómetro
  float accel_ang_x = atan2((float)ay, sqrt((float)ax * ax + (float)az * az)) * RAD_TO_DEG;
  float accel_ang_y = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * RAD_TO_DEG;

  // 4. Ángulos desde giroscopio + filtro complementario
  float gyro_ang_x = ang_x_prev + ((float)gx / 131.0f) * dt;
  float gyro_ang_y = ang_y_prev + ((float)gy / 131.0f) * dt;

  ang_x = 0.98f * gyro_ang_x + 0.02f * accel_ang_x;
  ang_y = 0.98f * gyro_ang_y + 0.02f * accel_ang_y;

  ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  // 5. Mostrar ángulos
  Serial.print("Rotación X: ");
  Serial.print(ang_x);
  Serial.print("\tRotación Y: ");
  Serial.println(ang_y);

  delay(10);
}

