/*
 * Lectura RAW de acelerómetro y giroscopio del MPU6050 con ESP32
 * SDA → GPIO 21
 * SCL → GPIO 22
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// Crear objeto sensor (dirección por defecto 0x68)
MPU6050 sensor;

int ax, ay, az;
int gx, gy, gz;

void setup() {
  Serial.begin(115200);        // Velocidad serie para ESP32
  Wire.begin(21, 22);          // SDA = 21, SCL = 22 en ESP32

  sensor.initialize();         // Inicializar MPU6050

  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
    while (1);  // detener ejecución si no hay conexión
  }
}

void loop() {
  // Leer acelerómetro y giroscopio
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Imprimir valores RAW separados por tabuladores
  Serial.print("a[x y z]\t g[x y z]:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100);
}
