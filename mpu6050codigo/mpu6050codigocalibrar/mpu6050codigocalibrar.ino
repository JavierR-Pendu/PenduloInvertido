/*
 * Lectura y autocorrección de offsets del MPU‑6050 con ESP32
 * SDA → GPIO 21
 * SCL → GPIO 22
 */

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 sensor;  // Dirección por defecto 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;

long fAx, fAy, fAz;
long fGx, fGy, fGz;
int16_t pAx, pAy, pAz;
int16_t pGx, pGy, pGz;

int counter = 0;

int16_t axOff, ayOff, azOff;
int16_t gxOff, gyOff, gzOff;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA = 21, SCL = 22 en ESP32

  sensor.initialize();
  if (sensor.testConnection()) {
    Serial.println("Sensor iniciado correctamente");
  } else {
    Serial.println("Error al iniciar el sensor");
  }

  // Leer offsets actuales
  axOff = sensor.getXAccelOffset();
  ayOff = sensor.getYAccelOffset();
  azOff = sensor.getZAccelOffset();
  gxOff = sensor.getXGyroOffset();
  gyOff = sensor.getYGyroOffset();
  gzOff = sensor.getZGyroOffset();

  Serial.println("Offsets actuales:");
  Serial.print(axOff); Serial.print('\t');
  Serial.print(ayOff); Serial.print('\t');
  Serial.print(azOff); Serial.print('\t');
  Serial.print(gxOff); Serial.print('\t');
  Serial.print(gyOff); Serial.print('\t');
  Serial.println(gzOff);

  Serial.println("\nEnvíe cualquier caracter para empezar la calibración\n");
  while (!Serial.available());
  Serial.read();  // descartar carácter
  Serial.println("Calibrando, no mover la IMU");
}

void loop() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Filtro IIR simple
  fAx = fAx - (fAx >> 5) + ax;   pAx = fAx >> 5;
  fAy = fAy - (fAy >> 5) + ay;   pAy = fAy >> 5;
  fAz = fAz - (fAz >> 5) + az;   pAz = fAz >> 5;

  fGx = fGx - (fGx >> 3) + gx;   pGx = fGx >> 3;
  fGy = fGy - (fGy >> 3) + gy;   pGy = fGy >> 3;
  fGz = fGz - (fGz >> 3) + gz;   pGz = fGz >> 3;

  // Recalibrar cada 100 lecturas
  if (counter == 100) {
    Serial.print("Promedio:\t");
    Serial.print(pAx); Serial.print('\t');
    Serial.print(pAy); Serial.print('\t');
    Serial.print(pAz); Serial.print('\t');
    Serial.print(pGx); Serial.print('\t');
    Serial.print(pGy); Serial.print('\t');
    Serial.println(pGz);

    // Ajuste fino de offsets
    axOff += (pAx > 0) ? -1 : 1;
    ayOff += (pAy > 0) ? -1 : 1;
    azOff += ((pAz - 16384) > 0) ? -1 : 1;

    gxOff += (pGx > 0) ? -1 : 1;
    gyOff += (pGy > 0) ? -1 : 1;
    gzOff += (pGz > 0) ? -1 : 1;

    sensor.setXAccelOffset(axOff);
    sensor.setYAccelOffset(ayOff);
    sensor.setZAccelOffset(azOff);
    sensor.setXGyroOffset(gxOff);
    sensor.setYGyroOffset(gyOff);
    sensor.setZGyroOffset(gzOff);

    counter = 0;
  }

  counter++;
}

