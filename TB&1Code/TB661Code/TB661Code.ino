const int pinPWMA = 16;
const int pinAIN2 = 17;
const int pinAIN1 = 18;
const int pinBIN1 = 19;
const int pinBIN2 = 26;  // Cambio por conflicto I2C
const int pinPWMB = 25;  // Cambio por conflicto I2C
const int pinSTBY = 23;

const int waitTime = 2000;
const int speed = 200;  // Velocidad de 0 a 255

const int pinMotorA[3] = { pinPWMA, pinAIN2, pinAIN1 };
const int pinMotorB[3] = { pinPWMB, pinBIN1, pinBIN2 };

enum moveDirection { forward, backward };
enum turnDirection { clockwise, counterClockwise };

void setup() {
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);

  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  pinMode(pinSTBY, OUTPUT);

  // Configuración PWM para ESP32 (8-bit, 1 kHz)
  ledcSetup(0, 1000, 8);  // Canal 0 para PWMA
  ledcAttachPin(pinPWMA, 0);

  ledcSetup(1, 1000, 8);  // Canal 1 para PWMB
  ledcAttachPin(pinPWMB, 1);
}

void loop() {
  enableMotors();

  move(forward, speed);
  delay(waitTime);

  move(backward, speed);
  delay(waitTime);

  turn(clockwise, speed);
  delay(waitTime);

  turn(counterClockwise, speed);
  delay(waitTime);

  fullStop();
  delay(waitTime);
}

// Control de dirección
void move(int direction, int spd) {
  if (direction == forward) {
    moveMotorForward(pinMotorA, spd, 0);
    moveMotorForward(pinMotorB, spd, 1);
  } else {
    moveMotorBackward(pinMotorA, spd, 0);
    moveMotorBackward(pinMotorB, spd, 1);
  }
}

void turn(int direction, int spd) {
  if (direction == clockwise) {
    moveMotorForward(pinMotorA, spd, 0);
    moveMotorBackward(pinMotorB, spd, 1);
  } else {
    moveMotorBackward(pinMotorA, spd, 0);
    moveMotorForward(pinMotorB, spd, 1);
  }
}

void fullStop() {
  disableMotors();
  stopMotor(pinMotorA, 0);
  stopMotor(pinMotorB, 1);
}

// Control de motores con PWM (ESP32)
void moveMotorForward(const int pinMotor[3], int spd, int pwmChannel) {
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);
  ledcWrite(pwmChannel, spd);
}

void moveMotorBackward(const int pinMotor[3], int spd, int pwmChannel) {
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], HIGH);
  ledcWrite(pwmChannel, spd);
}

void stopMotor(const int pinMotor[3], int pwmChannel) {
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], LOW);
  ledcWrite(pwmChannel, 0);
}

void enableMotors() {
  digitalWrite(pinSTBY, HIGH);
}

void disableMotors() {
  digitalWrite(pinSTBY, LOW);
}