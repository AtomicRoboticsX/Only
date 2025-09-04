#include <Servo.h>

// ====== TB6612FNG – Canal A ======
const int PIN_STBY = 4;   
const int PIN_PWMA = 5;   // PWM motor
const int PIN_AIN1 = 7;   
const int PIN_AIN2 = 8;   

int velocidad = 135;  // 0–255

// ====== Servo dirección ======
Servo direccion;
const int PIN_SERVO = 3;   // servo conectado a D3
int posicionServo = 90;     // inicial

// ====== Sensores ultrasónicos ======
const int trigIzq = A0, echoIzq = A1;
const int trigFront = A2, echoFront = A3;
const int trigDer = A4, echoDer = A5;

long distanciaIzq, distanciaFront, distanciaDer;
const int UMBRAL = 20; // cm

// ====== Botón en pin 13 ======
const int PIN_BOTON = 13;
bool robotActivo = false;   // bandera: inicia apagado

// ====== Funciones Motor ======
void motorAvanzar(int vel) {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, constrain(vel, 0, 135));
}

void motorDetener() {
  analogWrite(PIN_PWMA, 0);
}

void motorStandby(bool activo) {
  digitalWrite(PIN_STBY, activo ? HIGH : LOW);
}

// ====== Medir distancia ======
long medirDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duracion = pulseIn(echo, HIGH, 20000); // timeout 20ms
  long distancia = duracion * 0.034 / 2; // en cm
  return distancia == 0 ? 999 : distancia; // si no lee, devuelve "muy lejos"
}

// ====== Setup ======
void setup() {
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);

  pinMode(trigIzq, OUTPUT);
  pinMode(echoIzq, INPUT);
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigDer, OUTPUT);
  pinMode(echoDer, INPUT);

  pinMode(PIN_BOTON, INPUT); // Pulsador a 5V con resistencia pull-down

  direccion.attach(PIN_SERVO);
  direccion.write(posicionServo); // posición inicial recto

  motorStandby(true);
  motorDetener();

  Serial.begin(9600);
}

// ====== Función principal del robot ======
void ejecutarRobot() {
  // Leer distancias
  distanciaFront = medirDistancia(trigFront, echoFront);
  distanciaIzq   = medirDistancia(trigIzq, echoIzq);
  distanciaDer   = medirDistancia(trigDer, echoDer);

  // Mostrar en monitor serial
  Serial.print("Izq: "); Serial.print(distanciaIzq);
  Serial.print(" cm | Front: "); Serial.print(distanciaFront);
  Serial.print(" cm | Der: "); Serial.print(distanciaDer);
  Serial.print(" cm | Servo: "); Serial.println(posicionServo);

  // Lógica de decisión
  if (distanciaFront > UMBRAL) {
    // Avanzar recto
    posicionServo = 90;
    direccion.write(posicionServo);
    motorAvanzar(velocidad);
  }
  else if (distanciaIzq > UMBRAL) {
    // Girar izquierda
    posicionServo = 155;
    direccion.write(posicionServo);
    motorAvanzar(velocidad);
    delay(500); 
    distanciaFront = medirDistancia(trigFront, echoFront);
    if (distanciaFront > UMBRAL) {
      posicionServo = 90;
      direccion.write(posicionServo);
      motorAvanzar(velocidad);
    }
  }
  else if (distanciaDer > UMBRAL) {
    // Girar derecha
    posicionServo = 30;
    direccion.write(posicionServo);
    motorAvanzar(velocidad);
    delay(500); 
    distanciaFront = medirDistancia(trigFront, echoFront);
    if (distanciaFront > UMBRAL) {
      posicionServo = 90;
      direccion.write(posicionServo);
      motorAvanzar(velocidad);
    }
  }
  else {
    // Todo bloqueado → detenerse
    motorDetener();
  }

  delay(100); // pequeña pausa para estabilidad
}

// ====== Loop ======
void loop() {
  if (!robotActivo && digitalRead(PIN_BOTON) == HIGH) {
    robotActivo = true; // primera vez que se aprieta, se activa
    Serial.println("Robot iniciado!");
    delay(300); // pequeña pausa para evitar rebote del botón
  }

  if (robotActivo) {
    ejecutarRobot();
  } else {
    motorDetener();
  }
}
