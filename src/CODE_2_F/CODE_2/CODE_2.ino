#include <Servo.h>

// ====== TB6612FNG – Canal A ======
const int PIN_STBY = 4;   
const int PIN_PWMA = 5;   // PWM motor
const int PIN_AIN1 = 7;   
const int PIN_AIN2 = 8;   

int velocidad = 160;  // 0–255

// ====== Servo dirección ======
Servo direccion;
const int PIN_SERVO = 3;   // servo conectado a D3
int posicionServo = 90;     // inicial

// ====== Sensores ultrasónicos ======
const int trigIzq = A0, echoIzq = A1;
const int trigFront = A2, echoFront = A3;
const int trigDer = A4, echoDer = A5;

long distanciaIzq, distanciaFront, distanciaDer;
const int UMBRAL = 70; // cm

// ====== Pulsador inicio ======
const int PIN_START = 13;
bool iniciado = false;

// ====== Pines TCS3200 ======
#define S0 11
#define S1 12
#define S2 9
#define S3 10
#define sensorOut 6

// Valores RGB de referencia
int rojoRef[3]  = {238, 39, 55};
int verdeRef[3] = {68, 214, 44};
int tolerancia = 30;

#define N_CAL 10
#define N_HIST 5
int histR[N_HIST], histG[N_HIST], histB[N_HIST];
int idxHist = 0;

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
  long duracion = pulseIn(echo, HIGH, 20000); 
  long distancia = duracion * 0.034 / 2; 
  return distancia == 0 ? 999 : distancia;
}

// ====== Funciones sensor de color ======
long readColorRaw(int s2, int s3){
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  long total = 0;
  for(int i=0;i<5;i++){
    long lectura = pulseIn(sensorOut, LOW, 100000);
    if(lectura == 0) lectura = 10000;
    total += lectura;
  }
  return total/5;
}

int normalizarAuto(long lectura, long minVal, long maxVal){
  if(lectura < minVal) lectura = minVal;
  if(lectura > maxVal) lectura = maxVal;
  return map(lectura, maxVal, minVal, 0, 255);
}

bool dentroTolerancia(int valor, int referencia){
  return (valor >= referencia - tolerancia && valor <= referencia + tolerancia);
}

String detectarColor(){
  // Calibración de luz
  long rawR[N_CAL], rawG[N_CAL], rawB[N_CAL];
  long minR=99999, maxR=0, minG=99999, maxG=0, minB=99999, maxB=0;

  for(int i=0;i<N_CAL;i++){
    rawR[i]=readColorRaw(LOW, LOW);
    rawG[i]=readColorRaw(HIGH, HIGH);
    rawB[i]=readColorRaw(LOW, HIGH);

    if(rawR[i]<minR) minR=rawR[i]; if(rawR[i]>maxR) maxR=rawR[i];
    if(rawG[i]<minG) minG=rawG[i]; if(rawG[i]>maxG) maxG=rawG[i];
    if(rawB[i]<minB) minB=rawB[i]; if(rawB[i]>maxB) maxB=rawB[i];
  }

  long sumR=0, sumG=0, sumB=0;
  for(int i=0;i<N_CAL;i++){
    sumR+=rawR[i]; sumG+=rawG[i]; sumB+=rawB[i];
  }

  int R = normalizarAuto(sumR/N_CAL, minR, maxR);
  int G = normalizarAuto(sumG/N_CAL, minG, maxG);
  int B = normalizarAuto(sumB/N_CAL, minB, maxB);

  // Suavizado
  histR[idxHist] = R;
  histG[idxHist] = G;
  histB[idxHist] = B;
  idxHist = (idxHist + 1) % N_HIST;

  long avgR=0, avgG=0, avgB=0;
  for(int i=0;i<N_HIST;i++){
    avgR += histR[i];
    avgG += histG[i];
    avgB += histB[i];
  }
  avgR /= N_HIST; avgG /= N_HIST; avgB /= N_HIST;

  if(dentroTolerancia(avgR, rojoRef[0]) &&
     dentroTolerancia(avgG, rojoRef[1]) &&
     dentroTolerancia(avgB, rojoRef[2])){
    return "ROJO";
  }
  else if(dentroTolerancia(avgR, verdeRef[0]) &&
          dentroTolerancia(avgG, verdeRef[1]) &&
          dentroTolerancia(avgB, verdeRef[2])){
    return "VERDE";
  }
  else return "DESCONOCIDO";
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

  pinMode(PIN_START, INPUT);

  direccion.attach(PIN_SERVO);
  direccion.write(posicionServo);

  // Pines TCS3200
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  motorStandby(true);
  motorDetener();

  Serial.begin(9600);
}

// ====== Loop ======
void loop() {
  if (!iniciado) {
    if (digitalRead(PIN_START) == HIGH) {
      iniciado = true;
      Serial.println("Robot iniciado!");
    } else {
      return;
    }
  }

  // Leer distancias
  distanciaFront = medirDistancia(trigFront, echoFront);
  distanciaIzq   = medirDistancia(trigIzq, echoIzq);
  distanciaDer   = medirDistancia(trigDer, echoDer);

  Serial.print("Front: "); Serial.print(distanciaFront);
  Serial.print(" | Izq: "); Serial.print(distanciaIzq);
  Serial.print(" | Der: "); Serial.print(distanciaDer);

  if (distanciaFront > UMBRAL) {
    // Avanzar recto
    posicionServo = 90;
    direccion.write(posicionServo);
    motorAvanzar(velocidad);
  } 
  else {
    // === Antes de mirar izquierda/derecha, ver el color ===
    String color = detectarColor();
    Serial.print(" | Color: "); Serial.println(color);

    if (color == "ROJO") {
      posicionServo = 155;
      direccion.write(posicionServo);
      motorAvanzar(velocidad);
      delay(500);
      posicionServo = 90;
      direccion.write(posicionServo);
    } 
    else if (color == "VERDE") {
      posicionServo = 30;
      direccion.write(posicionServo);
      motorAvanzar(velocidad);
      delay(500);
      posicionServo = 90;
      direccion.write(posicionServo);
    } 
    else {
      // Si no detecta color válido, usar lógica normal
      if (distanciaIzq > 25) {
        posicionServo = 155;
        direccion.write(posicionServo);
        motorAvanzar(velocidad);
        delay(2500);
        posicionServo = 90;
        direccion.write(posicionServo);
      } 
      else if (distanciaDer > 25) {
        posicionServo = 30;
        direccion.write(posicionServo);
        motorAvanzar(velocidad);
        delay(2500);
        posicionServo = 90;
        direccion.write(posicionServo);
      } 
      else {
        motorDetener();
      }
    }
  }

  delay(100);
}
