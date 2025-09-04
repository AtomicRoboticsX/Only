// Pines del motor paso a paso
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

int demora = 5; // milisegundos entre pasos (controla velocidad)

// Secuencia de pasos (modo wave drive: 4 pasos)
int secuencia[4][4] = {
  {1, 0, 0, 0}, // Paso 1
  {0, 1, 0, 0}, // Paso 2
  {0, 0, 1, 0}, // Paso 3
  {0, 0, 0, 1}  // Paso 4
};

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Girar en un sentido
  for (int i = 0; i < 512; i++) { // 512 pasos = 1 vuelta aprox (28BYJ-48)
    pasoAdelante();
  }

  delay(1000);

  // Girar en sentido contrario
  for (int i = 0; i < 512; i++) {
    pasoAtras();
  }

  delay(1000);
}

// Avanza un paso adelante
void pasoAdelante() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(IN1, secuencia[i][0]);
    digitalWrite(IN2, secuencia[i][1]);
    digitalWrite(IN3, secuencia[i][2]);
    digitalWrite(IN4, secuencia[i][3]);
    delay(demora);
  }
}

// Avanza un paso hacia atrás
void pasoAtras() {
  for (int i = 3; i >= 0; i--) {
    digitalWrite(IN1, secuencia[i][0]);
    digitalWrite(IN2, secuencia[i][1]);
    digitalWrite(IN3, secuencia[i][2]);
    digitalWrite(IN4, secuencia[i][3]);
    delay(demora);
  }
}
