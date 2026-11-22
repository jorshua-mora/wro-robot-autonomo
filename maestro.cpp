#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>

#define DIRECCION_ESCLAVO 0x08
#define MPU6050_ADDR 0x68 // Dirección I2C del giroscopio

// --- PINES DEL BOTÓN DE INICIO ---
#define PIN_BTN_OUT 3
#define PIN_BTN_IN  2

// --- PINES DE SENSORES ULTRASÓNICOS ---
#define MAX_DISTANCE 200

#define TRIG_FRONT 10
#define ECHO_FRONT 9
#define TRIG_DER 8
#define ECHO_DER 7 
#define TRIG_IZQ 6
#define ECHO_IZQ 5

NewPing sensorIzq(TRIG_IZQ, ECHO_IZQ, MAX_DISTANCE);
NewPing sensorDer(TRIG_DER, ECHO_DER, MAX_DISTANCE);
NewPing sensorFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);

// --- MÁQUINA DE ESTADOS ---
enum EstadoCarro { RECTA, CURVA, COOLDOWN, APROXIMACION, FIN };
EstadoCarro estadoActual = RECTA;

// --- CONTADOR Y MEMORIA DEL CUADRANTE INICIAL ---
int contadorCurvas = 0;
const int TOTAL_CURVAS_RETO = 12; // 3 vueltas completas

float distFrontInicial = 0.0;
float distDerInicial = 0.0;
float distIzqInicial = 0.0;
float targetOffsetLateral = 0.0; 

// --- AJUSTES DE PRECISIÓN PARA APROXIMACIÓN ---
const float MARGEN_TOLERANCIA_FRONT = 150.0; // Margen en cm para la ventana de frenado
const int LECTURAS_CONFIRMACION = 3;       // Lecturas consecutivas requeridas para validar posición
int contadorConfirmaciones = 0;            // Contador de validaciones en tiempo real

// --- PARÁMETROS DE PISTA ---
const float DIST_DETECCION_FRONT = 85.0; 
const float ANGULO_OBJETIVO = 76; 
float anguloObjetivoAjustado = 69;

const unsigned long TIEMPO_APROXIMACION = 500; 
const unsigned long TIEMPO_COOLDOWN = 100; 
const unsigned long FRONT_COOLDOWN = 600; 

unsigned long tiempoInicioAproximacion = 0;
unsigned long tiempoInicioCooldown = 0;
int direccionGiroActual = 90;

// --- VARIABLES DEL GIROSCOPIO ---
float anguloZ_acumulado = 0.0;
float offsetZ = 0.0; 

// --- CONSTANTES DEL PD ---
float Kp = 10.33;  
float Kd = 14.0;  
float error_anterior = 0;   
unsigned long tiempo_anterior = 0; 

// --- MEMORIA DE SENSORES PARA FILTRADO ---
float distIzqFiltrada = 30.0;
float distDerFiltrada = 30.0;
float distFrontFiltrada = 120.0;

// --- VELOCIDADES ---
const int velCrucero = 255; 
const int velCurva = 255; 
const int velAproximacion = 255; 

int sentidoPista = 0;

float leerGiroscopioZ();
float filtrarLectura(float lecturaCruda, float lecturaAnterior);
void enviarComandosEsclavo(int angulo, int velocidad); 

void setup() {
  // Configuración del Pulsador de Inicio
  pinMode(PIN_BTN_OUT, OUTPUT);
  digitalWrite(PIN_BTN_OUT, HIGH); 
  pinMode(PIN_BTN_IN, INPUT);

  pinMode(TRIG_IZQ, OUTPUT); pinMode(ECHO_IZQ, INPUT);   
  pinMode(TRIG_DER, OUTPUT); pinMode(ECHO_DER, INPUT);   
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);

  Wire.begin(); 
  
  // ESPERA A QUE SE PRESIONE EL BOTÓN PARA CONTINUAR
  while (digitalRead(PIN_BTN_IN) == LOW) {
    delay(20);
  }

  // Despertar MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  // Calibración del Giroscopio
  long sumaZ = 0;
  for(int i = 0; i < 50; i++) {
    sumaZ += leerGiroscopioZ();
    delay(3);
  }
  offsetZ = sumaZ / 50.0;

  // CAPTURA LIMPIA DE POSICIÓN INICIAL (Muestra filtrada de 15 lecturas)
  delay(100);
  float sumF = 0, sumI = 0, sumD = 0;
  int lecturasValidasF = 0, lecturasValidasI = 0, lecturasValidasD = 0;

  for(int i = 0; i < 15; i++) {
    float f = sensorFront.ping_cm();
    float iz = sensorIzq.ping_cm();
    float de = sensorDer.ping_cm();
    
    if (f > 5.0 && f < 180.0) { sumF += f; lecturasValidasF++; }
    if (iz > 2.0 && iz < 180.0) { sumI += iz; lecturasValidasI++; }
    if (de > 2.0 && de < 180.0) { sumD += de; lecturasValidasD++; }
    delay(20);
  }
  
  distFrontInicial = (lecturasValidasF > 0) ? (sumF / lecturasValidasF) : 80.0;
  distIzqInicial = (lecturasValidasI > 0) ? (sumI / lecturasValidasI) : 30.0;
  distDerInicial = (lecturasValidasD > 0) ? (sumD / lecturasValidasD) : 30.0;
  
  // Desviación lateral respecto al centro del carril
  targetOffsetLateral = (distDerInicial - distIzqInicial) / 2.0;

  tiempo_anterior = micros(); 
}

void loop() {
  unsigned long tiempoActualMicros = micros(); 
  float dt = (float)(tiempoActualMicros - tiempo_anterior) / 1000000.0; 
  if (dt <= 0) dt = 0.001; 

  // 1. LECTURA Y FILTRADO DE SENSORES
  if (estadoActual == RECTA || estadoActual == APROXIMACION) {
    float rawFront = sensorFront.ping_cm();
    distFrontFiltrada = filtrarLectura(rawFront, distFrontFiltrada);
    delay(20);
  }

  if (estadoActual == RECTA || estadoActual == COOLDOWN || estadoActual == APROXIMACION) {
    float rawIzq = sensorIzq.ping_cm();
    distIzqFiltrada = filtrarLectura(rawIzq, distIzqFiltrada);
    delay(25); 
    
    float rawDer = sensorDer.ping_cm();
    distDerFiltrada = filtrarLectura(rawDer, distDerFiltrada);
    delay(25);
  }

  // 2. MÁQUINA DE ESTADOS
  if (estadoActual == RECTA) {
    if ((millis() - tiempoInicioCooldown) > FRONT_COOLDOWN) {
      // Durante el cooldown, no permite detectar la pared frontal
    
      if ((distFrontFiltrada <= DIST_DETECCION_FRONT) && (distFrontFiltrada > 60.0)) {
        estadoActual = CURVA;
        anguloZ_acumulado = 0.0; 

        if (sentidoPista == 0) {
          delay(20);
          float izqDirecto = sensorIzq.ping_cm();
          if (izqDirecto <= 0.0) izqDirecto = 180.0; 

          delay(25);
          float derDirecto = sensorDer.ping_cm();
          if (derDirecto <= 0.0) derDirecto = 180.0; 

          if (izqDirecto > derDirecto) {
            sentidoPista = 72;  // Antihorario (Pared interna = Izquierda)
          } else {
            sentidoPista = 112; // Horario (Pared interna = Derecha)
          }
        }
        direccionGiroActual = sentidoPista;
      }
    }
  }
  else if (estadoActual == CURVA) {
    float giroZ = (leerGiroscopioZ() - offsetZ) / 131.0; 
    anguloZ_acumulado += (giroZ * dt);

    if (abs(anguloZ_acumulado) >= ANGULO_OBJETIVO) {
      contadorCurvas++; 

      if (contadorCurvas >= TOTAL_CURVAS_RETO) {
        estadoActual = APROXIMACION;
        tiempoInicioAproximacion = millis();
        contadorConfirmaciones = 0;
        error_anterior = 0;
      } else {
        estadoActual = COOLDOWN;
        tiempoInicioCooldown = millis();
        error_anterior = 0; 
        
        float rawIzq = sensorIzq.ping_cm();
        distIzqFiltrada = (rawIzq <= 0.0) ? 30.0 : rawIzq;
        delay(20);
        float rawDer = sensorDer.ping_cm();
        distDerFiltrada = (rawDer <= 0.0) ? 30.0 : rawDer;
      }
    }
  }
  else if (estadoActual == COOLDOWN) {
    if (millis() - tiempoInicioCooldown > TIEMPO_COOLDOWN) {
      estadoActual = RECTA;
    }
  }
  else if (estadoActual == APROXIMACION) {
    unsigned long tiempoEnAproximacion = millis() - tiempoInicioAproximacion;
    
    if (tiempoEnAproximacion > TIEMPO_APROXIMACION) {
      
      // 1. Condición original: Ventana ideal de frenado
      if (distFrontFiltrada <= 165.0 && distFrontFiltrada > 100.0) {
        contadorConfirmaciones++;
        if (contadorConfirmaciones >= LECTURAS_CONFIRMACION) {
          estadoActual = FIN; 
        }
      } else {
        contadorConfirmaciones = 0; 
      }

      
      // 2. FRENO DE EMERGENCIA POR TIEMPO: Si han pasado 2 segundos buscando la pared, detente
      if (tiempoEnAproximacion > 550) {
        estadoActual = FIN;
      }
    }
  }

  // 3. CÁLCULO DE MOVIMIENTO
  int anguloDestino = 92;
  int velocidadActual = velCrucero;

  // --- REDUCCIÓN DE VELOCIDAD ---
  // Si ya completó 10 curvas, significa que está enfrentando la curva 11 o 12.
  // Forzamos al auto a ir lento desde antes para estabilizar la entrada a la meta.
  bool enFaseFinal = (contadorCurvas >= 11); 

  if (estadoActual == FIN) {
    anguloDestino = 92;   
    velocidadActual = 0;   
  }
  else if (estadoActual == CURVA) {
    anguloDestino = direccionGiroActual;
    // Si estamos en las últimas curvas, gira lento; si no, gira a velocidad normal
    velocidadActual = enFaseFinal ? velAproximacion : velCurva; 
  } 
   
  else if (estadoActual == COOLDOWN) {
    anguloDestino = 92; // Ruedas totalmente rectas
    velocidadActual = enFaseFinal ? velAproximacion : velCrucero;
  }
  else {
    // CONTROL PID
    float distancia_al_centro = ((distDerFiltrada - distIzqFiltrada)) / 2.0; 
    float error = 0; 
    float limitePID = 10.0; // Límite de giro normal para alta velocidad

    if (estadoActual == APROXIMACION) {
      error = distancia_al_centro - 0.0; 
      velocidadActual = velAproximacion;
      
      limitePID = 12.0; 
    }
    else {
      // Zona muerta para las rectas normales
      const float ZONA_AZUL = 2.5; 
      if (abs(distancia_al_centro) <= ZONA_AZUL) {
        error = 0; 
      } else {
        if (distancia_al_centro > 0) error = distancia_al_centro - ZONA_AZUL; 
        else error = distancia_al_centro + ZONA_AZUL; 
      }
      
      // Si estamos en la recta 11 o 12, va lento. Si no, va a velocidad de crucero.
      velocidadActual = enFaseFinal ? velAproximacion : velCrucero; 
    }

    float P = Kp * error; 
    float tasa_cambio = (error - error_anterior) / dt; 
    
    tasa_cambio = constrain(tasa_cambio, -150.0, 150.0);
    float D = Kd * tasa_cambio; 
    
    float salidaPID = P + D; 
    
    // APLICAMOS EL NUEVO LÍMITE DINÁMICO
    salidaPID = constrain(salidaPID, -limitePID, limitePID); 
    
    anguloDestino = 92 + salidaPID; 
    anguloDestino = constrain(anguloDestino, 57, 127); // Tope físico del servo

    error_anterior = error; 
  }

  enviarComandosEsclavo(anguloDestino, velocidadActual);
  tiempo_anterior = tiempoActualMicros; 
}

float filtrarLectura(float lecturaCruda, float lecturaAnterior) {
  // Si NewPing pierde el eco (da 0) o da un valor fuera de rango, ignoramos el error
  if (lecturaCruda <= 2.0 || lecturaCruda > 180.0) {
    return lecturaAnterior; // Retorna el último dato real sin contaminar la mezcla
  }
  
  float alpha = 0.4;
  return (alpha * lecturaCruda) + ((1.0 - alpha) * lecturaAnterior);
}

void enviarComandosEsclavo(int angulo, int velocidad) {
  Wire.beginTransmission(DIRECCION_ESCLAVO); 
  Wire.write((byte)angulo);                   
  Wire.write((byte)velocidad);                
  Wire.endTransmission();                     
}

float leerGiroscopioZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x47); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  
  if (Wire.available() >= 2) {
    int16_t Z = Wire.read() << 8 | Wire.read();
    return (float)Z;
  }
  return 0.0;
}


