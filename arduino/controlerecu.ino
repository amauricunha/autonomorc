#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <mcp_can.h>

// --- Configuração dos sensores e atuadores ---
const int pinTCRT5000 = 3; // Sensor óptico na roda (interrupção)
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastPulseInterval = 0; // NOVO: tempo entre pulsos em ms

const int pinESC = 9; // PWM para ESC do motor
int pwmValue = 120;   // Valor inicial de PWM (ajuste conforme necessário)

const int trigPin = 6; // HC-SR04 trigger
const int echoPin = 7; // HC-SR04 echo

int pinLed = 12; // led conectado no pino digital 12
int pinSensor = 2; // fototransistor conectado no pino digial 2
   
// --- Mock para MPU9250 ---
float getVehicleSpeedFromMPU() {
  // TODO: Implementar leitura real do MPU9250
  // Mock: retorna valor fixo para teste
  return 1.0; // m/s
}

// --- Interrupção para contar pulsos do TCRT5000 ---
void onPulse() {
  unsigned long now = millis();
  lastPulseInterval = now - lastPulseTime; // NOVO: calcula intervalo desde o último pulso
  lastPulseTime = now;
  pulseCount++;
  Serial.println("Pulso detectado!"); // DEBUG: Mostra quando um pulso é detectado
}

// --- Pinos RC Receiver ---
const int pinRC_CH1 = 3; // direção (entrada PWM do RC)
const int pinRC_CH2 = 2; // aceleração (entrada PWM do RC)
const int pinRC_CH3 = 4; // controle de tração (entrada PWM do RC)
const int pinRC_CH4 = 5; // modo manual/autônomo (entrada PWM do RC)
const int pinServo = 10; // saída PWM para servo de direção

Servo servoDirecao;

// --- Variáveis de modo ---
bool tractionControlEnabled = false;
bool autonomousMode = false;

// --- MCP2515 CAN ---
const int CAN_CS_PIN = 8;
MCP_CAN CAN(CAN_CS_PIN);
bool canAvailable = false; // NOVO: flag para indicar se CAN está disponível

// --- Variáveis para comandos CAN ---
int can_esc_pwm = 120;
int can_servo_angle = 90;

// --- Variáveis globais para obstáculos e CAN ---
bool obstacleDetected = false;
bool lastObstacleState = false;
#define CAN_OBS_ID 0x300 // Defina um ID CAN para mensagem de obstáculo

// --- Inicialização ---
void setup(){ 
  pinMode(pinSensor, INPUT); // define como entrada 
  pinMode(pinLed, OUTPUT); // definoe como saída   
  digitalWrite(pinLed, LOW); // led incia apagado
  
  pinMode(pinTCRT5000, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinTCRT5000), onPulse, FALLING);
  Serial.print("TCRT5000 digitalRead: ");
  Serial.println(digitalRead(pinTCRT5000));

  pinMode(pinESC, OUTPUT);
  analogWrite(pinESC, pwmValue);

  // Inicialização condicional dos sensores e atuadores opcionais
  #ifdef HAS_ULTRASONIC
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  #endif

  #ifdef HAS_RC
    pinMode(pinRC_CH1, INPUT);
    pinMode(pinRC_CH2, INPUT);
    pinMode(pinRC_CH3, INPUT);
    pinMode(pinRC_CH4, INPUT);
    pinMode(pinServo, OUTPUT);
    servoDirecao.attach(pinServo);
  #endif

  Serial.begin(115200);

  // Inicialização CAN
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN BUS OK!");
    canAvailable = true;
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("CAN BUS FAIL!");
    canAvailable = false;
    // Não trava o loop, apenas desabilita CAN
  }
}

// --- Função para calcular velocidade da roda e imprimir RPM ---
float getWheelSpeed() {
  static unsigned long lastCheck = 0;
  static unsigned long lastCount = 0;
  static float lastSpeed = 0.0;
  static float lastRPM = 0.0;
  unsigned long now = millis();
  float speed = lastSpeed;
  float rpm = lastRPM;
  if (now - lastCheck >= 100) {
    unsigned long deltaCount = pulseCount - lastCount;
    float wheelCircumference = 0.21; // metros
    float deltaTime = (now - lastCheck) / 1000.0; // segundos

    // NOVO: calcula RPM usando o intervalo do último pulso se houver pulso novo
    if (deltaCount > 0 && lastPulseInterval > 0) {
      rpm = 60000.0 / lastPulseInterval; // 1 pulso por volta, 60000 ms/min
      speed = (rpm * wheelCircumference) / 60.0; // m/s
      lastSpeed = speed;
      lastRPM = rpm;
      Serial.print("lastPulseInterval: ");
      Serial.print(lastPulseInterval);
      Serial.print(" ms, RPM: ");
      Serial.println(rpm, 2);
    } else if (deltaCount > 0 && deltaTime > 0) {
      // fallback para cálculo antigo
      rpm = (deltaCount / deltaTime) * 60.0;
      speed = (rpm * wheelCircumference) / 60.0;
      lastSpeed = speed;
      lastRPM = rpm;
      Serial.print("deltaCount: ");
      Serial.print(deltaCount);
      Serial.print(" deltaTime: ");
      Serial.print(deltaTime, 4);
      Serial.print(" RPM: ");
      Serial.println(rpm, 2);
    }
    lastCount = pulseCount;
    lastCheck = now;
  }
  return speed;
}

// --- Funções que dependem de hardware opcional ---
// Use checagens para evitar travamentos se o hardware não estiver presente

float readUltrasonic() {
  #ifdef HAS_ULTRASONIC
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20ms
    float distance = duration * 0.034 / 2.0; // cm
    return distance;
  #else
    return 999; // valor alto = sem obstáculo
  #endif
}

void processCAN() {
  if (!canAvailable) return;
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    // Exemplo: ID 0x100 = controle motor/servo
    if (rxId == 0x100 && len >= 2) {
      can_esc_pwm = rxBuf[0];      // PWM do ESC (0-255)
      can_servo_angle = rxBuf[1];  // Ângulo do servo (0-180)
      Serial.print("CAN RX: ESC="); Serial.print(can_esc_pwm);
      Serial.print(" SERVO="); Serial.println(can_servo_angle);
    }
  }
}

void sendNavigationDataCAN() {
  if (!canAvailable) return;
  // Exemplo: envia wheelSpeed (2 bytes), pulseCount (2 bytes), slip (1 byte * 100)
  float wheelSpeed = getWheelSpeed();
  unsigned int wheelSpeedInt = (unsigned int)(wheelSpeed * 100); // 2 bytes
  unsigned int pulseCountInt = (unsigned int)(pulseCount & 0xFFFF); // 2 bytes
  float slip = 0.0;
  float vehicleSpeed = getVehicleSpeedFromMPU();
  if (vehicleSpeed > 0.1) {
    slip = (wheelSpeed - vehicleSpeed) / vehicleSpeed;
  }
  int slipInt = (int)(slip * 100); // 1 byte

  unsigned char navData[5];
  navData[0] = (wheelSpeedInt >> 8) & 0xFF;
  navData[1] = wheelSpeedInt & 0xFF;
  navData[2] = (pulseCountInt >> 8) & 0xFF;
  navData[3] = pulseCountInt & 0xFF;
  navData[4] = (char)slipInt;

  CAN.sendMsgBuf(0x200, 0, 5, navData);
}

#define CAN_HEARTBEAT_ID 0x400
unsigned long lastHeartbeat = 0;
void sendHeartbeatCAN() {
  if (!canAvailable) return;
  unsigned char data[1] = {0xAA}; // valor arbitrário
  CAN.sendMsgBuf(CAN_HEARTBEAT_ID, 0, 1, data);
}

void safetyStop() {
  float dist = 9999;
  #ifdef trigPin
  if (digitalPinToInterrupt(trigPin) >= 0) {
    dist = readUltrasonic();
  }
  #endif
  if (dist > 0 && dist < 20) { // obstáculo a menos de 20cm
    obstacleDetected = true;
    analogWrite(pinESC, 0); // para o motor
    Serial.println("Obstáculo detectado! Motor bloqueado.");
  } else {
    obstacleDetected = false;
  }

  // Envia mensagem CAN para o Raspberry Pi se houver mudança de estado do obstáculo
  if (canAvailable && autonomousMode && (obstacleDetected != lastObstacleState)) {
    unsigned char obsData[1];
    obsData[0] = obstacleDetected ? 1 : 0;
    CAN.sendMsgBuf(CAN_OBS_ID, 0, 1, obsData);
    lastObstacleState = obstacleDetected;
  }
}

void readSensorAndControlLed() {
  // LED mais rápido e mais forte (HIGH = 5V, sem delay extra)
  if (digitalRead(pinSensor) == LOW) {
    digitalWrite(pinLed, HIGH); // acende o led
  } else {
    digitalWrite(pinLed, LOW); // apaga o led
  }
}

// --- Função para ler PWM RC (pulseIn) ---
int readRCChannel(int pin) {
  // PWM RC típico: 1000us (mín) a 2000us (máx)
  int val = pulseIn(pin, HIGH, 25000); // timeout 25ms
  if (val == 0) val = 1500; // valor neutro se sem sinal
  return val;
}

// --- Atualizar modos a partir dos canais 3 e 4 ---
void updateModes() {
  int ch4 = readRCChannel(pinRC_CH4);
  int ch3 = readRCChannel(pinRC_CH3);
  // CH4: modo (acima de 1500us = autônomo)
  autonomousMode = (ch4 > 1500);
  // CH3: controle de tração (acima de 1500us = ativado)
  tractionControlEnabled = (ch3 > 1500);
}

// --- Função para controle manual (pass-through RC) ---
void manualControl() {
  int acel = readRCChannel(pinRC_CH2);
  int direcao = readRCChannel(pinRC_CH1);
  int escPWM = map(acel, 1000, 2000, 90, 150);
  escPWM = constrain(escPWM, 90, 150);
  int servoAngle = map(direcao, 1000, 2000, 0, 180);
  servoDirecao.write(servoAngle);

  // Permite ré (escPWM < 90) mesmo com obstáculo detectado
  if (!obstacleDetected || escPWM < 90) {
    analogWrite(pinESC, escPWM);
  } else {
    analogWrite(pinESC, 0); // mantém motor parado
  }
}

// --- Função stub para diagnóstico de sensores ---
void checkSensorHealth() {
  // TODO: Implementar diagnóstico real dos sensores
  // Por enquanto, apenas imprime mensagem de diagnóstico
  //Serial.println("Sensor health check: OK");
}

void loop(){
  updateModes();

  // Só executa controles se os pinos existem
  #if defined(pinRC_CH1) && defined(pinRC_CH2) && defined(pinServo)
  if (autonomousMode) {
    autonomousControl();
  } else {
    manualControl();
    if (canAvailable && millis() - lastCanNavSend > 100) {
      sendNavigationDataCAN();
      lastCanNavSend = millis();
    }
  }
  #endif

  safetyStop();
  readSensorAndControlLed();

  // [NOVO] Heartbeat CAN
  if (canAvailable && millis() - lastHeartbeat > 500) {
    sendHeartbeatCAN();
    lastHeartbeat = millis();
  }

  // [NOVO] Diagnóstico de sensores
  checkSensorHealth();

  // DEBUG: Mostra o valor do pino do sensor para ajudar no diagnóstico
  // Só imprime se pulso detectado ou valor do sensor mudou
  static int lastTcrtValue = -1;
  int tcrtValue = digitalRead(pinTCRT5000);
  if (tcrtValue != lastTcrtValue) {
    Serial.print("TCRT5000 raw: ");
    Serial.println(tcrtValue);
    lastTcrtValue = tcrtValue;
  }
  static unsigned long lastPulseCount = 0;
  if (pulseCount != lastPulseCount) {
    Serial.print("pulseCount: ");
    Serial.println(pulseCount);
    lastPulseCount = pulseCount;
  }

  getWheelSpeed();

  // Remove o delay(50) para resposta mais rápida do LED e cálculo de RPM
  // delay(50); // ajuste conforme necessário
}