#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <mcp_can.h>

// --- Configuração dos sensores e atuadores ---
const int pinTCRT5000 = 3; // Sensor óptico na roda (interrupção)
volatile unsigned long pulseCount = 0;
unsigned long lastPulseTime = 0;

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
void IRAM_ATTR onPulse() {
  pulseCount++;
  lastPulseTime = millis();
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

// --- Variáveis para comandos CAN ---
int can_esc_pwm = 120;
int can_servo_angle = 90;

// --- Inicialização ---
void setup(){ 
  pinMode(pinSensor, INPUT); // define como entrada 
  pinMode(pinLed, OUTPUT); // definoe como saída   
  digitalWrite(pinLed, LOW); // led incia apagado
  
  pinMode(pinTCRT5000, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinTCRT5000), onPulse, FALLING);

  pinMode(pinESC, OUTPUT);
  analogWrite(pinESC, pwmValue);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(pinRC_CH1, INPUT);
  pinMode(pinRC_CH2, INPUT);
  pinMode(pinRC_CH3, INPUT);
  pinMode(pinRC_CH4, INPUT);
  pinMode(pinServo, OUTPUT);
  servoDirecao.attach(pinServo);

  Serial.begin(115200);

  // Inicialização CAN
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN BUS OK!");
  } else {
    Serial.println("CAN BUS FAIL!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
}

// --- Função para calcular velocidade da roda ---
float getWheelSpeed() {
  static unsigned long lastCheck = 0;
  static unsigned long lastCount = 0;
  unsigned long now = millis();
  float speed = 0.0;
  if (now - lastCheck >= 100) { // a cada 100ms
    unsigned long deltaCount = pulseCount - lastCount;
    // Supondo 1 pulso por volta, ajuste conforme encoder
    float wheelCircumference = 0.21; // metros (ajuste conforme roda)
    speed = (deltaCount * wheelCircumference * 10); // m/s (10x por segundo)
    lastCount = pulseCount;
    lastCheck = now;
  }
  return speed;
}

// --- Função para ler distância do HC-SR04 ---
float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20ms
  float distance = duration * 0.034 / 2.0; // cm
  return distance;
}

// --- Controle de tração e segurança ---
void tractionControl() {
  float wheelSpeed = getWheelSpeed();
  float vehicleSpeed = getVehicleSpeedFromMPU();
  float slip = 0.0;
  if (vehicleSpeed > 0.1) {
    slip = (wheelSpeed - vehicleSpeed) / vehicleSpeed;
  }
  // Se slip > 20%, reduzir PWM do motor
  if (slip > 0.2) {
    pwmValue = max(pwmValue - 5, 90); // reduz PWM, mínimo 90
    analogWrite(pinESC, pwmValue);
    Serial.println("Slip detectado! Reduzindo PWM.");
  } else if (slip < 0.05 && pwmValue < 150) {
    pwmValue = min(pwmValue + 1, 150); // aumenta PWM devagar
    analogWrite(pinESC, pwmValue);
  }
  Serial.print("WheelSpeed: "); Serial.print(wheelSpeed);
  Serial.print(" VehicleSpeed: "); Serial.print(vehicleSpeed);
  Serial.print(" Slip: "); Serial.println(slip);
}

// --- Parada automática por obstáculo ---
#define CAN_OBS_ID 0x300  // ID para mensagem de obstáculo

bool obstacleDetected = false;
bool lastObstacleState = false;

void safetyStop() {
  float dist = readUltrasonic();
  if (dist > 0 && dist < 20) { // obstáculo a menos de 20cm
    obstacleDetected = true;
    analogWrite(pinESC, 0); // para o motor
    Serial.println("Obstáculo detectado! Motor bloqueado.");
  } else {
    obstacleDetected = false;
  }

  // Envia mensagem CAN para o Raspberry Pi se houver mudança de estado do obstáculo
  if (autonomousMode && (obstacleDetected != lastObstacleState)) {
    unsigned char obsData[1];
    obsData[0] = obstacleDetected ? 1 : 0;
    CAN.sendMsgBuf(CAN_OBS_ID, 0, 1, obsData);
    lastObstacleState = obstacleDetected;
  }
}

void readSensorAndControlLed() {
  if (digitalRead(pinSensor) == LOW){ // se for LOW 
        digitalWrite(pinLed, HIGH); // acende o led
  }else{ // caso contrário
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

void processCAN() {
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

void autonomousControl() {
  processCAN();
  servoDirecao.write(can_servo_angle);
  // Permite ré (can_esc_pwm < 90) mesmo com obstáculo detectado
  if (!obstacleDetected || can_esc_pwm < 90) {
    if (tractionControlEnabled) {
      tractionControl();
    } else {
      analogWrite(pinESC, can_esc_pwm);
    }
  } else {
    analogWrite(pinESC, 0); // mantém motor parado
  }
}

unsigned long lastCanNavSend = 0;

// --- Envio dos dados de navegação via CAN ---
void sendNavigationDataCAN() {
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
  unsigned char data[1] = {0xAA}; // valor arbitrário
  CAN.sendMsgBuf(CAN_HEARTBEAT_ID, 0, 1, data);
}

void checkSensorHealth() {
  static unsigned long lastPulse = 0;
  if (millis() - lastPulseTime > 1000) {
    // Falha no encoder (sem pulsos recentes)
    digitalWrite(pinLed, HIGH); // pisca LED como alerta
  }
  // Adicionar lógica para outros sensores se necessário
}

void loop(){
  updateModes();
  if (autonomousMode) {
    autonomousControl();
  } else {
    manualControl();
    if (millis() - lastCanNavSend > 100) {
      sendNavigationDataCAN();
      lastCanNavSend = millis();
    }
  }
  safetyStop();
  readSensorAndControlLed();

  // [NOVO] Heartbeat CAN
  if (millis() - lastHeartbeat > 500) {
    sendHeartbeatCAN();
    lastHeartbeat = millis();
  }

  // [NOVO] Diagnóstico de sensores
  checkSensorHealth();

  delay(50); // ajuste conforme necessário
}