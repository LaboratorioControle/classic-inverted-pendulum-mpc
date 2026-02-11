// ==============================
// INCLUSÃO DAS BIBLIOTECAS
// ==============================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <MPC.h>

// ==============================
// CONFIGURAÇÃO DOS PINOS
// ==============================
#define ENCODER_PEND_A 33
#define ENCODER_PEND_B 25
#define ENCODER_MOT_A 14
#define ENCODER_MOT_B 13

#define POTENCIOMETRO 34

#define MOTOR_PWM1 27
#define MOTOR_PWM2 26

#define BOT_LIGA 4
#define BOT_DESLIGA 15

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ==============================
// VARIÁVEIS DE LEITURA DOS ENCODERS
// ==============================

// Resolução do encoder
const int PULSOS_POR_REV_PEND = 600;  

// PRECISA VERIFICAR SE PRECISA
const int RESOLUCAO_PEND = PULSOS_POR_REV_PEND * 4;  // Quadratura 4x

// Armazena a contagem de pulsos
volatile long encoderPendCount = 0;     
volatile long encoderMotCount = 0;

// Armazena a última contagem de pulsos para permitir 
volatile int lastEncodedPend = 0;
volatile int lastEncodedMot = 0;

const float FATOR_CONV_DIST = 145.366; //Pulsos pos cm


// ==============================
// VARIÁVEIS DOS SINAIS DE ENSAIO
// ==============================

// Variáveis do degrau
volatile bool degrauAtivo = false;
unsigned long tempoDegrauInicio = 0;
unsigned long duracaoDegrau = 100;    // ms
uint8_t intensidadeDegrau = 0;
char sentidoDegrau = 'R';

// Variáveis do seno
volatile bool senoideAtiva = false;
unsigned long tempoSenoInicio = 0;
float amplitudeSeno = 0.0;            // 0-255
float frequenciaSeno = 0.5;           // Hz
unsigned long duracaoSeno = 1000;     // ms

// Variáveis de comando manual
volatile char comandoManual = 'P';
volatile uint8_t pwmManual = 180;


// ==============================
// VARIÁVEIS DO CONTROLE LQR
// ==============================

// Variáveis do controlador LQR
volatile bool controleLQRAtivo = false;
float K[4] = {-15, 140, -80, 20};
float K_swing = 30;

// Limiares de troca
const float THETA_SWITCH = 15 * PI/180.0;       
const float THETA_DOT_SWITCH = 100 * PI/180.0;  
const float FIM_CURSO_VIRTUAL =  24.0/100.0; 

// Setpoint posição
float set_point_x = 0.0;

// Comando via botões físicos
bool ajustandoPosicao = false; 
bool emAjuste = false;
bool ajustou = false;
unsigned long inicioCombo = 0;
bool comboDetectado = false;


// ==============================
// VARIÁVEIS DO CONTROLE MPC
// ==============================
volatile bool controleMPCAtivo = false;
MPC mpc = MPC(MPCForm::EXPONENCIAL, 30);
float pos_limite = 20.0/100.0;
float ang_limite = 12.0 * (PI/180.0);
float vel_limite = 50.0/100.0;
float comando_limite = 12.0;
float ulast = 0;

// ==============================
// DADOS DA PLANTA
// ==============================
const float PERIODO = 10.0;  // ms

const float m = 0.0205;       // Massa pêndulo
const float l = 0.18;         // Distância até o centro de massa
const float g = 9.81;         // Gravidade
const float I = 0.000207;     // Momento de Inércia
const float M = 0.3088;       // Massa carrinho
const float b = 0.000008;     // Atrito viscoso do pêndulo
const float c = 6.0;          // Atrito viscoso do carrinho
const float kt = 0.175;       // Constante de torque do motor
const float kb = 0.04;        // Constante de força eletromotriz
const float Rm = 10.5;        // Resistência do motor
const float r  = 0.071;       // Raio da polia
const float guia = 30.0;      // Tamanho da guia (cm)


// ==============================
// ESTADOS DA PLANTA
// ==============================
float x = 0.0;          // Posição (m)
float x_dot = 0.0;      // Velocidade do carro (m/s)
float theta = 0.0;      // Ângulo (rad)
float theta_dot = 0.0;  // Velocidade angular (rad/s)


// ==============================
// ENVIO DE DADOS PARA O MATLAB
// ==============================
typedef struct {
  uint32_t t_ms;
  float theta_deg;
  float theta_dot;
  float x_cm;
  float x_dot_cm;
  float u;
} LogData;

QueueHandle_t filaLog;

// ==============================
// INTERRUPÇÕES DOS ENCODERS
// ==============================
void IRAM_ATTR updateEncoderPend() {
  int MSB = digitalRead(ENCODER_PEND_A);
  int LSB = digitalRead(ENCODER_PEND_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedPend << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPendCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPendCount--;

  lastEncodedPend = encoded;
}


void IRAM_ATTR updateEncoderMot() {
  int MSB = digitalRead(ENCODER_MOT_A);
  int LSB = digitalRead(ENCODER_MOT_B);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedMot << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderMotCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderMotCount--;

  lastEncodedMot = encoded;
}


// ==============================
// FUNÇÃO PARA CONTROLAR DEGRAU
// ==============================
void aplicaDegrau() {
  if (degrauAtivo) {
    if (sentidoDegrau == 'R') {
      ledcWrite(0, intensidadeDegrau);  // MOTOR_PWM1
      ledcWrite(1, 0);                  // MOTOR_PWM2
    } else {
      ledcWrite(0, 0);                  // MOTOR_PWM1
      ledcWrite(1, intensidadeDegrau);  // MOTOR_PWM2
    }

    if (millis() - tempoDegrauInicio >= duracaoDegrau) {
      degrauAtivo = false;           
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}


// ==============================
// FUNÇÃO PARA CONTROLAR MANUALMENTE
// ==============================
void controleManual() {
  if (comandoManual == 'L') {
    ledcWrite(0, 0);
    ledcWrite(1, pwmManual);
  }
  else if (comandoManual == 'R') {
    ledcWrite(0, pwmManual);
    ledcWrite(1, 0);
  }
  else if (comandoManual == 'P'){
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}


// ==============================
// FUNÇÃO PARA CONTROLAR SENOIDE
// ==============================
void aplicaSenoide() {
  if (senoideAtiva) {
    float t = (millis() - tempoSenoInicio) / 1000.0;  // tempo em segundos
    float s = sin(2 * PI * frequenciaSeno * t);       // valor da senoide [-1,1]
    int pwm = (int)(amplitudeSeno * abs(s));          // PWM sempre positivo

    if (s >= 0) {
      ledcWrite(0, pwm);  // MOTOR_PWM1 
      ledcWrite(1, 0);    // MOTOR_PWM2 
    } else {
      ledcWrite(0, 0);    // MOTOR_PWM1 
      ledcWrite(1, pwm);  // MOTOR_PWM2 
    }

    if (millis() - tempoSenoInicio >= duracaoSeno) {
      senoideAtiva = false;
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }

  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}


// ==============================
// CONTROLADOR POR SWING UP
// ==============================
float sign(float x_cop) {
    if (x_cop > 0) return 1.0;
    else if (x_cop < 0) return -1.0;
    else return 0.0;
}

float Force2Volt(float F) {
    float volt = (F * Rm * r * r + kt * kb * x_dot) / (kt * r);
    return volt;
}

float swingUpController() {
    float E = m*g*l*(1 - cos(theta)) + 0.5*(I + m*l*l)*(theta_dot*theta_dot);
    float E_des = 2*m*g*l;  // topo

    float arg = theta_dot * cos(theta);

    float k_energy = K_swing * g;

    float x_2dot_desejado = k_energy * (E - E_des) * sign(arg) - 8*x;
    
    float theta_2dot = (-b * theta_dot
                        - m * l * cos(theta) * x_2dot_desejado
                        - m * g * l * sin(theta))
                       / (I + m * l * l);

    // === Cálculo da força F no carrinho ===
    float F = (M + m) * x_2dot_desejado
                + m * l * cos(theta) * theta_2dot
                - m * l * sin(theta) * (theta_dot * theta_dot)
                + c * x_dot;

    return Force2Volt(F);
}


// ==============================
// FUNÇÃO PARA CONTROLE LQR
// ==============================
void controleEstadoLQR() {

  float erroX = x - (set_point_x / 100.0f);  // converte cm → metros, se sua pos está em m
  float erroTheta = theta - PI; 

  float u = 0;

  bool emZonaPerigo = abs(x) >= FIM_CURSO_VIRTUAL;
  bool emRegiaoLQR = (abs(erroTheta) < THETA_SWITCH) && (abs(theta_dot) < THETA_DOT_SWITCH);

  if (emZonaPerigo){
    u = - K[3] * erroX;
  }else if(emRegiaoLQR){
    u = -(K[0]*erroX + K[1]*erroTheta + K[2]*x_dot + K[3]*theta_dot);
  }else{
    u = swingUpController();
  }

  u = constrain(u, -12.0, 12.0);
  ulast = u;
  float u_pwm = (u / 12.0) * 255.0;
  
  if (u_pwm >= 0) {
    ledcWrite(0, (int)u_pwm);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, (int)(-u_pwm));
  }
}

void desativaControladorLQR(){
  controleLQRAtivo = false;
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void ativaControladorLQR(){
  controleLQRAtivo = true;
  degrauAtivo = false;
  senoideAtiva = false;

  if (theta <= 1e-2) {
    ledcWrite(1, 200);
    vTaskDelay(pdMS_TO_TICKS(50));
    ledcWrite(1, 0);
  }
}


// ==============================
// FUNÇÃO PARA CONTROLE MPC
// ==============================
void setupMPC(){
  // =========================
  // MATRIZES DO MODELO
  // =========================
  mpc.A = Matrix(4,4);
  mpc.A(0,0)=1.0000; mpc.A(0,1)=0.0000; mpc.A(0,2)=0.0091;  mpc.A(0,3)=0.0000;
  mpc.A(1,0)=0.0000; mpc.A(1,1)=1.0022; mpc.A(1,2)=-0.0039; mpc.A(1,3)=0.0100;
  mpc.A(2,0)=0.0000; mpc.A(2,1)=0.0044; mpc.A(2,2)=0.8224;  mpc.A(2,3)=0.0000;
  mpc.A(3,0)=0.0000; mpc.A(3,1)=0.4346; mpc.A(3,2)=-0.7526; mpc.A(3,3)=1.0021;

  mpc.B = Matrix(4,1);
  mpc.B(0,0)=0.0000;
  mpc.B(1,0)=0.0001;
  mpc.B(2,0)=0.0068;
  mpc.B(3,0)=0.0288;

  // =========================
  // MATRIZ DE SAÍDA
  // =========================
  mpc.Cr = Matrix(2,4);
  mpc.Cr(0,0)=1; mpc.Cr(0,1)=0; mpc.Cr(0,2)=0; mpc.Cr(0,3)=0;
  mpc.Cr(1,0)=0; mpc.Cr(1,1)=1; mpc.Cr(1,2)=0; mpc.Cr(1,3)=0;

  mpc.Cc = Matrix(3,4);
  mpc.Cc(0,0)=1; mpc.Cc(0,1)=0; mpc.Cc(0,2)=0; mpc.Cc(0,3)=0;
  mpc.Cc(1,0)=0; mpc.Cc(1,1)=1; mpc.Cc(1,2)=0; mpc.Cc(1,3)=0;
  mpc.Cc(2,0)=0; mpc.Cc(2,1)=0; mpc.Cc(2,2)=1; mpc.Cc(2,3)=0;

  // =========================
  // PESOS DO MPC
  // =========================
  mpc.Qy = Matrix(2,2);
  mpc.Qy(0,0)=320; mpc.Qy(0,1)=0;
  mpc.Qy(1,0)=0; mpc.Qy(1,1)=120;

  mpc.Qu = Matrix(1,1);
  mpc.Qu(0,0) = 0.001;

  // =========================
  // LIMITES
  // =========================
  mpc.ycmax = Matrix(3,1);
  mpc.ycmax(0,0)= pos_limite;
  mpc.ycmax(1,0)= ang_limite;
  mpc.ycmax(2,0)= vel_limite;

  mpc.ycmin = Matrix(3,1);
  mpc.ycmin(0,0)= -pos_limite;
  mpc.ycmin(1,0)= -ang_limite;
  mpc.ycmin(2,0)= -vel_limite;

  mpc.umax = Matrix(1,1); 
  mpc.umax(0,0) = comando_limite;

  mpc.umin = Matrix(1,1); 
  mpc.umin(0,0) = -comando_limite;

  mpc.deltamax = Matrix(1,1); 
  mpc.deltamax(0,0) = 1e5;

  mpc.deltamin = Matrix(1,1); 
  mpc.deltamin(0,0) = -1e5;

  // =========================
  // CALCULA MATRIZES
  // =========================
  //float pontos[7] = {1, 5, 10, 15, 20, 25, 30};
  //mpc.compute_MPC_Matrices(pontos);

  float lambda[1] = {0.01f};
  float alpha = 0.5f;
  float tau = PERIODO/1000;
  mpc.compute_MPC_Matrices(lambda, alpha, tau);
}

void controleEstadoMPC() {

  float erroX = x - (set_point_x / 100.0f);  // converte cm → metros, se sua pos está em m
  float erroTheta = theta - PI; 

  float u = 0;

  bool emZonaPerigo = abs(x) >= FIM_CURSO_VIRTUAL;
  bool emRegiaoMPC = (abs(erroTheta) < THETA_SWITCH) && (abs(theta_dot) < THETA_DOT_SWITCH);

  if (emZonaPerigo){
    u = - (K[1] * erroX + K[3] * x_dot);
  }else if(emRegiaoMPC){
    float estados[4] = {x, erroTheta, x_dot, theta_dot};

    float spt[2] = {0.0f, 0.0f};
    u = mpc.compute_MPC_Command(ulast, spt, estados)[0];
  }else{
    u = swingUpController();
  }

  u = constrain(u, -12.0, 12.0);
  ulast = u;
  float u_pwm = (u / 12.0) * 255.0;
  
  if (u_pwm >= 0) {
    ledcWrite(0, (int)u_pwm);
    ledcWrite(1, 0);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, (int)(-u_pwm));
  }
}

void desativaControladorMPC(){
  controleMPCAtivo = false;
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void ativaControladorMPC(){
  controleMPCAtivo = true;

  if (theta <= 1e-2) {
    ledcWrite(1, 100);
    vTaskDelay(pdMS_TO_TICKS(10));
    ledcWrite(1, 0);
  }
}

// ==============================
// FUNÇÃO PARA ATIVAÇÃO PELOS BOTÕES FÍSICOS
// ==============================
void ajustaPosInicial() {
    int leitura = analogRead(POTENCIOMETRO);
    const int centro = 2048;
    const int zonaMorta = 150;
    int erro = leitura - centro;

    if (abs(erro) < zonaMorta) {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        ajustandoPosicao = false;
        return;
    }

    float Kp = 0.1;
    int velocidade = abs(erro) * Kp;
    velocidade = constrain(velocidade, 0, 255);

    if (erro < 0) {
        ledcWrite(1, 0);
        ledcWrite(0, velocidade); // DIREITA
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, velocidade); // ESQUERDA
    }
}

void ativaAjustePosInicial() {
    controleLQRAtivo = false;
    degrauAtivo = false;
    senoideAtiva = false;
    comandoManual = 'P';

    ajustandoPosicao = true;
    ajustou = true;  // Flag para zerar o encoder
}

void gerenciaBotoes() {

    bool liga = digitalRead(BOT_LIGA);
    bool desliga = digitalRead(BOT_DESLIGA);
    unsigned long agora = millis();

    // ===============================
    // ENTRADA NO AJUSTE
    // ===============================
    if (!emAjuste && liga && desliga) {

        if (!comboDetectado) {
            comboDetectado = true;
            inicioCombo = agora;
        }

        if (agora - inicioCombo >= 300) {
            ativaAjustePosInicial();
            emAjuste = true;
        }
        return;
    }

    comboDetectado = false;

    // ===============================
    // SAÍDA DO AJUSTE
    // ===============================
    if (emAjuste) {
        if (desliga && !liga) {
            ajustandoPosicao = false;
            emAjuste = false;

            if (ajustou) {
                encoderPendCount = 0;
                encoderMotCount = 0;
                lastEncodedPend = 0;
                lastEncodedMot = 0;
                ajustou = false;
            }
        }
        return;
    }

    // ===============================
    // ESTADO NORMAL
    // ===============================
    if (liga && !desliga) {
        //K[0] = -15; K[1] = 140; K[2] = -80; K[3] = 20; K_swing = 25;
        //ativaControladorLQR();
        ativaControladorMPC();
    }

    if (desliga && !liga) {
        //desativaControladorLQR();
        desativaControladorMPC();
        degrauAtivo = false;
        senoideAtiva = false;
    }
}


// ======================================================================
//  DISPLAY OLED
// ======================================================================
void telaBoasVindas() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(20, 8);
  display.println("Projeto");

  display.setCursor(10, 22);
  display.println("Lab Integrador");

  display.setCursor(0, 48);
  display.println("Inicializando...");

  display.display();
  delay(2500);
}

void telaCalibrandoPendulo() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 8);
  display.println("Calibrando pendulo...");
  display.setCursor(0, 28);
  display.println("Deixe em repouso");

  display.display();
}

void atualizarDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ----- Titulo centralizado -----
  display.setTextSize(1);
  const char titulo[] = "Projeto LabIntegrador";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(titulo, 0, 0, &x1, &y1, &w, &h);
  int16_t xTitulo = (SCREEN_WIDTH - w) / 2;
  display.setCursor(xTitulo, 0);
  display.print(titulo);

  // Linha separadora
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  // ----- Bloco MOTOR -----
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print("Posição");

  display.setTextSize(2);
  display.setCursor(50, 24);
  display.print(x*100, 1);

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("cm");

  display.drawLine(0, 42, SCREEN_WIDTH - 1, 42, SSD1306_WHITE);

  // ----- Bloco PENDULO -----
  display.setTextSize(1);
  display.setCursor(0, 46);
  display.print("Set Point");

  display.setTextSize(2);
  display.setCursor(50, 52 - 8);
  display.print(set_point_x, 1);

  display.setTextSize(1);
  display.setCursor(0, 62 - 4);
  display.print("deg");

  display.display();
}


// ==============================
// TAREFA DE AMOSTRAGEM (10 ms)
// ==============================
void taskLeitura(void *parameter) {
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Variáveis auxiliares para cálculo de velocidade
  float theta_ant = 0.0;
  float x_ant  = 0.0;
  float tempo_s = 0.0;

  while (true) {
    vTaskDelayUntil(&xLastWakeTime, periodo);

    if(emAjuste){
      ajustaPosInicial();
    } else if(controleMPCAtivo){
      controleEstadoMPC();
    } else if(controleLQRAtivo){
      controleEstadoLQR();
    } else if(degrauAtivo){
      aplicaDegrau();
    } else if(senoideAtiva){
      aplicaSenoide();
    } else {
      controleManual();
    }

    gerenciaBotoes();

    tempo_s = millis() / 1000.0;

    // Cópias locais (evita conflito com interrupções)
    long countMot = encoderMotCount;
    long countPend = encoderPendCount;

    // Cálculo da posição em metros e da velocidade em m/s
    x = (float) countMot / (FATOR_CONV_DIST * 100.0f);
    x_dot = (x - x_ant) / (PERIODO / 1000.0);
    x_ant = x;

    // Cálculo do ângulo entre 0 e 2π
    long theta_bruto = countPend % RESOLUCAO_PEND;
    if (theta_bruto < 0) theta_bruto += RESOLUCAO_PEND;  // Garante faixa positiva
    theta = (theta_bruto * 2 * PI) / RESOLUCAO_PEND;  // 0 a 2π

    // Cálculo da velocidade angular (trata salto 2π → 0)
    float delta_theta = theta - theta_ant;

    // Se houve passagem pelo zero:
    if (delta_theta > PI)       delta_theta -= 2*PI;
    else if (delta_theta < -PI) delta_theta += 2*PI;

    theta_dot = delta_theta / (PERIODO / 1000.0) ;
    theta_ant = theta;

    // Leitura do potênciometro para definição do set point da posição
    int leituraPot = analogRead(POTENCIOMETRO);
    set_point_x = ((float)leituraPot / 4095.0f) * guia - guia/2.0;

    // Envio de dados para o MatLab para plot online
    LogData log;

    log.t_ms       = millis();
    log.theta_deg  = theta * 180.0f / PI;
    log.theta_dot  = theta_dot * 180.0f / PI;
    log.x_cm       = x * 100.0f;
    log.x_dot_cm   = x_dot * 100.0f;
    log.u = ulast;
            
    // Envia para a fila (não bloqueia)
    xQueueSend(filaLog, &log, 0);
  }
}


// ==============================
// VERIFICA COMANDOS DE ENTRADA VIA SERIAL
// ==============================
void taskSerialRx(void *parameter) {
    String buffer = "";
    while (true) {
        while (Serial.available()) {
            char c = Serial.read();

            // === 1. PROCESSAMENTO DE LINHA (Terminador '\n' ou '\r') ===
            if (c == '\n' || c == '\r') {
                
                // Processa o buffer se ele não estiver vazio
                if (buffer.length() > 0) {
                    
                    // Converte o primeiro caractere para maiúsculo para facilitar a comparação
                    char comando = buffer.charAt(0);
                    if (comando >= 'a' && comando <= 'z') {
                        comando = comando - 32; // Converte para maiúsculo
                    }

                    // COMANDO DE DEGRAU (Ex: D,500,200,R)
                    if (comando == 'D') {
                        // A lógica original de substring é mantida aqui.
                        int idx1 = buffer.indexOf(',');
                        int idx2 = buffer.indexOf(',', idx1 + 1);
                        int idx3 = buffer.lastIndexOf(',');
                        
                        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                            String t = buffer.substring(idx1 + 1, idx2);
                            String v = buffer.substring(idx2 + 1, idx3);
                            String s = buffer.substring(idx3 + 1);

                            s.toUpperCase(); // Garante 'L' ou 'R' maiúsculo
                            
                            duracaoDegrau = t.toInt();
                            intensidadeDegrau = constrain(v.toInt(), 0, 255);
                            sentidoDegrau = (s == "L") ? 'L' : 'R'; // L ou R

                            degrauAtivo = true;
                            senoideAtiva = false; // Desativa a senoide se o degrau for ativado
                            tempoDegrauInicio = millis();
                            
                            Serial.printf("Comando Degrau recebido: Duração=%ldms, Intensidade=%d, Sentido=%c\n", 
                                          duracaoDegrau, intensidadeDegrau, sentidoDegrau);
                        } else {
                             Serial.println("Erro: Comando D com formato inválido.");
                        }
                    }

                    // COMANDO SENOIDE: S,amplitude,frequencia
                    else if (comando == 'S') { 
                      int idx1 = buffer.indexOf(',');
                      int idx2 = buffer.indexOf(',', idx1 + 1);
                      int idx3 = buffer.lastIndexOf(',');

                      if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                          amplitudeSeno = constrain(buffer.substring(idx1 + 1, idx2).toInt(), 0, 255);
                          frequenciaSeno = buffer.substring(idx2 + 1, idx3).toFloat();
                          duracaoSeno = buffer.substring(idx3 + 1).toFloat();  // Novo: duração em ms ou s

                          senoideAtiva = true;
                          degrauAtivo = false; // Desativa o degrau se a senoide for ativada
                          tempoSenoInicio = millis();
                      }
                    }

                    else if (comando == 'Q') {
                        int idx1 = buffer.indexOf(',');
                        int idx2 = buffer.indexOf(',', idx1 + 1);
                        int idx3 = buffer.indexOf(',', idx2 + 1);
                        int idx4 = buffer.indexOf(',', idx3 + 1);
                        int idx5 = buffer.lastIndexOf(',');

                        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3 && idx5 > idx4) {
                            K[0] = buffer.substring(idx1 + 1, idx2).toFloat();
                            K[1] = buffer.substring(idx2 + 1, idx3).toFloat();
                            K[2] = buffer.substring(idx3 + 1, idx4).toFloat();
                            K[3] = buffer.substring(idx4 + 1, idx5).toFloat();
                            K_swing = buffer.substring(idx5 + 1).toFloat();

                            ativaControladorLQR();
                        }
                    }

                    else if (comando == 'X') {
                      desativaControladorLQR();
                    }

                    // Verifica se o buffer tem APENAS 1 caractere (L, R ou P)
                    else if (buffer.length() == 1) { 

                      // COMANDOS SIMPLES: L / R / P
                        if (comando == 'L' || comando == 'R' || comando == 'P') {
                            
                          comandoManual = comando;
                            
                          // Desativa degrau/senoide se um comando manual for enviado
                          degrauAtivo = false;
                          senoideAtiva = false;
                        } else if(comando == 'Z') {
                          encoderPendCount = 0;
                          encoderMotCount = 0;
                          lastEncodedPend = 0;
                          lastEncodedMot = 0;
                        }
                    }
                    
                    // Limpa o buffer após o processamento, independentemente do sucesso
                    buffer = "";
                }
            } else {
                // Adiciona o caractere ao buffer (Ignora '\n' e '\r' do input)
                buffer += c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequena pausa para permitir que outras tasks rodem
    }
}


// ==============================
// ENVIA O BUFFER DE DADOS VIA SERIAL
// ==============================
void taskSerialTx(void *parameter) {
  LogData log;
  uint8_t header[2] = {0xAA, 0x55};

  vTaskDelay(pdMS_TO_TICKS(2000));

  while (true) {
    if (xQueueReceive(filaLog, &log, portMAX_DELAY) == pdTRUE) {
      Serial.write(header, 2);
      Serial.write((uint8_t*)&log, sizeof(LogData));
    }
  }
}

// ======================================================================
// TAREFA DE ATUALIZAÇÃO DO DISPLAY OLED
// ======================================================================
void taskDisplay(void *parameter) {
    const TickType_t displayPeriodo = pdMS_TO_TICKS(100); // 100ms (10Hz)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, displayPeriodo);

        atualizarDisplay();
    }
}


// ==============================
// CONFIGURAÇÃO INICIAL
// ==============================
void setup() {

  // Criação da fila para comunicação com o MatLab em tempo real
  filaLog = xQueueCreate(200, sizeof(LogData)); // buffer de 200 amostras
  
  // Inicialização da comunicação serial
  Serial.begin(115200);
  Wire.begin(21, 22);

  //Inicia Controlador MPC
  setupMPC();

  // Inicialização do Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("Falha ao iniciar display!");
  }
  telaBoasVindas();

  // Definição dos pinos
  pinMode(ENCODER_PEND_A, INPUT);
  pinMode(ENCODER_PEND_B, INPUT);
  pinMode(ENCODER_MOT_A, INPUT);
  pinMode(ENCODER_MOT_B, INPUT);
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  pinMode(POTENCIOMETRO, INPUT);


  // PWM MOTOR - Esquerda
  ledcSetup(0, 10000, 8);        // Canal 0, 10kHz, 8 bits
  ledcAttachPin(MOTOR_PWM1, 0);

  // PWM MOTOR - Direita
  ledcSetup(1, 10000, 8);        // Canal 1, 10kHz, 8 bits
  ledcAttachPin(MOTOR_PWM2, 1);

  // Interrupções
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_A), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_B), updateEncoderPend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_A),  updateEncoderMot,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_MOT_B),  updateEncoderMot,  CHANGE);

  // Cria tarefa FreeRTOS
  xTaskCreatePinnedToCore(taskLeitura, "TaskLeitura", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskDisplay, "TaskDisplay", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSerialRx,   "TaskSerialRx", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSerialTx, "TaskSerialTx", 4096, NULL, 1, NULL, 0);  
}

void loop() {
}