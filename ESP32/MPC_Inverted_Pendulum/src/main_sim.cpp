// ==============================
// INCLUSÃO DAS BIBLIOTECAS
// ==============================
#include <MPC.h>

// ==============================
// VARIÁVEIS DO CONTROLE LQR
// ==============================

// Variáveis do controlador LQR
float K[4] = {0, 0, 0, 0};
float K_swing = 44;

// Limiares de troca
const float THETA_SWITCH = 12 * PI/180.0;       
const float THETA_DOT_SWITCH = 100 * PI/180.0;  
const float FIM_CURSO_VIRTUAL =  0.20; 

// Setpoint posição
float set_point_x = 0.0;

// ==============================
// VARIÁVEIS DO CONTROLE MPC
// ==============================
MPC mpc = MPC(MPCForm::LINEAR, 100);
float pos_limite = 20.0/100.0;
float ang_limite = 12.0 * (PI/180.0);
float vel_limite = 45.0/100.0;
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
  mpc.Qy(0,0)=800; mpc.Qy(0,1)=0;
  mpc.Qy(1,0)=0; mpc.Qy(1,1)=200;

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
  float pontos[9] = {1, 4, 8, 12, 16, 20, 24, 28, 32};
  mpc.compute_MPC_Matrices(pontos);
}


struct DadosSistema {
    struct { float g, Ts, Tf, spt, angulo_troca, guia; } geral;
    struct { float theta0, theta_dot0, x0, x_dot0; } inicial;

    struct { float m, l, I, b, r_massa, E_des; } pendulo;
    struct { float m, c, l, h; } carro;
    struct { float Rm, Kb, Kt, R; } motor;
};

DadosSistema dados;

float Force2Volt2(float F, float x3) {
    float volt = (F * Rm * r * r + kt * kb * x3) / (kt * r);
    return volt;
}

float Volt2Force(float volt, float x3) {
    float force = (kt*volt*r - kt*kb*x_dot)/(Rm*(r*r));
    return force;
}

void Modelo_Continuo(float x[4], float u, float dx[4], const DadosSistema &d)
{
    float theta1 = x[1];
    float x_dot1 = x[2];
    float theta_dot1 = x[3];

    float m = d.pendulo.m;
    float l = d.pendulo.l;
    float I = d.pendulo.I;
    float b = d.pendulo.b;

    float M = d.carro.m;
    float c = d.carro.c;

    float g = d.geral.g;

    // CONVERTER u (Volts) → Força
    float F = Volt2Force(u,x_dot1);  

    float alpha = (m*m*l*l*(sin(theta1)*sin(theta1)) +
                    M*m*l*l + (M+m)*I);

    float x2dd =
        ( b*m*l*theta_dot1*cos(theta1)
        + m*m*l*l*g*sin(theta1)*cos(theta1)
        + (I + m*l*l)*(F - c*x_dot1 + m*l*sin(theta1)*theta_dot1*theta_dot1)
        ) / alpha;

    float theta2dd =
        -( F*m*l*cos(theta1)
         - c*m*l*x_dot1*cos(theta1)
         + m*m*l*l*theta_dot1*theta_dot1*sin(theta1)*cos(theta1)
         + (M+m)*(b*theta_dot1 + m*g*l*sin(theta1))
         ) / alpha;

    dx[0] = x_dot1;
    dx[1] = theta_dot1;
    dx[2] = x2dd;
    dx[3] = theta2dd;
}


void RK4_discreto(float x[4], float u, float Ts, const DadosSistema &d, float x_next[4])
{
    float k1[4], k2[4], k3[4], k4[4], temp[4];

    Modelo_Continuo(x, u, k1, d);

    for(int i=0;i<4;i++) temp[i] = x[i] + 0.5*Ts*k1[i];
    Modelo_Continuo(temp, u, k2, d);

    for(int i=0;i<4;i++) temp[i] = x[i] + 0.5*Ts*k2[i];
    Modelo_Continuo(temp, u, k3, d);

    for(int i=0;i<4;i++) temp[i] = x[i] + Ts*k3[i];
    Modelo_Continuo(temp, u, k4, d);

    for(int i=0;i<4;i++)
        x_next[i] = x[i] + Ts*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0;
}

float swingUpController2(const float x[4], const DadosSistema &d)
{
    // -------------------------------------------------------
    // Estados
    // -------------------------------------------------------
    float x_pos     = x[0];
    float theta1     = x[1];
    float x_dot1     = x[2];
    float theta_dot1 = x[3];

    // -------------------------------------------------------
    // Parâmetros (igual ao Modelo_Continuo)
    // -------------------------------------------------------
    float m = d.pendulo.m;
    float l = d.pendulo.l;
    float I = d.pendulo.I;
    float b = d.pendulo.b;

    float M = d.carro.m;
    float c = d.carro.c;

    float g = d.geral.g;

    // -------------------------------------------------------
    // Energia do pêndulo
    // -------------------------------------------------------
    float E =
        m * g * l * (1.0f - cos(theta1)) +
        0.5f * (I + m * l * l) * (theta_dot1 * theta_dot1);

    float E_des = 2.0f * m * g * l; 

    float arg = theta_dot1 * cos(theta1);

    float k_energy = 44.0f * g;
 
    float x_2dot_des =
        k_energy * (E - E_des) * sign(arg);
    
    float theta_2dot =
        ( -b * theta_dot1
          - m * l * cos(theta1) * x_2dot_des
          - m * g * l * sin(theta1) )
        / (I + m * l * l);
    float F =
        (M + m) * x_2dot_des
        + m * l * cos(theta1) * theta_2dot
        - m * l * sin(theta1) * (theta_dot1 * theta_dot1)
        + c * x_dot1;

    //Serial.printf("Valores F %.2f, E %.2f, theta2dot %.2f", F, E, theta_2dot);
    return Force2Volt2(F, x_dot1);
}



// ===========================================================
// TASK DE SIMULAÇÃO
// ===========================================================

void simulationTask(void *pvParameters) {

    const float Ts = dados.geral.Ts;
    const float Tf = dados.geral.Tf;
    const int NT = int(Tf / Ts);

    // buffers para enviar ao MATLAB
    float *tempo  = new float[NT];
    float *x1     = new float[NT];
    float *x2     = new float[NT];
    float *x3     = new float[NT];
    float *x4     = new float[NT];
    float *u_vec  = new float[NT];

    // estado inicial
    float x[4] = {
        0,
        1.0 * (PI/180.0),
        0,
        0
    };

    float u;

    for (int k = 0; k < NT; k++) {

        float t = k * Ts;

        if(t == 15.0){
          x[3] = x[3] - 65*PI/180;
        }

        // avança a planta
        float x_next[4];
        RK4_discreto(x, u, Ts, dados, x_next);

        // copia o estado
        memcpy(x, x_next, sizeof(x));

        tempo[k] = t;
        x1[k] = x[0];
        x2[k] = x[1];
        x3[k] = x[2];
        x4[k] = x[3];
        
        bool usaMPC = (abs(abs(x[1])-PI) < 8.0*PI/180.0) && (abs(x[3]) < 90.0*PI/180.0);
        bool emPerigo = (abs(x[0]) > 0.18);


        if (usaMPC){
          float erro = x[1] - PI;
          float estados[4] = {x[0], erro, x[2], x[3]};
          float spt[2] = {5.0f/100.0f, 0.0f};

          u = mpc.compute_MPC_Command(ulast, spt, estados)[0];

        }else if(emPerigo){
          u = -30 * x[0];
        }else{
          u = swingUpController2(x,dados);
        }
        
        u = constrain(u, -12.0, 12.0);

        u_vec[k] = u;
        ulast = u;

        vTaskDelay(1);
    }

    Serial.println("INICIO");

    //envia formato CSV para o MATLAB
    for (int i = 0; i < NT; i++) {
        Serial.print(tempo[i], 6); Serial.print(",");
        Serial.print(x1[i], 6);   Serial.print(",");
        Serial.print(x2[i], 6);   Serial.print(",");
        Serial.print(x3[i], 6);   Serial.print(",");
        Serial.print(x4[i], 6);   Serial.print(",");
        Serial.println(u_vec[i], 6);
    }

    Serial.println("FIM");

    // libera memória
    delete[] tempo;
    delete[] x1; delete[] x2; delete[] x3; delete[] x4;
    delete[] u_vec;

    vTaskDelete(NULL);
}

void setupSimulacao(){
  dados.geral.g   = 9.81;
    dados.geral.Ts  = 0.010;
    dados.geral.Tf  = 30.0;

    dados.pendulo.m = 20.5/1000.0;
    dados.pendulo.l = 0.18;
    dados.pendulo.I = 0.000207;
    dados.pendulo.b = 0.000008;
    dados.pendulo.r_massa = 0.01;

    dados.carro.m = 308.82/1000.0;
    dados.carro.c = 6.0;
    dados.carro.l = 0.1;
    dados.carro.h = 0.05;

    dados.motor.Rm = 10.5;
    dados.motor.Kb = 0.04;
    dados.motor.Kt = 0.175;
    dados.motor.R  = 0.071;

    // cria a task de simulação
    xTaskCreatePinnedToCore(
        simulationTask,
        "SimTask",
        20000,
        NULL,
        1,
        NULL,
        1
    );
}

// ==============================
// CONFIGURAÇÃO INICIAL
// ==============================
void setup() {
  Serial.begin(115200);

  // Inicia Controlador MPC
  setupMPC();

  setupSimulacao();
}

void loop() {
}