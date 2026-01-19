%% Controlador Preditivo Com Restrições

%% Modelo do sistema do pêndulo invertido

clear; clc; close all;

%% Declaração das variáveis do sistema

% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade
dados.geral.Ts = 10/1000;       % [s] Período de amostragem
dados.geral.Tf = 10;            % [s] Tempo total da simulação
dados.geral.spt = 0;         % [cm] Posição desejada do carro
dados.geral.angulo_troca = 30;         % [°] Ângulo para troca dos controladores
dados.geral.guia = 0.70; % [m] Tamanho da guia linear

% Condições Iniciais
dados.geral.inicial.theta0 = 10;      % [°] Ângulo inicial
dados.geral.inicial.theta_dot0 = 0;   % [°/s] Velocidade angular inicial
dados.geral.inicial.x0 = 0;           % [m] Posição inicial do carro
dados.geral.inicial.x_dot0 = 0;       % [m/s] Velocidade inicial do carro

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 10.5/1000;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.18;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = (1/12)*dados.pendulo.m*(dados.pendulo.l*2)^2;  % [kg·m^2] Momento de inércia da haste em relação ao centro
dados.pendulo.b = 0.00005;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
dados.pendulo.r_massa = 0.01; % [m] Raio da massa na ponta do pêndulo 
dados.pendulo.E_des = 2*dados.pendulo.m*dados.geral.g*dados.pendulo.l; % [J] Energia Potencial Desejada do Pêndulo

% ==============================
% Dados do carro
% ==============================
dados.carro.m = 208.82/1000;         % [kg]    Massa do carro
dados.carro.c = 2;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia
dados.carro.l = 0.1;           % [m] Largura do carro
dados.carro.h = 0.05;           % [m] Altura do carro

% ==============================
% Dados do motor
% ==============================
dados.motor.Rm = 12.5;         % [Ω]      Resistência do enrolamento (Chute)
dados.motor.Kb = 0.031;         % [V·s/rad] Constante de força contra-eletromotriz (Chute)
dados.motor.Kt = 0.17;         % [N·m/A]  Constante de torque (Torque/Corrente)
dados.motor.R  = 0.071;         % [-]      Relação de transmissão (1/14)

%% Geração das matrizes de espaço de estados

m0 = dados.carro.m;
m1 = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
g = dados.geral.g;
tau = dados.geral.Ts;


alpha1 = (m1*l)/(m1*l^2+I); beta1 = g*alpha1;
alpha0 = m0+m1-alpha1*m1*l; beta0 = m1*l*beta1;

% x = [theta theta_dot r r_dot]

Ac = [0 1 0 0; ((alpha1*beta0)/alpha0)+beta1 0 0 0; 0 0 0 1
                  -beta0/alpha0 0 0 0];

Bc = [0; -alpha1/alpha0; 0; 1/alpha0];

[A, B] = c2d(Ac,Bc,tau);

C = [1 0 0 0
     0 0 1 0];

dados.planta.Ac = Ac; dados.planta.Bc = Bc; 
dados.planta.A = A; dados.planta.B = B; dados.planta.C = C;

clear alpha0 alpha1 beta0 beta1;
clear A Ac B Bc C g l I m0 m1 tau;

close all;

%% Variáveis gerais

A = dados.planta.A;
B = dados.planta.B; 
tau = dados.geral.Ts; 

%pos_limite = dados.geral.guia/2*0.8;
pos_limite = inf;
ang_limite = inf*(pi/180);
comando_limite = inf;

ang_inicial = 5*(pi/180);
pos_inicial = 0/100;

pos_spt = 0/100;

tsim=30;

%% Variáveis do MPC

[n,nu] = size(B); 

MPC.A = A; 
MPC.B = B;

% Definição das variáveis rastreadas
MPC.Cr = [1 0 0 0; 0 1 0 0]; 

MPC.Qy = [50 0; 0 100];
MPC.Qu=0.001; 
MPC.N = 35; 

% Definição das variáveis restringidas
MPC.Cc = [1 0 0 0; 0 1 0 0];

% Definição das restrições
MPC.ycmin = [-pos_limite; -ang_limite]; MPC.ycmax= [pos_limite; ang_limite]; 
MPC.umin = -comando_limite; MPC.umax = comando_limite; 
MPC.ulast = 0; MPC.deltamin=-inf; MPC.deltamax=inf;

% Cálculo das matrizes do MPC
[MPC]=compute_MPC_Matrices(MPC);

% Condições Iniciais
x0=[pos_inicial;ang_inicial;0;0];

% Inicialização do lest e captura do tamanho dos vetores 
lest=(0:tau:tsim)';
nt=size(lest, 1);
num_var_reguladas = size(MPC.Cr,1);

% Inicialização dos vetores
lesx=zeros(nt,length(B)); 
lesy=zeros(nt, num_var_reguladas); 
lesu=zeros(nt, 1);

% Sinal de referência
yref = zeros(length(lest)*num_var_reguladas,1);         
yref(1:2:end) = pos_spt;  

lesx(1,:)=x0';
lesy(1,:) = MPC.Cr*lesx(1, :)'; 
for i=1:nt-1
    yref_pred=yref(i+1:i+MPC.N*num_var_reguladas);

    F=MPC.F1*lesx(i,:)'+MPC.F2*yref_pred;
    Bineq=MPC.G1*lesx(i, :)'+MPC.G2*MPC.ulast+MPC.G3; 

    options=optimset('maxIter', 100000); 
    utilde_opt = quadprog(MPC.H, F, MPC.Aineq, Bineq, ...
    [], [], MPC.utildemin, MPC.utildemax, [], options);

    u=P_i(1, nu, MPC.N)*utilde_opt;
    lesu (i, :)=u'; 
    MPC.ulast=u;
    xplus=MPC.A*lesx(i, :)'+MPC.B*u;
    lesx(i+1, :)=xplus';
    lesy(i+1, :)=MPC.Cr*xplus;

end

%% Plot dos resultados

subplot(2,2,1);
stairs(lest,lesu); grid on;
title('Comando');

pos_simulado = lesy(:,1).*100;

subplot(2,2,2);
plot(lest,pos_simulado,lest,yref(1:2:end).*100); grid on;
title('Posição (cm)');

ang_simulado = lesy(:,2).*(180/pi);

subplot(2,2,3);
plot(lest,ang_simulado,lest,yref(2:2:end)); grid on;
title('Ângulo (°)');

% ang_simulado_desvio = lesy(:,2).*(180/pi); % Desvio em graus (Delta_theta)
% ang_simulado_abs = ang_simulado_desvio + 180; % Angulo Absoluto (theta_abs)
% 
% yref_ang_abs = ones(nt, 1) * 180;
% 
% subplot(2,2,3);
% plot(lest, ang_simulado_abs, lest, yref_ang_abs); 
% grid on;
% title('Ângulo (°)');
% legend('Ângulo Simulado (Absoluto)', 'Referência (180°)', 'Location', 'best');

