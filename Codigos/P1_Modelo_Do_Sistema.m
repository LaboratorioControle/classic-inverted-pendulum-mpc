%% Modelo do sistema do pêndulo invertido

clear; clc; %close all;

%% Declaração das variáveis do sistema

% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade
dados.geral.Ts = 10/1000;       % [s] Período de amostragem
dados.geral.Tf = 20;            % [s] Tempo total da simulação
dados.geral.spt = 0;         % [cm] Posição desejada do carro
dados.geral.angulo_troca = 5;         % [°] Ângulo para troca dos controladores
dados.geral.guia = 0.60; % [m] Tamanho da guia linear

% Condições Iniciais
dados.geral.inicial.theta0 = 5;      % [°] Ângulo inicial
dados.geral.inicial.theta_dot0 = 0;   % [°/s] Velocidade angular inicial
dados.geral.inicial.x0 = 0;           % [m] Posição inicial do carro
dados.geral.inicial.x_dot0 = 0;       % [m/s] Velocidade inicial do carro

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 20.5/1000;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.18;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = 0.000207;  % [kg·m^2] Momento de inércia da haste em relação ao centro
dados.pendulo.b = 0.000008;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
dados.pendulo.r_massa = 0.01; % [m] Raio da massa na ponta do pêndulo 
dados.pendulo.E_des = 2*dados.pendulo.m*dados.geral.g*dados.pendulo.l; % [J] Energia Potencial Desejada do Pêndulo

% ==============================
% Dados do carro
% ==============================
dados.carro.m = 308.82/1000;         % [kg]    Massa do carro
dados.carro.c = 6.0;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia
dados.carro.l = 0.1;           % [m] Largura do carro
dados.carro.h = 0.05;           % [m] Altura do carro

% ==============================
% Dados do motor
% ==============================
dados.motor.Rm = 10.5;         % [Ω]      Resistência do enrolamento (Chute)
dados.motor.Kb = 0.04;         % [V·s/rad] Constante de força contra-eletromotriz (Chute)
dados.motor.Kt = 0.175;         % [N·m/A]  Constante de torque (Torque/Corrente)
dados.motor.R  = 0.071;         % [-]      Relação de transmissão (1/14)

% ==============================
% Atrito Seco
% ==============================
dados.carro.Fc = 0.035;      % N

dados.controlador.energia.k = 44;
dados.controlador.energia.n = 1;

%% Geração das matrizes de espaço de estados

% Estados x1=x, x2=theta, x3=x_ponto, x4=theta_ponto
g = dados.geral.g;
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
b = dados.pendulo.b;
M = dados.carro.m;
c = dados.carro.c;
Rm = dados.motor.Rm;
Kb = dados.motor.Kb;
Kt = dados.motor.Kt;
r = dados.motor.R;
tau = dados.geral.Ts;

alpha = (I + m*l^2)*(M + m) - (m*l)^2;

% Matrizes de estado para utilizar o sinal de comando Tensão [V]
Ac = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (m^2*l^2*g)/alpha, -(I + m*l^2)*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*m*l/alpha;
     0, m*g*l*(M + m)/alpha, -m*l*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*(M + m)/alpha];

Bc = [0;
     0;
    (I + m*l^2)*Kt/(alpha*Rm*r);
     m*l*Kt/(alpha*Rm*r)];

C = [1 0 0 0
     0 1 0 0];

D = 0;

dados.planta.Ac = Ac;
dados.planta.Bc = Bc;
dados.planta.C = C;
dados.planta.D = D;

[A, B] = c2d(Ac,Bc,tau);

dados.planta.A = A;
dados.planta.B = B;

clear g m l I b M c Rm Kb Kt r alpha A B C D Ac Bc tau;

%% Simulação do Modelo - Degrau

% % % Parâmetros
t_final = dados.geral.Tf;         
tau = dados.geral.Ts;
amplitude_degrau = 200*12/255; % Volt
duracao_degrau = 200/1000; % s

x0 = [0 0 0 0];
x(1,:) = x0;
% 
% % Modelo Discreto
% t = (0:tau:t_final)';
% u = zeros(size(t));
% u(t <= duracao_degrau) = amplitude_degrau;
% 
% 
% for k=1:length(t)-1
%     x(k+1,:) = RK4_discrete(x(k,:),u(k),tau,dados);
% end
% 
% x(:, 1) = x(:, 1).*100;
% x(:, 2) = x(:, 2).*((180)/pi);
% x(:, 3) = x(:, 3).*100;
% x(:, 4) = x(:, 4).*((180)/pi);
% 
% % Modelo Contínuo
tspan = [0 t_final];

[t2,x2] = ode45(@(t2,x2) Modelo_Continuo_Script_Degrau(t2, x2, dados, duracao_degrau, amplitude_degrau), tspan, x0);

x2(:, 1) = x2(:, 1).*100;
x2(:, 2) = x2(:, 2).*((180)/pi);
x2(:, 3) = x2(:, 3).*100;
x2(:, 4) = x2(:, 4).*((180)/pi);
% 
% clear amplitude_degrau duracao_degrau k t_final tau tspan x0;

%% Simulação do Modelo - Senoide

% % Parâmetros
% t_final = 60;         
% tau = dados.geral.Ts;
% amplitude = 200*12/255; % Volt
% duracao = 5000/1000; % s
% frequencia = 2; % Hz
% 
% x0 = [0 0 0 0];
% x(1,:) = x0;
% 
% Modelo Discreto
% t = (0:tau:t_final)';
% u = zeros(size(t));
% u(t <= duracao_degrau) = amplitude_degrau;
% 
% 
% for k=1:length(t)-1
%     x(k+1,:) = RK4_discrete(x(k,:),0,tau,dados);
% end
% 
% % %Modelo Contínuo
% % tspan = [0 t_final];
% % 
% % [t2,x2] = ode45(@(t2,x2) Modelo_Continuo_Script_Senoide(t2, x2, dados, duracao, amplitude, frequencia), tspan, x0);
% % 
% % x2(:, 1) = x2(:, 1).*100;
% % x2(:, 2) = x2(:, 2).*((180)/pi);
% % x2(:, 3) = x2(:, 3).*100;
% % x2(:, 4) = x2(:, 4).*((180)/pi);
% 
% clear amplitude_degrau duracao_degrau k t_final tau tspan x0;

%% Plot dos Resultados (Contínuo x Discreto)

% figure; % Cria uma nova janela de figura
% 
% % 1. Posição do Carrinho (x)
% subplot(2, 2, 1);
% stairs(t, x(:, 1), 'b-');          
% hold on;                                  
% plot(t2, x2(:, 1), 'r--');
% hold off;                                
% title('Posição do Carrinho');
% xlabel('Tempo (s)');
% ylabel('Posição (cm)');
% legend('Discreto (RK4)', 'Contínuo (ode45)');
% grid on;
% 
% % 2. Ângulo do Pêndulo (theta)
% subplot(2, 2, 2);
% stairs(t, x(:, 2), 'b-');
% hold on;
% plot(t2, x2(:, 2), 'r--');
% hold off;
% title('Ângulo do Pêndulo');
% xlabel('Tempo (s)');
% ylabel('Ângulo (°)');
% legend('Discreto (RK4)', 'Contínuo (ode45)');
% grid on;
% 
% % 3. Velocidade do Carrinho (x_dot)
% subplot(2, 2, 3);
% stairs(t, x(:, 3), 'b-');
% hold on;
% plot(t2, x2(:, 3), 'r--');
% hold off;
% title('Velocidade do Carrinho');
% xlabel('Tempo (s)');
% ylabel('Velocidade (m/s)');
% legend('Discreto (RK4)', 'Contínuo (ode45)');
% grid on;
% 
% % 4. Velocidade Angular do Pêndulo (theta_dot)
% subplot(2, 2, 4);
% stairs(t, x(:, 4), 'b-');
% hold on;
% plot(t2, x2(:, 4), 'r--');
% hold off;
% title('Velocidade Angular do Pêndulo');
% xlabel('Tempo (s)');
% ylabel('Velocidade Ang. (rad/s)');
% legend('Discreto (RK4)', 'Contínuo (ode45)');
% grid on;

% % clear t t2 x x2 u;

%% Plot dos Resultados Experimentais x Simulados

figure;
importados = importdata('Ensaios\dados_degrau_I200_D200_191254.csv');
t_import = importados(:,1)-importados(1,1);
x_import = importados(:,2:5);

posicao_import = x_import(:, 3).*100;
angulo_import = x_import(:, 1);
velocidade_import = x_import(:, 4).*100;
vel_angular_import = x_import(:, 2);

% 1. Posição do Carrinho
subplot(2, 2, 1);
stairs(t_import, posicao_import); 
hold on;                                  
plot(t2, x2(:, 1)); 
title('Posição do Carrinho');
xlabel('Tempo (s)');
ylabel('Posição (cm)');
%legend('Dados Experimentais','Dados Simulados');
grid on;

% 2. Ângulo do Pêndulo
subplot(2, 2, 2);
stairs(t_import, angulo_import);
hold on;
plot(t2, x2(:, 2));
title('Ângulo do Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
%legend('Dados Experimentais','Dados Simulados');
grid on;

% 3. Velocidade do Carrinho
subplot(2, 2, 3);
stairs(t_import, velocidade_import);
hold on;
plot(t2, x2(:, 3));
title('Velocidade do Carrinho');
xlabel('Tempo (s)');
ylabel('Velocidade (m/s)');
%legend('Dados Experimentais','Dados Simulados');
grid on;

% 4. Velocidade Angular do Pêndulo
subplot(2, 2, 4);
stairs(t_import, vel_angular_import);
hold on;
plot(t2, x2(:, 4));
title('Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Velocidade Ang. (°/s)');
%legend('Dados Experimentais','Dados Simulados');
grid on;

% %clear angulo_import fat_conv_ang fat_conv_pos importados posicao_import t_import vel_angular_import velocidade_import x_import;