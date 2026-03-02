%% Controlador Híbrido: Swing-Up por Energia + LQR
% Este script implementa um controlador híbrido para o pêndulo invertido.
% Inicialmente, o pêndulo é conduzido à posição vertical utilizando um
% controlador baseado em energia (Swing-Up). Quando o sistema atinge a
% vizinhança da posição de equilíbrio instável, ocorre o chaveamento para
% um controlador LQR discreto para estabilização.

%close all;
%clear;
run init_project;
clc;

%% Definição do controlador LQR discreto

% Matrizes de ponderação do LQR
dados.controlador.lqr.Q = diag([15 5 0 0]); 
dados.controlador.lqr.R = 0.001;

% Cálculo do ganho de realimentação de estados
dados.controlador.lqr.K = dlqr( ...
    dados.planta.A, ...
    dados.planta.B, ...
    dados.controlador.lqr.Q, ...
    dados.controlador.lqr.R);

%% Definição do controlador de Swing-Up baseado em energia

dados.controlador.energia.k = 5;   % Ganho do controlador de energia
dados.controlador.energia.n = 1;    % (Parâmetro reservado / extensão futura)

%% Parâmetros físicos do sistema

m = dados.pendulo.m;   % Massa do pêndulo
l = dados.pendulo.l;   % Comprimento do pêndulo
I = dados.pendulo.I;   % Momento de inércia do pêndulo
g = dados.geral.g;     % Aceleração da gravidade

%% Parâmetros de simulação

Ts = dados.geral.Ts;   % Período de amostragem
Tf = 30;   % Tempo final de simulação

k_lqr = dados.controlador.lqr.K;

% Limiares para chaveamento Swing-Up → LQR
theta_switch     = 15  * pi/180;   % Limite angular [rad]
theta_dot_switch = 100 * pi/180;   % Limite de velocidade angular [rad/s]

%% Pré-alocação de memória

N = round(Tf / Ts);

t        = zeros(1, N+1);   % Tempo
u_volt   = zeros(1, N+1);   % Tensão de controle
u_force  = zeros(1, N+1);   % Força aplicada ao carro
x        = zeros(N+1, 4);   % Estados do sistema
E        = zeros(1, N+1);   % Energia do pêndulo

%% Condições iniciais

x(1,:)      = [0 180*pi/180 0 0];   % [posição; ângulo; vel. carro; vel. pêndulo]
u_volt(1)   = 0;
u_force(1)  = 0;
t(1)        = 0;
E(1)        = 0;

% Estado desejado (posição de equilíbrio instável)
x_des = [0; 180*pi/180; 0; 0];

%% Simulação em malha fechada

i = 1;

for k = 0:Ts:Tf
    
    % Atualização do tempo
    t(i+1) = k + Ts;

    %if i == 1
    %    estado_novo = RK4_discrete(x(i,:), -200*12/255, Ts, dados);
    
    %else
        % Integração do modelo não linear (RK4)
        estado_novo = RK4_discrete(x(i,:), u_volt(i), Ts, dados)';
    %end

    

    % Aplicação de um distúrbio impulsivo aos 20 segundos
    if t(i+1) == 10.0
        estado_novo(4) = estado_novo(4) - 45*pi/180;
    end

    x(i+1,:) = estado_novo';

    % Extração dos estados relevantes
    theta     = estado_novo(2);
    theta_dot = estado_novo(4);

    %% Chaveamento entre Swing-Up e LQR
    
    if abs(estado_novo(1)) >= 0.23
        u_volt(i+1) = -15 * estado_novo(1) - 80*estado_novo(3);
    elseif (abs(theta - pi) < theta_switch) && ...
       (abs(theta_dot) < theta_dot_switch)

        % Região de estabilização → LQR
        u_volt(i+1) = -k_lqr * (estado_novo - x_des);

    else
        % Região fora do equilíbrio → Swing-Up por energia
        u_volt(i+1) = swingUp_energy_based_controller(estado_novo, dados);
    end

    % Saturação do atuador
    u_volt(i+1) = sat(u_volt(i+1), 12, -12);

    % Conversão tensão → força
    u_force(i+1) = Volt2Force(u_volt(i+1), estado_novo(3), dados.motor);

    % Cálculo da energia do pêndulo
    E(i+1) = m * g * l * (1 - cos(theta)) ...
           + 0.5 * (I + m * l^2) * theta_dot^2;

    i = i + 1;
end

%% Organização dos resultados da simulação

simulacao.lqr.tempo              = t;
simulacao.lqr.angulo             = (180/pi) * x(:,2);
simulacao.lqr.velocidade_pendulo = (180/pi) * x(:,4);
simulacao.lqr.posicao            = 100 * x(:,1);
simulacao.lqr.velocidade_carro   = 100 * x(:,3);
simulacao.lqr.u_force            = u_force;
simulacao.lqr.u_volt             = u_volt;
simulacao.lqr.energia            = E;

%% Plot dos resultados

importados = importdata('data\raw\dados_swingUp_16022026.csv');

off_set = 102;

t_import = importados(:,1) - importados(off_set,1);
t_import = t_import/1000;
t_import = t_import(off_set:end);

angulo_import      = importados(off_set:end,2);
vel_angular_import = importados(off_set:end,3);
posicao_import     = importados(off_set:end,4);
velocidade_import  = importados(off_set:end,5);
u_import           = importados(off_set:end,6);


figure;
subplot(2,1,1);
hold on;
stairs(simulacao.lqr.tempo, simulacao.lqr.angulo, 'LineWidth', 2);
%stairs(t_import, angulo_import);
grid on;
title('Ângulo do Pêndulo [°]');
xlabel('Tempo [s]');

subplot(2,1,2);
hold on;
stairs(simulacao.lqr.tempo, simulacao.lqr.velocidade_pendulo, 'LineWidth', 2);
%stairs(t_import, vel_angular_import);
grid on;
title('Velocidade Angular do Pêndulo [°/s]');
xlabel('Tempo [s]');

figure;
subplot(2,1,1);
hold on;
stairs(simulacao.lqr.tempo, simulacao.lqr.posicao, 'LineWidth', 2);
%stairs(t_import, posicao_import);
grid on;
title('Posição Linear do Carro [cm]');
xlabel('Tempo [s]');

subplot(2,1,2);
hold on;
stairs(simulacao.lqr.tempo, simulacao.lqr.velocidade_carro, 'LineWidth', 2);
%stairs(t_import, velocidade_import);
grid on;
title('Velocidade Linear do Carro [cm/s]');
xlabel('Tempo [s]');

figure;
subplot(1,2,1);
stairs(simulacao.lqr.tempo, simulacao.lqr.u_force, 'LineWidth', 2);
grid on;
title('Comando de Controle – Força [N]');

subplot(1,2,2);
hold on;
stairs(simulacao.lqr.tempo, simulacao.lqr.u_volt, 'LineWidth', 2);
%stairs(t_import, u_import);
grid on;
title('Comando de Controle – Tensão [V]');

figure;
stairs(simulacao.lqr.tempo, simulacao.lqr.energia, 'LineWidth', 2);
grid on;
title('Energia do Pêndulo');
xlabel('Tempo [s]');



clear estado_novo E g i l k k_lqr I m N t theta theta_dot theta_dot_switch theta_switch Tf Ts;
clear u_force u_volt x x_des;