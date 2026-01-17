%% Controlador Híbrido: Swing-Up por Energia + LQR
% Este script implementa um controlador híbrido para o pêndulo invertido.
% Inicialmente, o pêndulo é conduzido à posição vertical utilizando um
% controlador baseado em energia (Swing-Up). Quando o sistema atinge a
% vizinhança da posição de equilíbrio instável, ocorre o chaveamento para
% um controlador LQR discreto para estabilização.

close all;
clc;

%% Definição do controlador LQR discreto

% Matrizes de ponderação do LQR
dados.controlador.lqr.Q = diag([1 5 1 1]); 
dados.controlador.lqr.R = 0.001;

% Cálculo do ganho de realimentação de estados
dados.controlador.lqr.K = dlqr( ...
    dados.planta.A, ...
    dados.planta.B, ...
    dados.controlador.lqr.Q, ...
    dados.controlador.lqr.R);

%% Definição do controlador de Swing-Up baseado em energia

dados.controlador.energia.k = 44;   % Ganho do controlador de energia
dados.controlador.energia.n = 1;    % (Parâmetro reservado / extensão futura)

%% Parâmetros físicos do sistema

m = dados.pendulo.m;   % Massa do pêndulo
l = dados.pendulo.l;   % Comprimento do pêndulo
I = dados.pendulo.I;   % Momento de inércia do pêndulo
g = dados.geral.g;     % Aceleração da gravidade

%% Parâmetros de simulação

Ts = dados.geral.Ts;   % Período de amostragem
Tf = dados.geral.Tf;   % Tempo final de simulação

k_lqr = dados.controlador.lqr.K;

% Limiares para chaveamento Swing-Up → LQR
theta_switch     = 6  * pi/180;   % Limite angular [rad]
theta_dot_switch = 20 * pi/180;   % Limite de velocidade angular [rad/s]

%% Pré-alocação de memória

N = round(Tf / Ts);

t        = zeros(1, N+1);   % Tempo
u_volt   = zeros(1, N+1);   % Tensão de controle
u_force  = zeros(1, N+1);   % Força aplicada ao carro
x        = zeros(N+1, 4);   % Estados do sistema
E        = zeros(1, N+1);   % Energia do pêndulo

%% Condições iniciais

x(1,:)      = [0 1*pi/180 0 0];   % [posição; ângulo; vel. carro; vel. pêndulo]
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

    % Integração do modelo não linear (RK4)
    estado_novo = RK4_discrete(x(i,:), u_volt(i), Ts, dados)';

    % Aplicação de um distúrbio impulsivo aos 20 segundos
    if abs(k - 20) < Ts/2
        estado_novo(4) = estado_novo(4) + 20*pi/180;
    end

    x(i+1,:) = estado_novo';

    % Extração dos estados relevantes
    theta     = estado_novo(2);
    theta_dot = estado_novo(4);

    %% Chaveamento entre Swing-Up e LQR

    if (abs(theta - pi) < theta_switch) && ...
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

simulacao.tempo              = t;
simulacao.angulo             = wrapTo360((180/pi) * x(:,2));
simulacao.velocidade_pendulo = (180/pi) * x(:,4);
simulacao.posicao            = 100 * x(:,1);
simulacao.velocidade_carro   = 100 * x(:,3);
simulacao.u_force            = u_force;
simulacao.u_volt             = u_volt;
simulacao.energia            = E;

%% Plot dos resultados

figure;
subplot(2,1,1);
stairs(simulacao.tempo, simulacao.angulo, 'LineWidth', 2);
grid on;
title('Ângulo do Pêndulo [°]');
xlabel('Tempo [s]');

subplot(2,1,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo, 'LineWidth', 2);
grid on;
title('Velocidade Angular do Pêndulo [°/s]');
xlabel('Tempo [s]');

figure;
subplot(2,1,1);
stairs(simulacao.tempo, simulacao.posicao, 'LineWidth', 2);
grid on;
title('Posição Linear do Carro [cm]');
xlabel('Tempo [s]');

subplot(2,1,2);
stairs(simulacao.tempo, simulacao.velocidade_carro, 'LineWidth', 2);
grid on;
title('Velocidade Linear do Carro [cm/s]');
xlabel('Tempo [s]');

figure;
subplot(1,2,1);
stairs(simulacao.tempo, simulacao.u_force, 'LineWidth', 2);
grid on;
title('Comando de Controle – Força [N]');

subplot(1,2,2);
stairs(simulacao.tempo, simulacao.u_volt, 'LineWidth', 2);
grid on;
title('Comando de Controle – Tensão [V]');

figure;
stairs(simulacao.tempo, simulacao.energia, 'LineWidth', 2);
grid on;
title('Energia do Pêndulo');
xlabel('Tempo [s]');

clear estado_novo E g i l k k_lqr I m N simulacao t theta theta_dot theta_dot_switch theta_switch Tf Ts;
clear u_force u_volt x x_des;