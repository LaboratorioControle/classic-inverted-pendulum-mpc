%% Controlador LQR Discreto
% Implementação e simulação de um controlador LQR discreto
% aplicado ao pêndulo invertido linearizado em torno da
% posição de equilíbrio instável (180°).

close all;
clc;

%% Definição das matrizes de ponderação do LQR
% Q: penalização dos estados
% R: penalização do esforço de controle

dados.controlador.lqr.Q = diag([10 5 1 1]);
dados.controlador.lqr.R = 0.01;

% Cálculo do ganho de realimentação de estados
dados.controlador.lqr.K = dlqr( ...
    dados.planta.A, ...
    dados.planta.B, ...
    dados.controlador.lqr.Q, ...
    dados.controlador.lqr.R);

%% Parâmetros de simulação

Ts = dados.geral.Ts;   % Período de amostragem [s]
Tf = dados.geral.Tf;   % Tempo final de simulação [s]

% Estado inicial [posição; ângulo; velocidade carro; velocidade pêndulo]
x(1,:) = [0 185*pi/180 0 0];

% Inicialização dos vetores
u_volt(1)  = 0;        % Tensão aplicada ao motor [V]
u_force(1) = 0;        % Força aplicada ao carro [N]
t(1)       = 0;        % Tempo [s]

% Estado desejado (equilíbrio instável)
x_des = [0; 180*pi/180; 0; 0];


%% Simulação em malha fechada

i = 1;

for k = 0:Ts:Tf
    
    % Atualização do tempo
    t(i+1) = k;
    
    % Integração do modelo não linear (RK4)
    novo_estado = RK4_discrete(x(i,:), u_volt(i), Ts, dados)';
    x(i+1,:) = novo_estado';
    
    
    u_volt(i+1) = -dados.controlador.lqr.K * (novo_estado - x_des);
    u_volt(i+1) = sat(u_volt(i+1), 12, -12); % Saturação em tensão
    
    % Conversão de tensão para força
    u_force(i+1) = Volt2Force( ...
        u_volt(i+1), ...
        novo_estado(3), ...
        dados.motor);
    
    i = i + 1;
end

%% Organização dos dados de simulação

simulacao.tempo                = t;
simulacao.angulo               = (180/pi) * x(:,2);
simulacao.velocidade_pendulo   = (180/pi) * x(:,4);
simulacao.posicao              = 100 * x(:,1);
simulacao.velocidade_carro     = 100 * x(:,3);
simulacao.u_force              = u_force;
simulacao.u_volt               = u_volt;

%% Plot dos resultados

figure;

subplot(2,2,1);
stairs(simulacao.tempo, simulacao.angulo, 'LineWidth', 2);
grid on;
title('Posição Angular do Pêndulo [°]');

subplot(2,2,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo, 'LineWidth', 2);
grid on;
title('Velocidade Angular do Pêndulo [°/s]');

subplot(2,2,3);
stairs(simulacao.tempo, simulacao.posicao, 'LineWidth', 2);
grid on;
title('Posição Linear do Carro [cm]');

subplot(2,2,4);
stairs(simulacao.tempo, simulacao.velocidade_carro, 'LineWidth', 2);
grid on;
title('Velocidade Linear do Carro [cm/s]');

figure;

subplot(1,2,1);
stairs(simulacao.tempo, simulacao.u_force, 'LineWidth', 2);
grid on;
title('Comando de Controle - Força [N]');

subplot(1,2,2);
stairs(simulacao.tempo, simulacao.u_volt, 'LineWidth', 2);
grid on;
title('Comando de Controle - Tensão [V]');

clear i k novo_estado simulacao t Tf Ts u_force u_volt x x_des;