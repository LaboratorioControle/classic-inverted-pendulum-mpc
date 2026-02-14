%% Declaração das variáveis do pêndulo invertido

% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 20.4/1000;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.18;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = 0.0003;  % [kg·m^2] Momento de inércia da haste em relação ao centro
dados.pendulo.b = 0;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
dados.geral.guia = 0.60; % [m] Tamanho da guia linear


% ==============================
% Dados do carro
% ==============================
dados.carro.m = 74.5/1000;         % [kg]    Massa do carro
dados.carro.c = 11.2498;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia


% ==============================
% Dados do motor
% ==============================
dados.motor.Rm = 6.8845;         % [Ω]      Resistência do enrolamento (Chute)
dados.motor.Kb = 0.0177;         % [V·s/rad] Constante de força contra-eletromotriz (Chute)
dados.motor.Kt = 0.1748;         % [N·m/A]  Constante de torque (Torque/Corrente)
dados.motor.R  = 0.071;         % [-]      Relação de transmissão (1/14)


% ==============================
% Atrito Seco
% ==============================
dados.carro.Fc = 0.035;      % N


%% Geração das matrizes de estado

run params_ss_pendulum;