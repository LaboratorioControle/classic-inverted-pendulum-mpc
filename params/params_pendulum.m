%% Declaração das variáveis do pêndulo invertido

% ==============================
% Dados gerais
% ==============================
dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade

% ==============================
% Dados do pêndulo
% ==============================
dados.pendulo.m = 20.5/1000;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 0.18;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = 0.000207;  % [kg·m^2] Momento de inércia da haste em relação ao centro
dados.pendulo.b = 0.000008;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
dados.geral.guia = 0.60; % [m] Tamanho da guia linear


% ==============================
% Dados do carro
% ==============================
dados.carro.m = 308.82/1000;         % [kg]    Massa do carro
dados.carro.c = 6.0;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia


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