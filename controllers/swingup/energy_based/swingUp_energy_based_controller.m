function u_volt = swingUp_energy_based_controller(x, dados)
%ENERGYSWINGUPCONTROLLER Controlador de Swing-Up baseado em energia
%
%   u_volt = ENERGYSWINGUPCONTROLLER(x, dados)
%
%   Esta função implementa um controlador de Swing-Up por energia
%   para o pêndulo invertido. A lei de controle busca injetar ou
%   dissipar energia no sistema de forma a conduzir o pêndulo da
%   posição inferior até a vizinhança da posição de equilíbrio
%   instável (posição invertida).
%
%   Entradas:
%       x     - Vetor de estados do sistema
%               [x; theta; x_dot; theta_dot]
%       dados - Estrutura contendo os parâmetros do sistema e do controlador:
%               .pendulo.m          - Massa do pêndulo [kg]
%               .pendulo.l          - Comprimento do pêndulo [m]
%               .pendulo.I          - Momento de inércia [kg·m²]
%               .geral.g            - Aceleração da gravidade [m/s²]
%               .controlador.energia.k - Ganho do controlador de energia
%               .motor              - Estrutura com parâmetros do motor
%
%   Saída:
%       u_volt - Tensão de controle aplicada ao motor [V]

%% Extração dos estados

theta     = x(2);    % Ângulo do pêndulo [rad]
x_dot     = x(3);    % Velocidade linear do carro [m/s]
theta_dot = x(4);    % Velocidade angular do pêndulo [rad/s]

%% Parâmetros físicos do sistema

m = dados.pendulo.m;    % Massa do pêndulo
l = dados.pendulo.l;    % Comprimento do pêndulo
I = dados.pendulo.I;    % Momento de inércia do pêndulo
g = dados.geral.g;      % Aceleração da gravidade

%% ------------------------------------------------------------------------
%% Ganhos do controlador de energia

ke = dados.controlador.energia.k;

%% Cálculo da energia do sistema

% Energia total do pêndulo (potencial + cinética)
E = m * g * l * (1 - cos(theta)) ...
    + 0.5 * (I + m * l^2) * theta_dot^2;

% Energia desejada para a posição invertida
E_des = 2 * m * g * l;

% Erro de energia
diffE = E - E_des;

%% Definição do sentido de injeção de energia

% Termo auxiliar para definir o sentido da aceleração
arg = theta_dot * cos(theta);

% Sinal do termo auxiliar
% Caso arg = 0, o sinal retornado é zero, o que pode
% impedir a injeção inicial de energia.
sign_arg = sign(arg);

%% Aceleração linear desejada do carro

x_ddot_des = ke * g * diffE * sign_arg - 8 * x(1);

%% Conversão da aceleração desejada em tensão de controle

% Cálculo da força necessária para o Swing-Up
u_force = swingUp_compute_force(x, x_ddot_des, dados);

% Conversão da força em tensão aplicada ao motor
u_volt = Force2Volt(u_force, x_dot, dados.motor);

end
