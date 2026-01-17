function F = swingUp_compute_force(estados, x_2dot_desejado, dados)
%SWINGUP_COMPUTE_FORCE Calcula a força de controle para o Swing-Up por energia
%
%   F = SWINGUP_COMPUTE_FORCE(estados, x_2dot_desejado, dados)
%
%   Esta função calcula a força de controle aplicada ao carro
%   durante a fase de Swing-Up do pêndulo invertido, utilizando
%   uma abordagem baseada em energia. A força é obtida a partir
%   da aceleração linear desejada do carro e da dinâmica acoplada
%   do sistema carro–pêndulo, com o objetivo de conduzir o
%   pêndulo até a vizinhança da posição de equilíbrio instável.
%
%   Entradas:
%       estados           - Vetor de estados do sistema
%                           [x; theta; x_dot; theta_dot]
%       x_2dot_desejado   - Aceleração linear desejada do carro [m/s²]
%       dados             - Estrutura contendo os parâmetros do sistema:
%                           .pendulo.m  - Massa do pêndulo [kg]
%                           .pendulo.l  - Comprimento do pêndulo [m]
%                           .pendulo.I  - Momento de inércia do pêndulo [kg·m²]
%                           .pendulo.b  - Coeficiente de atrito viscoso do pêndulo [N·m·s]
%                           .carro.m    - Massa do carro [kg]
%                           .carro.c    - Coeficiente de atrito viscoso do carro [N·s/m]
%                           .geral.g    - Aceleração da gravidade [m/s²]
%
%   Saída:
%       F - Força de controle aplicada ao carro [N]

%% Extração dos estados do sistema

theta     = estados(2);    % Ângulo do pêndulo [rad]
x_dot     = estados(3);    % Velocidade linear do carro [m/s]
theta_dot = estados(4);    % Velocidade angular do pêndulo [rad/s]

%% Parâmetros físicos do sistema

% Pêndulo
m = dados.pendulo.m;       % Massa do pêndulo
l = dados.pendulo.l;       % Comprimento do pêndulo
I = dados.pendulo.I;       % Momento de inércia
b = dados.pendulo.b;       % Atrito viscoso do pêndulo

% Carro
M = dados.carro.m;         % Massa do carro
c = dados.carro.c;         % Atrito viscoso do carro

% Gravidade
g = dados.geral.g;

%% Cálculo da aceleração angular do pêndulo

theta_2dot = ( ...
    - b * theta_dot ...
    - m * l * cos(theta) * x_2dot_desejado ...
    - m * g * l * sin(theta) ) / (I + m * l^2);

%% Cálculo da força de controle aplicada ao carro

F = (M + m) * x_2dot_desejado ...
    + m * l * cos(theta) * theta_2dot ...
    - m * l * sin(theta) * theta_dot^2 ...
    + c * x_dot;

end
