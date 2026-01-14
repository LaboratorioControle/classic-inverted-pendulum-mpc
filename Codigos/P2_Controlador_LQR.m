%% Controlador LQR Discreto
clc; 

run Codigos\P1_Modelo_Do_Sistema.m;

%%

% Definição das matrizes de ponderamento do LQR
controlador.lqr.Q = diag([10 5 1 1]);controlador.lqr.R = 0.01;
controlador.lqr.K = dlqr(dados.planta.A,dados.planta.B,controlador.lqr.Q,controlador.lqr.R);

%% Simulação em malha fechada

Ts = dados.geral.Ts; % Período de Amostragem
Tf = dados.geral.Tf; % Tempo final da simulação

x(1,:) = [0 185*pi/180 0 0]; % Estado Inicial
u_volt(1) = 0;
u_force(1) = 0;
t(1) = 0;

x_des = [0; 180*pi/180; 0; 0];            % Estado desejado

i = 1;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x));

dead_zone = 1*pi/180;

for k = 0:Ts:Tf
    t(i + 1) = k;

    novo_estado = RK4_discrete(x(i,:),u_volt(i),Ts,dados)';
    x(i+1,:) = novo_estado';

    
    
    %if novo_estado(2) < dead_zone
    %    u_volt(i+1) = 0;
    %else
        u_volt(i+1) = -controlador.lqr.K * (novo_estado - x_des);
        u_volt(i+1) = sat(u_volt(i+1),12,-12);
    %end

    u_force(i+1) = Volt2Force(u_volt(i+1),novo_estado(3),dados.motor);

    i = i+1;
end

simulacao.tempo = t;
simulacao.angulo = (180/pi).* x(:,2);
simulacao.velocidade_pendulo = (180/pi).*x(:,4);
simulacao.posicao = x(:,1).*100;
simulacao.velocidade_carro = x(:,3).*100;
simulacao.u_force = u_force;
simulacao.u_volt = u_volt;

%clear k u_force u_volt i novo_estado t Tf Ts x x_des sat;

%% Plot dos dados

figure;
subplot(2,2,1);
stairs(simulacao.tempo, simulacao.angulo,'LineWidth',2); grid on;
title('Posição Angular do Pêndulo [°]');

subplot(2,2,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular do Pêndulo [°/s]');

subplot(2,2,3);
stairs(simulacao.tempo, simulacao.posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [cm]');

subplot(2,2,4);
stairs(simulacao.tempo, simulacao.velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [cm/s]');

figure;

subplot(1,2,1);
stairs(simulacao.tempo,simulacao.u_force,'LineWidth',2); grid on;
title('Comando - Força [F]');

subplot(1,2,2);
stairs(simulacao.tempo,simulacao.u_volt,'LineWidth',2); grid on;
title('Comando - Tensão [V]');