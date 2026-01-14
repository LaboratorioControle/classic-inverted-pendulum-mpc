%% Projeto LQR Discreto

clear; clc; close all;

% Parâmetros do pêndulo Invertido
m1 = 2; m2 = 0.1; l = 0.3; g = 9.8;

% Definição das matrizes
A = [0 1 0 0
    (m1 + m2)*g/(m1*l) 0 0 0
     0 0 0 1
    -m2*g/m1 0 0 0];

B = [0; -1/(m1*l); 0; 1/m1];

% Matrizes de estado
T = 0.1;
[Ad,Bd] = c2d(A,B,T);

% Cálculo da matriz de ganho

Qd = diag([1 1 1 10000]);Rd = 1;
Kd = dlqr(Ad,Bd,Qd,Rd);

% Simulação

t = 100;
tempo = 0:T:t;
angulo_inicial = 20;

x(1,:) = [angulo_inicial*pi/180;0;0;0];

for k=1:t/T
    u(k) = -Kd*x(k,:)';
    x(k+1,:) = Ad*x(k,:)' + Bd*u(k);
end

angulo = (180/pi) * x(:,1);
velocidade_pendulo = x(:,2);
posicao = x(:,3);
velocidade_carro = x(:,4);
comando = u;

% Plot

subplot(2,2,1);
stairs(tempo, angulo,'LineWidth',2); grid on;
title('Posição Angular do Pêndulo [°]');

subplot(2,2,2);
stairs(tempo, velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular do Pêndulo [rad/s]');

subplot(2,2,3);
stairs(tempo, posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [m]');

subplot(2,2,4);
stairs(tempo, velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [m/s]');

figure;

stairs(tempo(1:length(tempo)-1),comando,'LineWidth',2); grid on;
title('Comando [N]');

%% 

clear; clc; close all;

m1=2; m2=0.1; l=0.3; g=9.8;

A = [     0            1 0 0;
    ((m1+m2)*g)/(m1*l) 0 0 0;
          0            0 0 1
      -(m2/m1)*g       0 0 0];

B = [    0;
    -1/(m1*l);
         0;
        1/m1];

T = 0.1;
[Ad,Bd] = c2d(A,B,T);

for i=1:3
    switch i
        case 1
            % Ponderação na Posição Angular
            Qd = diag([10000 1 1 1]); Rd = 1;
            Kd(:,i) = dlqr(Ad,Bd,Qd,Rd);

        case 2
            % Ponderação na Velocidade Angular
            Qd = diag([1 10000 1 1]); Rd = 1;
            Kd(:,i) = dlqr(Ad,Bd,Qd,Rd);

        case 3
            % Ponderação no Controle - Esforço de Controle
            Qd = diag([1 1 1 1]); Rd = 10000;
            Kd(:,i) = dlqr(Ad,Bd,Qd,Rd);
    end
end

t = 10;
tempo = [0:T:t];
x(1,:) = [20*pi/180;0;0;0];

for m = 1:3
    for k=1:10/T
        u(k) = -Kd(:,m)'*x(k,:)';
        x(k+1,:) = Ad*x(k,:)' + Bd*u(k);
    end

    angulo(:,m) = (180/pi)*x(:,1);
    velocidade(:,m) = x(:,2);
    comando(:,m) = u';
end


subplot(2,2,1)
stairs(tempo,angulo,'LineWidth',2); grid on;
title('Posição Angular Pêndulo [º]');
legend('Pond. Posição','Pond. Velocidade','Pond. Comando')

subplot(2,2,2)
stairs(tempo,velocidade,'LineWidth',2); grid on;
title('Velocidade Angular Pêndulo [rad/s]');
legend('Pond. Posição','Pond. Velocidade','Pond. Comando')

subplot(2,2,[3,4])
stairs(tempo(1:length(tempo)-1),comando,'LineWidth',2); grid on;
title('Comando [N]');
legend('Pond. Posição','Pond. Velocidade','Pond. Comando')