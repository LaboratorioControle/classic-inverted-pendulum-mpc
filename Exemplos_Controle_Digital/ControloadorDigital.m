%% Controlador PID Digital

clear all; clc; close all;

%% Período de Amostragem
Ts = 0.1;

%% Modelo Discreto da Planta e Controlador
z = tf('z',Ts);

G = 1/((z-0.1)*(z-0.5));

Kp=0.03;Ki=0.3;Kd=0.0007;N=0;
Gc = pid(Kp,Ki,Kd,N,Ts,'Iformula', 'BackwardEuler');

%% Modelos discreto em malha fechada

Gmf1 = feedback(G,1);
Gmf2 = feedback(G*Gc,1);

%% Solução das Equações Diferença

time = 10/Ts; % Tempo de simulação
r = ones(1,time); % Degrau unitário

for k = 1:time
    switch k
        case 1
            y1(k) = 0; % Saída da planta em Malha Fechada sem Controlador
            y(k) = 0; % Saída da planta em Malha Fechada com Controlador
            e(k) = r(k)-y(k); % Erro
            u(k) = (Kp + Ki * Ts + Kd/Ts)*e(k); % Sinal de saída do controlador
        case 2
            y1(k) = 0;
            y(k) = 0;
            e(k) = r(k)-y(k);
            u(k) = u(k-1) + Kp*e(k) - Kp*e(k-1) + Ki*Ts*e(k) + (Kd/Ts) * (e(k) - 2*e(k-1));
        otherwise
            y1(k) = 0.6*y1(k-1) - 1.05*y1(k-2)+r(k-2);
            y(k) = 0.6*y(k-1) - 0.05*y(k-2)+u(k-2);
            e(k) = r(k)-y(k);
            u(k) = u(k-1) + Kp*e(k) - Kp*e(k-1) + Ki*Ts*e(k) + (Kd/Ts) * (e(k) - 2*e(k-1) + e(k-2));
    end
end

%% Plot dos gráficos
subplot(2,1,1);
step(Gmf1,10); hold on; stairs([0:(time-1)]*Ts,y1,'rx');
title("Sistema Malha Fechada sem controlador");
legend('Solução comando Step', 'Solução Equação Diferença');

subplot(2,1,2);
step(Gmf2,10); hold on; stairs([0:(time-1)]*Ts,y,'rx');
title("Sistema Malha Fechada com controlador");
legend('Solução comando Step', 'Solução Equação Diferença');

% subplot(3,1,3);
% stairs(out.saida_simulink.time,out.saida_simulink.signals.values); hold on; stairs([0:(time-1)]*Ts,y,'rx');
% title("Saída Simulink PID Discreto");
% legend('Saída Simulink', 'Solução Equação Diferença');