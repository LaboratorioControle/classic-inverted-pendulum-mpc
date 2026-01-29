%% Comparação entre Simulação MPC e Dados Experimentais

run controllers\stabilization\mpc\controller_mpc_with_constraints_qpOASES.m;

close all;
clc;

%% Carregar dados experimentais
load("data/processed/Ensaio_MPC_2026-01-29_20-04-47.mat");

tempo_exp     = exportados.tempo;
u_exp         = exportados.u;
x_exp         = exportados.x * 100;                 % posição [cm]
theta_exp     = exportados.theta * 180/pi;          % ângulo [°]
x_dot_exp     = exportados.x_dot * 100;             % velocidade [cm/s]
theta_dot_exp = exportados.theta_dot * 180/pi;      % vel. angular [°/s]

% Cores padrão
corSim = [0 0.45 0.74];   % azul (simulação)
corExp = [0.2 0.7 0.2];   % verde (experimental)

%% POSIÇÃO DO CARRINHO

figure('Name','Posição e Vel. Linear','Color','w')
subplot(2,1,1)
hold on; grid on;

plot(tempo, posicao, 'LineWidth', 2.5, 'Color', corSim);
plot(tempo_exp, x_exp, '--', 'LineWidth', 1.8, 'Color', corExp);

plot(tempo, pos_spt, '--', 'LineWidth', 2.2, 'Color', [0.93 0.69 0.13]);

plot(tempo,  pos_limite*100 * ones(size(tempo)), 'r--', 'LineWidth', 2);
plot(tempo, -pos_limite*100 * ones(size(tempo)), 'r--', 'LineWidth', 2);

title('Posição do Carrinho (cm)');
xlabel('Tempo (s)');
ylabel('Posição (cm)');
legend('Simulação','Experimental','Referência','Limite Sup','Limite Inf','Location','best');

%% VELOCIDADE DO CARRINHO

subplot(2,1,2)
hold on; grid on;

plot(tempo, velocidade, 'LineWidth', 2.5, 'Color', corSim);
plot(tempo_exp, x_dot_exp, '--', 'LineWidth', 1.8, 'Color', corExp);

plot(tempo,  vel_limite*100 * ones(size(tempo)), 'r--', 'LineWidth', 2);
plot(tempo, -vel_limite*100 * ones(size(tempo)), 'r--', 'LineWidth', 2);

title('Velocidade do Carrinho (cm/s)');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');
legend('Simulação','Experimental','Limite Sup','Limite Inf','Location','best');

%% ÂNGULO DO PÊNDULO

figure('Name','Ângulo e Vel. Angular','Color','w')

subplot(2,1,1)
hold on; grid on;

plot(tempo, angulo, 'LineWidth', 2.5, 'Color', corSim);
plot(tempo_exp, theta_exp, '--', 'LineWidth', 1.8, 'Color', corExp);

plot(tempo, 180 + ang_limite*180/pi * ones(size(tempo)), 'r--', 'LineWidth', 2);
plot(tempo, 180 - ang_limite*180/pi * ones(size(tempo)), 'r--', 'LineWidth', 2);

title('Ângulo do Pêndulo (°)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
legend('Simulação','Experimental','Limite Sup','Limite Inf','Location','best');

%% VELOCIDADE ANGULAR DO PÊNDULO

subplot(2,1,2) 
hold on; grid on;

plot(tempo, vel_angular, 'LineWidth', 2.5, 'Color', corSim);
plot(tempo_exp, theta_dot_exp, '--', 'LineWidth', 1.8, 'Color', corExp);

title('Velocidade Angular do Pêndulo (°/s)');
xlabel('Tempo (s)');
ylabel('Velocidade Angular (°/s)');
legend('Simulação','Experimental','Location','best');

%% COMANDO

figure('Name','Sinal de Controle','Color','w')
hold on; grid on;

stairs(tempo, comando, 'LineWidth', 2.5, 'Color', corSim);
stairs(tempo_exp, u_exp, '--', 'LineWidth', 1.8, 'Color', corExp);

plot(tempo,  comando_limite * ones(size(tempo)), 'r--', 'LineWidth', 2);
plot(tempo, -comando_limite * ones(size(tempo)), 'r--', 'LineWidth', 2);

title('Comando (V)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
legend('Simulação','Experimental','Limite Sup','Limite Inf','Location','best');

%% Limpeza

clear ang_limite pos_limite comando_limite vel_limite comando angulo posicao velocidade vel_angular;
clear x_exp x_dot_exp u_exp theta_dot_exp theta_exp tempo tempo_exp pos_spt;
clear corSim corExp exportados;