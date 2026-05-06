%% CARREGAR DADOS DO CSV
data = readtable('data/raw/Exponencial/dados_esp32_20260331_Degrau_20cm_01_v2.csv');

%data = readtable('data/raw/Exponencial/dados_esp32_20260407_Disturbio_SinalComando_18cm.csv');

pos_limite = 0.20;
xlimite = 30;

% Extrair variáveis
t = data.t_ms / 1000;  % converter ms -> s
t = t - t(1);          % tempo relativo começando em 0

posicao = data.x_cm;
angulo = data.theta_deg;
velocidade = data.x_dot_cm;
vel_angular = data.theta_dot;

u_exp = data.u;
yref_exp = data.yref;
%yref = zeros(1,length(data.yref));

tempo_comp = data.tempo_computacional;
cod_erro = data.cod_erro;

%% Offset de dados

t_ini = 7.8;    % tempo inicial (s) 7.8; 8;6

idx = (t >= t_ini);

% Aplicar filtro em TODAS as variáveis

t = t(idx);
t = t - t(1);
posicao = posicao(idx);
angulo = angulo(idx);
velocidade = velocidade(idx);
vel_angular = vel_angular(idx);
u_exp = u_exp(idx);
yref_exp = yref_exp(idx);
tempo_comp = tempo_comp(idx);
cod_erro = cod_erro(idx);

angulo = unwrap(deg2rad(angulo));
angulo = rad2deg(angulo);
angulo = angulo/360;

%% PLOT PRINCIPAL
figure;

% ===================== POSIÇÃO =====================
subplot(2,2,1); hold on; grid on;
title('Posição do Carro vs Referência (cm)');
xlabel('Tempo (s)');

stairs(t, posicao, 'LineWidth', 1.5);
hold on;
stairs(t, yref_exp, '--', 'LineWidth', 1.1);
yline(pos_limite*100, '--', 'LineWidth', 1.1)
yline(-pos_limite*100, '--', 'LineWidth', 1.1)



legend('Posição medida', 'Referência','Limite');

ylim([-22, 22]);
xlim([0, xlimite]);


% ===================== ÂNGULO =====================
subplot(2,2,2); hold on; grid on;
title('Ângulo (°)');
xlabel('Tempo (s)');

stairs(t, angulo, 'LineWidth', 1.5);


ylim([-5, 365]);
xlim([0, xlimite]);

% ===================== VELOCIDADE =====================
subplot(2,2,3); hold on; grid on;
title('Velocidade do Carro (cm/s)');
xlabel('Tempo (s)');

stairs(t, velocidade, 'LineWidth', 1.5);


ylim([-50, 50]);
xlim([0, xlimite]);

% ===================== VELOCIDADE ANGULAR =====================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular (°/s)');
xlabel('Tempo (s)');

stairs(t, vel_angular, 'LineWidth', 1.5);


ylim([-800, 800]);
xlim([0, xlimite]);

%% CONTROLE E DESEMPENHO COMPUTACIONAL
figure;

% ===================== CONTROLE =====================
subplot(2,1,1); hold on; grid on;
title('Sinal de Controle (v)');
xlabel('Tempo (s)');

stairs(t, u_exp, 'LineWidth', 1.5);

ylim([-15, 15]);
xlim([0, xlimite]);

% ===================== TEMPO COMPUTACIONAL =====================
subplot(2,1,2); grid on; hold on;
title('Tempo Computacional do Algoritmo MPC (ms)');
xlabel('Tempo (s)');

tempo_ms = tempo_comp / 1000;

% Índices
idx_ok = cod_erro ~= -1;
idx_erro = cod_erro == -1;

% Pontos válidos
scatter(t(idx_ok), tempo_ms(idx_ok), 5, 'filled');

% (opcional) pontos com erro transparentes
% scatter(t(idx_erro), tempo_ms(idx_erro), 5, ...
%     'filled', 'MarkerFaceAlpha', 0.1, 'MarkerEdgeAlpha', 0.1);


ylim([0, 15]);
xlim([0, xlimite]);