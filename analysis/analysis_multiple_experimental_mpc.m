%% CONFIGURAÇÃO DOS CONTROLADORES

clear;
close all; 

arquivos = {
    'data/raw/Classic/dados_esp32_20260407_Disturbio_SinalEstados_20cm.csv'
    'data/raw/Linear/dados_esp32_20260407_Disturbio_RuidoEstados_20cm.csv'
    'data/raw/Exponencial/dados_esp32_20260407_Disturbio_SinalEstados_20cm.csv'

};

nomes = {'MPC Clássico', 'MPC Par. Trivial', 'MPC Par. Exponencial'};

t_ini_list = [3.7, 3.71, 5.24]; % um valor para cada arquivo


% arquivos = {
%     'data/raw/Classic/dados_esp32_20260330_Trajetoria_01Hz_17cm.csv'
%     'data/raw/Linear/dados_esp32_20260330_Trajetoria_01Hz_17cm.csv'
%     'data/raw/Exponencial/dados_esp32_20260330_Trajetoria_01Hz_17cm.csv'
% 
% };
% 
% nomes = {'MPC Clássico', 'MPC Par. Trivial', 'MPC Par. Exponencial'};
% 
% t_ini_list = [3.1, 4, 4.06]; % um valor para cada arquivo

% arquivos = {
%     'data/raw/Classic/dados_esp32_20260330_Trajetoria_01Hz_20cm.csv'
%     'data/raw/Linear/dados_esp32_20260330_Trajetoria_01Hz_20cm.csv'
%     'data/raw/Exponencial/dados_esp32_20260330_Trajetoria_01Hz_20cm.csv'
% 
% };
% 
% nomes = {'MPC Clássico', 'MPC Linear', 'MPC Exponencial'};
% 
% t_ini_list = [4.18, 3.19, 3.18]; % um valor para cada arquivo

n_ctrl = length(arquivos);

pos_limite = 0.20;
xlimite = 28;
t_fim = xlimite;

%% PRÉ-ALOCAÇÃO
dados = struct();

for i = 1:n_ctrl
    
    data = readtable(arquivos{i});
    
    % Tempo
    t = data.t_ms/1000;
    t = t - t(1);
    
    % Filtro inicial
    t_ini = t_ini_list(i);
    idx = (t >= t_ini) & (t <= t_fim + t_ini);
    
    t = t(idx);
    t = t - t(1);
    
    dados(i).t = t;
    dados(i).pos = data.x_cm(idx);
    dados(i).ang = data.theta_deg(idx);
    dados(i).vel = data.x_dot_cm(idx);
    dados(i).vel_ang = data.theta_dot(idx);

    dados(i).ang = unwrap(deg2rad(dados(i).ang));
    dados(i).ang = rad2deg(dados(i).ang);
    dados(i).ang = dados(i).ang/360;

    
    dados(i).u = data.u(idx);
    dados(i).yref = data.yref(idx);
    
    dados(i).tempo = data.tempo_computacional(idx)/1000; % ms
    dados(i).erro = data.cod_erro(idx);
end

%% ===================== ESTADOS =====================
figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 15])

% POSIÇÃO
subplot(2,2,1); hold on; grid on;
title('Posição do Carro (cm)');
xlabel('Tempo (s)');

for i = 1:n_ctrl
    stairs(dados(i).t, dados(i).pos, 'LineWidth', 1.2);
end

stairs(dados(1).t, dados(1).yref, '--', 'LineWidth', 1.2, 'Color', [0.3 0.3 0.3]);
yline(pos_limite*100, '--');
yline(-pos_limite*100, '--');



ylim([-22, 22]);
xlim([0, xlimite]);

% ÂNGULO
subplot(2,2,2); hold on; grid on;
title('Ângulo (°) / 360°');
xlabel('Tempo (s)');

valores = [0.5 -0.5 1.5 2.5 -1.5];
for v = valores
    h = yline(v, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
end

for i = 1:n_ctrl
    stairs(dados(i).t, dados(i).ang, 'LineWidth', 1.2);
end



%ylim([-5, 365]);
%ylim([-600, 1200]);
xlim([0, xlimite]);

% VELOCIDADE
subplot(2,2,3); hold on; grid on;
title('Velocidade do Carro (cm/s)');
xlabel('Tempo (s)');

for i = 1:n_ctrl
    stairs(dados(i).t, dados(i).vel, 'LineWidth', 1.2);
end

legend([nomes]);

ylim([-50, 50]);
xlim([0, xlimite]);

% VEL ANGULAR
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular (°/s)');
xlabel('Tempo (s)');

for i = 1:n_ctrl
    stairs(dados(i).t, dados(i).vel_ang, 'LineWidth', 1.2);
end

ylim([-800, 800]);
%ylim([-100, 100]);
xlim([0, xlimite]);

%% ===================== CONTROLE =====================
figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 10])
tiledlayout(n_ctrl,1,'TileSpacing','compact');

for i = 1:n_ctrl
    nexttile;
    hold on; grid on;
    
    stairs(dados(i).t, dados(i).u, 'LineWidth', 1.2);
    
    % Linhas de saturação (cinza escuro)
    yline(12,  '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
    yline(-12, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
    
    title(nomes{i});
    xlim([0, xlimite]);
    ylim([-15, 15]);
end

xlabel('Tempo (s)');






figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 7.5])
hold on; grid on;

title('Tempo Computacional (ms)');
xlabel('Tempo (s)');

legendas = {};

for i = 1:n_ctrl
    
    t = dados(i).t;
    tempo = dados(i).tempo;
    erro = dados(i).erro;
    
    % QUEBRA DA LINHA
    tempo_plot = tempo;
    tempo_plot(erro == -1) = NaN;
    
    plot(t, tempo_plot, 'LineWidth', 1.2);
    
    % MÉTRICAS (só pontos válidos)
    idx_ok = erro ~= -1;
    media = mean(tempo(idx_ok));
    pico = max(tempo(idx_ok));
    
    legendas{end+1} = sprintf('%s (μ=%.2f ms, pico=%.2f ms)', ...
                             nomes{i}, media, pico);
end

legend(legendas, 'Location','best');

ylim([0, 25]);
xlim([0, xlimite]);