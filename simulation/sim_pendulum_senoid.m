%% ============================================================
%  SIMULAÇÃO E VALIDAÇÃO DO MODELO DO PÊNDULO INVERTIDO
%  ENTRADA SENOIDAL
%
%  Comparação entre:
%   - Modelo Discreto (RK4)
%   - Modelo Contínuo (ODE45)
%   - Dados Experimentais (opcional)


%% Inicialização

clear;
close all;                       % Fecha todas as figuras abertas

run init_project;
clc;
%% CONFIGURAÇÕES DE PLOT (DINÂMICO)

plot_discreto     = true;   % Modelo discreto (RK4)
plot_continuo     = false;    % Modelo contínuo (ODE45)
plot_experimental = true;   % Dados experimentais (se houver)

%% PARÂMETROS DA SIMULAÇÃO - ENTRADA SENOIDAL

xlimite = 40;
t_final   = 40;                    % Tempo total de simulação [s]
tau       = dados.geral.Ts;        % Tempo de amostragem [s]
amplitude = 150*12/255;            % Amplitude da senoide [V]
duracao   = 2500/1000;             % Duração do sinal [s]
frequencia = 1;                    % Frequência da senoide [Hz]

% Condições iniciais
x0 = [0 0 0 0];                    % [posição, ângulo, velocidade, vel. angular]

%% MODELO DISCRETO (RK4)

% Vetor de tempo discreto
t = (0:tau:t_final)';

% Sinal de entrada senoidal
u = zeros(size(t));
idx = t <= duracao;
u(idx) = amplitude * sin(2*pi*frequencia*t(idx));

% Inicialização do vetor de estados
x = zeros(length(t), length(x0));
x(1,:) = x0;

% Integração discreta
for k = 1:length(t)-1
    x(k+1,:) = RK4_discrete(x(k,:), u(k), tau, dados);
end

% Conversão de unidades
x(:,1) = x(:,1) * 100;         % m → cm
x(:,2) = x(:,2) * (180/pi);    % rad → °
%x(:,2) = wrapTo360((180/pi) * x(:,2));
x(:,3) = x(:,3) * 100;         % m/s → cm/s
x(:,4) = x(:,4) * (180/pi);    % rad/s → °/s

%% MODELO CONTÍNUO (ODE45)

% Intervalo de integração
tspan = [0 t_final];

% Simulação contínua
[t2, x2] = ode45(@(t2,x2) ...
    Modelo_Continuo_Script_Senoide(t2, x2, dados, duracao, amplitude, frequencia), ...
    tspan, x0);

% Conversão de unidades
x2(:,1) = x2(:,1) * 100;
x2(:,2) = x2(:,2) * (180/pi);
%x2(:,2) = wrapTo360((180/pi) * x2(:,2));

x2(:,3) = x2(:,3) * 100;
x2(:,4) = x2(:,4) * (180/pi);

%% DADOS EXPERIMENTAIS (OPCIONAL)

importados = importdata('data\raw\Modelo\dados_16032026_senoide_A150_F1_D2500.csv');
importados = importados.data;

off_set = 0.85;
end_set = 40;

t_import = importados(:,1)/1000;
t_import = t_import - t_import(1);

idx = (t_import >= off_set) & (t_import <= end_set + off_set);

t_import = t_import(idx);
t_import = t_import - t_import(1);


%t_import = importados(:,1) - importados(off_set,1);
%t_import = t_import/1000;
%t_import = t_import(off_set:end_set);

angulo_import      = importados(idx,2);
vel_angular_import = importados(idx,3);
posicao_import     = importados(idx,4);
velocidade_import  = importados(idx,5);
u_import           = importados(idx,6);


angulo_import = unwrap(deg2rad(angulo_import));
angulo_import = rad2deg(angulo_import);
angulo_import = angulo_import - 360;

% 
% ===================== INTERPOLAÇÃO =====================
pos_sim_interp     = interp1(t, x(:,1), t_import, 'linear', 'extrap');
ang_sim_interp     = interp1(t, x(:,2), t_import, 'linear', 'extrap');
vel_sim_interp     = interp1(t, x(:,3), t_import, 'linear', 'extrap');
vel_ang_sim_interp = interp1(t, x(:,4), t_import, 'linear', 'extrap');

% ===================== FUNÇÃO R² =====================
calc_R2 = @(y_exp, y_sim) ...
    1 - sum((y_exp - y_sim).^2) / sum((y_exp - mean(y_exp)).^2);

R2_pos = calc_R2(posicao_import, pos_sim_interp)
R2_ang = calc_R2(angulo_import, ang_sim_interp)
R2_vel = calc_R2(velocidade_import, vel_sim_interp)
R2_vel_ang = calc_R2(vel_angular_import, vel_ang_sim_interp)


%% PLOT DOS RESULTADOS (DINÂMICO)

figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 15])


% ===================== POSIÇÃO DO CARRINHO =====================
subplot(2,2,1); hold on; grid on;
title('Posição do Carro (cm)');
%title(sprintf('Posição do Carro (cm) - R² = %.3f', R2_pos));
xlabel('Tempo (s)');
xlim([0, xlimite]);

if plot_experimental
    stairs(t_import, posicao_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,1), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,1), 'LineWidth', 1);
end

legend_entries = {};
if plot_experimental, legend_entries{end+1} = 'Experimental'; end
if plot_continuo,     legend_entries{end+1} = 'Contínuo';     end
if plot_discreto,     legend_entries{end+1} = 'Simulado';     end
legend(legend_entries);

% ===================== ÂNGULO DO PÊNDULO =====================
subplot(2,2,2); hold on; grid on;
title('Ângulo (°)');
xlabel('Tempo (s)');
xlim([0, xlimite]);
ylim([-70,70]);

if plot_experimental
    stairs(t_import, angulo_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,2), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,2), 'LineWidth', 1);
end

%legend(legend_entries);

% ===================== VELOCIDADE DO CARRINHO =====================
subplot(2,2,3); hold on; grid on;
title('Velocidade (cm/s)');
xlabel('Tempo (s)');
xlim([0, xlimite]);

if plot_experimental
    stairs(t_import, velocidade_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,3), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,3), 'LineWidth', 1);
end

%legend(legend_entries);

% ===================== VELOCIDADE ANGULAR =====================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular (°/s)');
xlabel('Tempo (s)');
xlim([0, xlimite]);

if plot_experimental
    stairs(t_import, vel_angular_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,4), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,4), 'LineWidth', 1);
end

%legend(legend_entries);


%% LIMPEZA

clear k tspan t_final tau idx importados legend_entries plot_discreto plot_continuo plot_experimental;
clear t t2 u off_set;