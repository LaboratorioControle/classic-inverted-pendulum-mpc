%% ============================================================
%  SIMULAÇÃO E VALIDAÇÃO DO MODELO DO PÊNDULO INVERTIDO
%  Comparação entre:
%   - Modelo Discreto (RK4)
%   - Modelo Contínuo (ODE45)
%   - Dados Experimentais


%% Inicialização

clear;
close all;                       % Fecha todas as figuras abertas

run init_project;
close all;                       % Fecha todas as figuras abertas


%% CONFIGURAÇÕES DE PLOT (DINÂMICO)
% Ative (true) ou desative (false) o que deseja visualizar
plot_discreto     = true;   % Modelo discreto (RK4)
plot_continuo     = false;    % Modelo contínuo (ODE45)
plot_experimental = true;    % Dados experimentais


%% SIMULAÇÃO DO MODELO - ENTRADA DEGRAU

% Tempo total de simulação
t_final = 40;                           % [s]

% Tempo de amostragem
tau = dados.geral.Ts;                  % [s]

% Parâmetros do degrau de entrada
amplitude = -200*12/255;         % [V]
duracao   = 500/1000;           % [s]

% Condições iniciais
x0 = [0 0 0 0];                        % [posição, ângulo, velocidade, vel. angular]


%% MODELO DISCRETO (INTEGRAÇÃO RK4)

% Vetor de tempo discreto
t = (0:tau:t_final)';

% Entrada de controle (degrau)
u = zeros(size(t));
u(t <= duracao) = amplitude;

% Inicialização do estado
x = zeros(length(t), length(x0));
x(1,:) = x0;

% Integração pelo método de Runge-Kutta de 4ª ordem
for k = 1:length(t)-1
    x(k+1,:) = RK4_discrete(x(k,:), u(k), tau, dados);
end

% Conversão de unidades
x(:,1) = x(:,1) * 100;         % m → cm
%x(:,2) = wrapTo360(x(:,2) * (180/pi));    % rad → °
x(:,2) = x(:,2) * (180/pi);
x(:,3) = x(:,3) * 100;         % m/s → cm/s
x(:,4) = x(:,4) * (180/pi);    % rad/s → °/s

%% MODELO CONTÍNUO (ODE45)


% Intervalo de integração
tspan = [0 t_final];

% Simulação contínua
[t2, x2] = ode45(@(t2,x2) ...
    Modelo_Continuo_Script_Degrau(t2, x2, dados, duracao, amplitude), ...
    tspan, x0);

% Conversão de unidades
x2(:,1) = x2(:,1) * 100;
x2(:,2) = x2(:,2) * (180/pi);
x2(:,3) = x2(:,3) * 100;
x2(:,4) = x2(:,4) * (180/pi);


%% DADOS EXPERIMENTAIS

% Importação dos dados experimentais
importados = importdata('data\raw\Modelo\dados_16032026_degrau_D500_I200_L.csv');
importados = importados.data;

off_set = 0.95;
end_set = 40;


t_import = importados(:,1)/1000;
t_import = t_import - t_import(1);

idx = (t_import >= off_set) & (t_import <= end_set + off_set);

t_import = t_import(idx);
t_import = t_import - t_import(1);


angulo_import      = importados(idx,2);
vel_angular_import = importados(idx,3);
posicao_import     = importados(idx,4);
velocidade_import  = importados(idx,5);
u_import           = importados(idx,6);

angulo_import = unwrap(deg2rad(angulo_import));
angulo_import = rad2deg(angulo_import);


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
xlabel('Tempo (s)');
ylim([-20,1]);

if plot_experimental
    stairs(t_import, posicao_import, 'LineWidth', 1.1);
end
if plot_continuo
    plot(t2, x2(:,1), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,1), 'LineWidth', 1.1);
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
ylim([-30,30]);

if plot_experimental
    stairs(t_import, angulo_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,2), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,2), 'LineWidth', 1);
end


% ===================== VELOCIDADE DO CARRINHO =====================
subplot(2,2,3); hold on; grid on;
title('Velocidade (cm/s)');
xlabel('Tempo (s)');

if plot_experimental
    stairs(t_import, velocidade_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,3), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,3), 'LineWidth', 1);
end


% ===================== VELOCIDADE ANGULAR =====================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular (°/s)');
xlabel('Tempo (s)');

if plot_experimental
    stairs(t_import, vel_angular_import, 'LineWidth', 1);
end
if plot_continuo
    plot(t2, x2(:,4), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,4), 'LineWidth', 1);
end



%% LIMPEZA

%clear k tspan t_final tau amplitude_degrau duracao_degrau angulo_import importados legend_entries plot_continuo plot_discreto;
%clear plot_experimental posicao_import t t2 t_import u vel_angular_import velocidade_import x x0 x2 x_import;
