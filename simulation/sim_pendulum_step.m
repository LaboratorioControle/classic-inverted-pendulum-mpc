%% ============================================================
%  SIMULAÇÃO E VALIDAÇÃO DO MODELO DO PÊNDULO INVERTIDO
%  Comparação entre:
%   - Modelo Discreto (RK4)
%   - Modelo Contínuo (ODE45)
%   - Dados Experimentais


%% Inicialização
close all;                       % Fecha todas as figuras abertas


%% CONFIGURAÇÕES DE PLOT (DINÂMICO)
% Ative (true) ou desative (false) o que deseja visualizar
plot_discreto     = true;   % Modelo discreto (RK4)
plot_continuo     = false;    % Modelo contínuo (ODE45)
plot_experimental = true;    % Dados experimentais


%% SIMULAÇÃO DO MODELO - ENTRADA DEGRAU

% Tempo total de simulação
t_final = 45;                           % [s]

% Tempo de amostragem
tau = dados.geral.Ts;                  % [s]

% Parâmetros do degrau de entrada
amplitude_degrau = -200*12/255;         % [V]
duracao_degrau   = 500/1000;           % [s]

% Condições iniciais
x0 = [0 0 0 0];                        % [posição, ângulo, velocidade, vel. angular]


%% MODELO DISCRETO (INTEGRAÇÃO RK4)

% Vetor de tempo discreto
t = (0:tau:t_final)';

% Entrada de controle (degrau)
u = zeros(size(t));
u(t <= duracao_degrau) = amplitude_degrau;

% Inicialização do estado
x = zeros(length(t), length(x0));
x(1,:) = x0;

% Integração pelo método de Runge-Kutta de 4ª ordem
for k = 1:length(t)-1
    x(k+1,:) = RK4_discrete(x(k,:), u(k), tau, dados);
end

% Conversão de unidades
x(:,1) = x(:,1) * 100;         % m → cm
x(:,2) = wrapTo360(x(:,2) * (180/pi));    % rad → °
x(:,3) = x(:,3) * 100;         % m/s → cm/s
x(:,4) = x(:,4) * (180/pi);    % rad/s → °/s

%% MODELO CONTÍNUO (ODE45)


% Intervalo de integração
tspan = [0 t_final];

% Simulação contínua
[t2, x2] = ode45(@(t2,x2) ...
    Modelo_Continuo_Script_Degrau(t2, x2, dados, duracao_degrau, amplitude_degrau), ...
    tspan, x0);

% Conversão de unidades
x2(:,1) = x2(:,1) * 100;
x2(:,2) = x2(:,2) * (180/pi);
x2(:,3) = x2(:,3) * 100;
x2(:,4) = x2(:,4) * (180/pi);


%% DADOS EXPERIMENTAIS

% Importação dos dados experimentais
importados = importdata('data\raw\dados_degrau_I200_D500_16022026.csv');

off_set = 101;

t_import = importados(:,1) - importados(off_set,1);
t_import = t_import/1000;
t_import = t_import(off_set:end);

angulo_import      = importados(off_set:end,2);
vel_angular_import = importados(off_set:end,3);
posicao_import     = importados(off_set:end,4);
velocidade_import  = importados(off_set:end,5);
u_import           = importados(off_set:end,6);

%% PLOT DOS RESULTADOS (DINÂMICO)

figure;

% ===================== POSIÇÃO DO CARRINHO =====================
subplot(2,2,1); hold on; grid on;
title('Posição do Carrinho');
xlabel('Tempo (s)');
ylabel('Posição (cm)');

if plot_experimental
    stairs(t_import, posicao_import, 'LineWidth', 1.2);
end
if plot_continuo
    plot(t2, x2(:,1), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,1), 'LineWidth', 1.2);
end

legend_entries = {};
if plot_experimental, legend_entries{end+1} = 'Experimental'; end
if plot_continuo,     legend_entries{end+1} = 'Contínuo';     end
if plot_discreto,     legend_entries{end+1} = 'Discreto';     end
legend(legend_entries);

% ===================== ÂNGULO DO PÊNDULO =====================
subplot(2,2,2); hold on; grid on;
title('Ângulo do Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');

if plot_experimental
    stairs(t_import, angulo_import);
end
if plot_continuo
    plot(t2, x2(:,2), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,2));
end

legend(legend_entries);

% ===================== VELOCIDADE DO CARRINHO =====================
subplot(2,2,3); hold on; grid on;
title('Velocidade do Carrinho');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');

if plot_experimental
    stairs(t_import, velocidade_import);
end
if plot_continuo
    plot(t2, x2(:,3), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,3));
end

legend(legend_entries);

% ===================== VELOCIDADE ANGULAR =====================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular do Pêndulo');
xlabel('Tempo (s)');
ylabel('Velocidade Angular (°/s)');

if plot_experimental
    stairs(t_import, vel_angular_import);
end
if plot_continuo
    plot(t2, x2(:,4), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,4));
end

legend(legend_entries);


%% LIMPEZA

clear k tspan t_final tau amplitude_degrau duracao_degrau angulo_import importados legend_entries plot_continuo plot_discreto;
clear plot_experimental posicao_import t t2 t_import u vel_angular_import velocidade_import x x0 x2 x_import;
