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
%x(:,2) = x(:,2) * (180/pi);    % rad → °
x(:,2) = wrapTo360((180/pi) * x(:,2));
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
%x2(:,2) = x2(:,2) * (180/pi);
x2(:,2) = wrapTo360((180/pi) * x2(:,2));

x2(:,3) = x2(:,3) * 100;
x2(:,4) = x2(:,4) * (180/pi);

%% DADOS EXPERIMENTAIS (OPCIONAL)

importados = importdata('data\raw\dados_16032026_senoide_A150_F1_D2500.csv');
importados = importados.data;

off_set = 85;
end_set = 4081;

t_import = importados(:,1) - importados(off_set,1);
t_import = t_import/1000;
t_import = t_import(off_set:end_set);

angulo_import      = importados(off_set:end_set,2);
vel_angular_import = importados(off_set:end_set,3);
posicao_import     = importados(off_set:end_set,4);
velocidade_import  = importados(off_set:end_set,5);
u_import           = importados(off_set:end_set,6);


%% PLOT DOS RESULTADOS (DINÂMICO)

figure;

% ===================== POSIÇÃO DO CARRINHO =====================
subplot(2,2,1); hold on; grid on;
title('Posição do Carro');
xlabel('Tempo (s)');
ylabel('Posição (cm)');

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
title('Ângulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');

if plot_experimental
    stairs(t_import, angulo_import, 'LineWidth', 1.1);
end
if plot_continuo
    plot(t2, x2(:,2), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,2), 'LineWidth', 1.1);
end

legend(legend_entries);

% ===================== VELOCIDADE DO CARRINHO =====================
subplot(2,2,3); hold on; grid on;
title('Velocidade do Carro');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');

if plot_experimental
    stairs(t_import, velocidade_import, 'LineWidth', 1.1);
end
if plot_continuo
    plot(t2, x2(:,3), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,3), 'LineWidth', 1.1);
end

legend(legend_entries);

% ===================== VELOCIDADE ANGULAR =====================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Velocidade Angular (°/s)');

if plot_experimental
    stairs(t_import, vel_angular_import, 'LineWidth', 1.1);
end
if plot_continuo
    plot(t2, x2(:,4), 'LineWidth', 1.5);
end
if plot_discreto
    stairs(t, x(:,4), 'LineWidth', 1.1);
end

legend(legend_entries);


%% LIMPEZA

clear k tspan t_final tau idx importados legend_entries plot_discreto plot_continuo plot_experimental;
clear t t2 u off_set;