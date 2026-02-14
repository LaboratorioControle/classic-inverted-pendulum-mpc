%% IDENTIFICAÇÃO DE PARÂMETROS

clear;
close all;

% Roda os scripts de parâmetros e carrega os dados experimentais do .csv
run init_project;
run sim_pendulum_senoid;

% Representa quais curvas serão utilizadas pelo otimizador
y_exp = [posicao_import, vel_angular_import];

% Vetor que armazena os parâmetros que serão otimizados
var(1) = dados.pendulo.b;
var(2) = dados.carro.m;
var(3) = dados.pendulo.I;

% A estratégia utilizada é otimizar multiplicadores dos parâmetros
% informados. Isso é feito para não precisar normalizar os dados e todos
% eles, independente da ordem numérica, terem o mesmo peso
p0 = ones(1, length(var));

% Define os limites dos multiplicadores dos parâmetros
lb_val = 0.0001;
ub_val = 1000;

lb = ones(1,length(var))* lb_val;
ub = ones(1,length(var))* ub_val;

% Opções do otimizador
options = optimoptions('lsqnonlin',...
    'Display','iter',...
    'MaxIterations',100);options = optimoptions('lsqnonlin',...
    'Display','iter',...
    'MaxIterations',200,...
    'StepTolerance',1e-0,...
    'FunctionTolerance',1e-10);

% Encontra os multiplicadores ótimos
p_otimo = lsqnonlin(@(p) ...
    cost_function_pendulum(p, dados, var, ...
    t_import, y_exp, duracao, amplitude, frequencia, x0), ...
    p0, lb, ub, options);

% Exibe os parâmetros estimados já realizando a multiplicação explicada
% anteriormente
disp('Multiplicadores estimados:')
disp(p_otimo)
disp('Parâmetros estimados:')
disp(p_otimo.*var)
