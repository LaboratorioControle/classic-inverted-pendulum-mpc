%% IDENTIFICAÇÃO DE PARÂMETROS NO SWING UP

clear;
close all;

% Roda os scripts de parâmetros e carrega os dados experimentais do .csv
run init_project;


importados = importdata('data\raw\dados_17032026_mpc_linear_3.csv');

importados = importados.data;

off_set = 500;
end_set = 2515;

t_import = importados(:,1) - importados(off_set,1);
t_import = t_import/1000;
t_import = t_import(off_set:end_set);

angulo_import      = importados(off_set:end_set,2);
vel_angular_import = importados(off_set:end_set,3);
posicao_import     = importados(off_set:end_set,4);
velocidade_import  = importados(off_set:end_set,5);
u_import           = importados(off_set:end_set,6);


% Representa quais curvas serão utilizadas pelo otimizador
y_exp = [posicao_import, angulo_import, velocidade_import, vel_angular_import];

% Vetor que armazena os parâmetros que serão otimizados
% var(1) = dados.motor.Rm;
% var(2) = dados.motor.Kb;
% var(3) = dados.motor.Kt;
% var(4) = dados.motor.R;
% var(5) = dados.carro.m;


var(1) = dados.motor.Rm;
%var(2) = dados.carro.m;
var(2) = dados.carro.c;
%var(4) = dados.motor.Kb;

%var(1) = dados.carro.m;
%var(2) = dados.carro.c;
%var(3) = dados.pendulo.m;
% var(1) = dados.pendulo.m;
% var(2) = dados.carro.m;

% A estratégia utilizada é otimizar multiplicadores dos parâmetros
% informados. Isso é feito para não precisar normalizar os dados e todos
% eles, independente da ordem numérica, terem o mesmo peso
p0 = ones(1, length(var));

% Define os limites dos multiplicadores dos parâmetros
lb_val = 0.001;
ub_val = 1000;

lb = ones(1,length(var))* lb_val;
ub = ones(1,length(var))* ub_val;

% Opções do otimizador
options = optimoptions('lsqnonlin',...
    'Display','iter',...
    'MaxIterations',100);options = optimoptions('lsqnonlin',...
    'Display','iter',...
    'MaxIterations',200,...
    'StepTolerance',1e-8,...
    'FunctionTolerance',1e-8);

% Encontra os multiplicadores ótimos
p_otimo = lsqnonlin(@(p) ...
    cost_function_pendulum_swingUp(p, dados, var, ...
    t_import, y_exp, u_import), ...
    p0, lb, ub, options);

% Exibe os parâmetros estimados já realizando a multiplicação explicada
% anteriormente
disp('Multiplicadores estimados:')
disp(p_otimo)
disp('Parâmetros estimados:')
disp(p_otimo.*var)
