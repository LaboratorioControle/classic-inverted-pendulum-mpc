%% 1. Otimização dos Parâmetros do LQR
clc;
% Supondo que 'dados' já está carregado ou foi definido

% Definir a função de custo (Handle para a função que criamos)
% Precisamos passar 'dados' como parâmetro extra
objectiveFcn = @(p) simular_e_custo_lqr(p, dados);

% Definir o número de variáveis [q1, q2, q3, q4, r]
n_vars = 5;

% Definir Limites (Bounds) para os parâmetros
% [q_pos, q_ang, q_vel_pos, q_vel_ang, r_ctrl]
lb = [0,   0,   0,   0,   1e-4]; % Limites inferiores (Q >= 0, R > 0)
ub = [100, 100, 10,  10,  1];    % Limites superiores (AJUSTE CONFORME NECESSÁRIO)

% Configurar o Algoritmo Genético (GA)
% Requer a Global Optimization Toolbox
options = optimoptions('ga', ...
    'Display', 'iter', ...          % Mostrar progresso
    'PlotFcn', @gaplotbestf, ...   % Plotar melhor custo a cada geração
    'UseParallel', true);         % Usar processamento paralelo se disponível

% Executar a otimização
disp('Iniciando otimização dos parâmetros Q e R...');
[p_opt, J_min] = ga(objectiveFcn, n_vars, [], [], [], [], lb, ub, [], options);

% Extrair os resultados ótimos
controlador.lqr.Q = diag(p_opt(1:4));
controlador.lqr.R = p_opt(5);
controlador.lqr.K = dlqr(dados.planta.A, dados.planta.B, controlador.lqr.Q, controlador.lqr.R);

disp("Otimização concluída. Custo mínimo: " + J_min);
disp("Q otimizado = "); disp(controlador.lqr.Q);
disp("R otimizado = "); disp(controlador.lqr.R);
disp("K otimizado = "); disp(controlador.lqr.K);

%% 2. Simulação em malha fechada (COM VALORES OTIMIZADOS)
% Esta seção é IDÊNTICA à sua, mas agora ela usa 
% os 'controlador.lqr.K' que acabamos de otimizar.

Ts = dados.geral.Ts; % Período de Amostragem
Tf = dados.geral.Tf; % Tempo final da simulação

x(1,:) = [dados.geral.inicial.x0 dados.geral.inicial.theta0*pi/180 0 0]; % Estado Inicial
u_volt(1) = 0;
u_force(1) = 0;
t(1) = 0;

x_des = [0; 0; 0; 0];            % Estado desejado

i = 1;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x));
dead_zone = 0.5*pi/180;

for k = 0:Ts:Tf
    
    novo_estado = RK4_discrete(x(i,:),u_force(i),Ts,dados)';
    x(i+1,:) = novo_estado';

    t(i + 1) = k;
    
    if abs(novo_estado(2)) < dead_zone % Usei 'abs' aqui
        u_volt(i+1) = 0;
    else
        % Usando o K otimizado
        u_volt(i+1) = -controlador.lqr.K * (novo_estado - x_des);
        u_volt(i+1) = sat(u_volt(i+1),12,-12);
    end

    u_force(i+1) = Volt2Force(u_volt(i+1),novo_estado(3),dados.motor);

    i = i+1;
end

simulacao.tempo = t;
simulacao.angulo = (180/pi).* x(:,2);
simulacao.velocidade_pendulo = (180/pi).*x(:,4);
simulacao.posicao = x(:,1).*100;
simulacao.velocidade_carro = x(:,3).*100;
simulacao.u_force = u_force;
simulacao.u_volt = u_volt;

clear k u_force u_volt i novo_estado t Tf Ts x x_des sat;

%% 3. Plot dos dados (Igual ao seu)

figure;
subplot(2,2,1);
stairs(simulacao.tempo, simulacao.angulo,'LineWidth',2); grid on;
title('Posição Angular do Pêndulo [°] (Otimizado)');

subplot(2,2,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular do Pêndulo [°/s] (Otimizado)');

subplot(2,2,3);
stairs(simulacao.tempo, simulacao.posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [cm] (Otimizado)');

subplot(2,2,4);
stairs(simulacao.tempo, simulacao.velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [cm/s] (Otimizado)');

figure;

subplot(1,2,1);
stairs(simulacao.tempo,simulacao.u_force,'LineWidth',2); grid on;
title('Comando - Força [F] (Otimizado)');

subplot(1,2,2);
stairs(simulacao.tempo,simulacao.u_volt,'LineWidth',2); grid on;
title('Comando - Tensão [V] (Otimizado)');