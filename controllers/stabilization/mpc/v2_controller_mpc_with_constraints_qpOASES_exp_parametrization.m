%% ============================================================
% Controlador Preditivo com Restrições (MPC + Swing-Up)
%
% Estratégia:
% - Swing-Up baseado em energia para levar o pêndulo à região próxima
% - MPC com restrições para estabilização em torno da posição invertida

clear;
run init_project;
run analysis_experimental_mpc.m;
close all;
clc;

%% 1. PARÂMETROS GERAIS DO SISTEMA

A   = dados.planta.A;
B   = dados.planta.B;
tau = dados.geral.Ts;

% Limites físicos (normalizados em SI)
pos_limite     = 18/100;           % posição máxima do carrinho [m]
ang_limite     = 15*(pi/180);      % desvio máximo do ângulo [rad]
vel_limite     = 50/100;           % velocidade máxima do carrinho [m/s]
comando_limite = 12;               % tensão máxima [V]

% Condições iniciais
pos_inicial = 0;
ang_inicial = 180*(pi/180);

% Setpoint
pos_spt = 15/100;

% Tempo total de simulação
tsim = 30;
xlimite = tsim;

% Contador de falhas do QP
qp_error_count = 0;

%% 2. CONTROLADOR DE SWING-UP BASEADO EM ENERGIA

dados.controlador.energia.k = 13;   % ganho da lei de energia

%% 3. DEFINIÇÃO DO MPC COM RESTRIÇÕES

[~, nu] = size(B);

MPC.A = A;
MPC.B = B;

% Estados rastreados: posição e ângulo
MPC.Cr = [1 0 0 0;
          0 1 0 0];

% Pesos do custo
MPC.Qy = diag([450 100]);   % penalização dos estados rastreados
MPC.Qu = 0.001;            % penalização do esforço de controle
MPC.N  = 33;                % horizonte de predição

% Estados restringidos: posição, ângulo e velocidade
MPC.Cc = [1 0 0 0];
          %0 1 0 0];
          %0 0 1 0];

% Limites das restrições
MPC.ycmin = [-pos_limite];% -ang_limite];% -vel_limite];
MPC.ycmax = [ pos_limite];%  ang_limite];%  vel_limite];

MPC.umin = -comando_limite;
MPC.umax =  comando_limite;

% Restrições sobre incremento de controle (não ativas)
MPC.ulast    = 0;
MPC.deltamin = -1e2;
MPC.deltamax =  1e2;

% Cálculo das matrizes do problema QP
MPC = compute_MPC_Matrices(MPC);


par.lambda = 0.2;
par.ne = 3; % Número de exponenciais para cada atuador (Matriz coluna)
par.tau = tau;
par.alpha = 0.5;
par.N = MPC.N;
%--------------------------

Pi_e=compute_Pi_e(par);

%% 4. INICIALIZAÇÃO DA SIMULAÇÃO

x0 = [pos_inicial; ang_inicial; 0; 0];
u  = 0;

lest = (0:tau:tsim)';
nt   = length(lest);

num_var_reguladas = size(MPC.Cr,1);

% Vetores de armazenamento
lesx = zeros(nt, 4);
lesy = zeros(nt, num_var_reguladas);
lesu = zeros(nt, nu);

lesx(1,:) = x0';
lesy(1,:) = MPC.Cr * x0;

x_des = [0 180*pi/180 0 0];

%% Geração de Trajetória
yref = zeros(nt * num_var_reguladas, 1);

for i=1:nt
    aux = 0;
    
    t_aux = i*tau;

    if t_aux >= 1.1
        aux = 0.05;
    end
    if t_aux >= 2.1
        aux = 0.10;
    end
    if t_aux >= 3.1
        aux = 0.15;
    end
    yref((i-1)*2 + 1) = aux;
end

% Referência empilhada
%yref = zeros(nt * num_var_reguladas, 1);
%yref(1:2:end) = pos_spt;

%% 5. CONFIGURAÇÃO DO SOLVER QP (qpOASES)

options = qpOASES_options('default');
%options.enableFarBounds        = 0;
options.maxIter                = 10;
options.terminationTolerance   = 1e-4;
%options.boundTolerance         = 1e-6;
%options.enableRegularisation   = 1;

QP = [];

%% 6. LOOP PRINCIPAL DE SIMULAÇÃO

for i = 1 : nt - MPC.N

    % Distúrbio aplicado aos 15 s
    %if lest(i) == 15.0
    %    lesu(i,1) = lesu(i,1) + 9;
    %    u = sat(u, comando_limite, -comando_limite);
    %end

    usar_MPC = (abs(lesx(i,2) - pi) < 15*pi/180) && ...
        (abs(lesx(i,4))      < 100*pi/180);

    if usar_MPC

        %tic  

        yref_pred = yref(i*num_var_reguladas+1:(i+MPC.N)*num_var_reguladas);

        err = lesx(i,:) - x_des;

        MPC.F     = MPC.F1*err' + MPC.F2*yref_pred;
        MPC.Bineq = MPC.G1*err' + MPC.G2*MPC.ulast + MPC.G3;

        MPCr=compute_reduced_matrices(MPC,Pi_e);


        if i == 1 
            [QP, p_opt, ~, exitflag] = ...
                qpOASES_sequence('i', MPCr.H, MPCr.F, MPCr.Aineq, [], [], [], MPCr.Bineq, options);
        else
            [p_opt, ~, exitflag] = ...
                qpOASES_sequence('h', QP, MPCr.F, [], [], [], MPCr.Bineq, options);
        end

        u=P_i(1, nu, MPC.N)*(Pi_e*p_opt);
        
        if(exitflag ~= 0)
            u = 0;
        end

        % Distúrbio aplicado aos 15 s
        if i >= 1510 && i < 1535
            u = u - 9;
            u = sat(u, comando_limite, -comando_limite);
            disp("opa");
        end

        %tempos_mpc(i) = toc;
    else
        % ---------- SWING-UP ----------
        u = swingUp_energy_based_controller(lesx(i,:), dados);

        % Proteção contra deslocamento excessivo
        if abs(lesx(i,1)) >= 0.175
            u = -15 * lesx(i,1);
        end

        u = sat(u, comando_limite, -comando_limite);
    end

    % if i == 1
    %     lesx(i,:) = RK4_discrete(lesx(i,:), 200*12/255, tau, dados);
    % end

    % Integração do modelo não linear
    xplus = RK4_discrete(lesx(i,:), u, tau, dados);

    %lesx(i,2) = wrapTo2Pi(lesx(i,2));

    % Armazenamento
    lesu(i)      = u;
    MPC.ulast    = u;
    lesx(i+1,:)  = xplus;
    lesy(i+1,:)  = MPC.Cr * xplus';
end

simulacao.mpc.exp.posicao = lesx(1:nt-MPC.N,1).*100;
simulacao.mpc.exp.angulo = lesx(1:nt-MPC.N,2).*180/pi;
simulacao.mpc.exp.velocidade = lesx(1:nt-MPC.N,3).*100;
simulacao.mpc.exp.vel_angular = lesx(1:nt-MPC.N,4).*180/pi;
simulacao.mpc.exp.tempo = lest(1:nt-MPC.N);
simulacao.mpc.exp.comando = lesu(1:nt-MPC.N);

%simulacao.mpc.exp.angulo = unwrap(deg2rad(simulacao.mpc.exp.angulo));
%simulacao.mpc.exp.angulo = rad2deg(simulacao.mpc.exp.angulo);
simulacao.mpc.exp.angulo = simulacao.mpc.exp.angulo/360;

dados.controlador.MPC.ComRestricoes = MPC;

%% ===================== ESTADOS (SIM vs EXP) =====================

figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 15])

% ================= POSIÇÃO =================
subplot(2,2,1); hold on; grid on;
title('Posição do Carro (cm)');
xlabel('Tempo (s)');

% Experimental
stairs(t, posicao);

% Simulação
stairs(simulacao.mpc.exp.tempo, simulacao.mpc.exp.posicao, 'LineWidth', 1);



% Referência
stairs(t, yref_exp, '--', 'LineWidth', 1.0, 'Color', [0.3 0.3 0.3]);




% Limites (padrão cinza)
yline(pos_limite*100, '--', 'Color', [0.4 0.4 0.4]);
yline(-pos_limite*100, '--', 'Color', [0.4 0.4 0.4]);

ylim([-22, 22]);
xlim([0, xlimite]);

legend('Experimental','Simulado');



% ================= ÂNGULO =================
subplot(2,2,2); hold on; grid on;
title('Ângulo (°) / 360°');
xlabel('Tempo (s)');

% Linhas de referência angular
valores = [0.5 -0.5 -1.5];
for v = valores
    yline(v, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8);
end

stairs(t, angulo, 'LineWidth', 1);

stairs(simulacao.mpc.exp.tempo, simulacao.mpc.exp.angulo, 'LineWidth', 1);

xlim([0, xlimite]);

% ================= VELOCIDADE =================
subplot(2,2,3); hold on; grid on;
title('Velocidade (cm/s)');
xlabel('Tempo (s)');

stairs(t, velocidade, 'LineWidth', 1);
stairs(simulacao.mpc.exp.tempo, simulacao.mpc.exp.velocidade, 'LineWidth', 1);


ylim([-50, 50]);
xlim([0, xlimite]);

% ================= VEL ANGULAR =================
subplot(2,2,4); hold on; grid on;
title('Velocidade Angular (°/s)');
xlabel('Tempo (s)');

stairs(t, vel_angular, 'LineWidth', 1);
stairs(simulacao.mpc.exp.tempo, simulacao.mpc.exp.vel_angular, 'LineWidth', 1);


ylim([-800, 800]);
xlim([0, xlimite]);

%% ===================== CONTROLE =====================

figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 8])
hold on; grid on;

stairs(t, u_exp, 'LineWidth', 1);
stairs(simulacao.mpc.exp.tempo, simulacao.mpc.exp.comando, 'LineWidth', 1);


% Saturação padrão cinza
yline(comando_limite,  '--', 'Color', [0.4 0.4 0.4]);
yline(-comando_limite, '--', 'Color', [0.4 0.4 0.4]);

xlabel('Tempo (s)');
ylabel('Tensão (V)');
title('Sinal de Controle');

xlim([0, xlimite]);
ylim([-15, 15]);

%legend('Experimental','Simulado');

% ---------------- SINAL DE CONTROLE -----------------

clear ang_inicial A B Bineq err exitflag;
clear F i MPC nt nu num_var_reguladas options pos_inicial qp_error_count QP tau tsim u usar_MPC;
clear x0 x_des xplus yref_pred p_opt lest lesu yref lesx lesy;