%% ============================================================
% Controlador Preditivo com Restrições (MPC + Swing-Up)
%
% Estratégia:
% - Swing-Up baseado em energia para levar o pêndulo à região próxima
% - MPC com restrições para estabilização em torno da posição invertida

clear;
run init_project;
%close all;
clc;

%% 1. PARÂMETROS GERAIS DO SISTEMA

A   = dados.planta.A;
B   = dados.planta.B;
tau = dados.geral.Ts;

% Limites físicos (normalizados em SI)
pos_limite     = 20/100;           % posição máxima do carrinho [m]
ang_limite     = 12*(pi/180);      % desvio máximo do ângulo [rad]
vel_limite     = 50/100;           % velocidade máxima do carrinho [m/s]
comando_limite = 12;               % tensão máxima [V]

% Condições iniciais
pos_inicial = 0;
ang_inicial = 0*(pi/180);

% Setpoint
pos_spt = 0/100;

% Tempo total de simulação
tsim = 30;

% Contador de falhas do QP
qp_error_count = 0;

%% 2. CONTROLADOR DE SWING-UP BASEADO EM ENERGIA

dados.controlador.energia.k = 30;   % ganho da lei de energia
dados.controlador.energia.n = 1;    % parâmetro reservado

%% 3. DEFINIÇÃO DO MPC COM RESTRIÇÕES

[~, nu] = size(B);

MPC.A = A;
MPC.B = B;

% Estados rastreados: posição e ângulo
MPC.Cr = [1 0 0 0;
          0 1 0 0];

% Pesos do custo
MPC.Qy = diag([500 100]);   % penalização dos estados rastreados
MPC.Qu = 0.001;            % penalização do esforço de controle
MPC.N  = 30;                % horizonte de predição

% Estados restringidos: posição, ângulo e velocidade
MPC.Cc = [1 0 0 0;
          0 1 0 0;
          0 0 1 0];

% Limites das restrições
MPC.ycmin = [-pos_limite; -ang_limite; -vel_limite];
MPC.ycmax = [ pos_limite;  ang_limite;  vel_limite];

MPC.umin = -comando_limite;
MPC.umax =  comando_limite;

% Restrições sobre incremento de controle (não ativas)
MPC.ulast    = 0;
MPC.deltamin = -1e5;
MPC.deltamax =  1e5;

% Cálculo das matrizes do problema QP
MPC = compute_MPC_Matrices(MPC);

lesN = [1; 5; 10; 15; 20; 25; 30];
Pi_r=compute_Pi_r(lesN,MPC.N,nu);

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

% Referência empilhada
yref = zeros(nt * num_var_reguladas, 1);
yref(1:2:end) = pos_spt;

lesx(1,:) = x0';
lesy(1,:) = MPC.Cr * x0;

x_des = [0 180*pi/180 0 0];

%% 5. CONFIGURAÇÃO DO SOLVER QP (qpOASES)

options = qpOASES_options('default');
%options.enableFarBounds        = 0;
options.maxIter                = 30;
options.terminationTolerance   = 1e-4;
%options.boundTolerance         = 1e-6;
%options.enableRegularisation   = 1;

QP = [];

%% 6. LOOP PRINCIPAL DE SIMULAÇÃO

for i = 1 : nt - MPC.N

    % Distúrbio aplicado aos 15 s
    if lest(i) == 15.0
        lesx(i,4) = lesx(i,4) - 65*pi/180;
    end

    if i == 1
        lesx(i,:) = RK4_discrete(lesx(i,:), 200*12/255, tau, dados);
    end
        

    usar_MPC = (abs(lesx(i,2) - pi) < 15*pi/180) && ...
                (abs(lesx(i,4))      < 100*pi/180);

    if usar_MPC
        % ---------- MPC ----------
        yref_pred = yref(i*num_var_reguladas + 1 : ...
                          (i+MPC.N)*num_var_reguladas);

        err = lesx(i,:) - x_des;

        MPC.F     = MPC.F1*err' + MPC.F2*yref_pred;
        MPC.Bineq = MPC.G1*err' + MPC.G2*MPC.ulast + MPC.G3;

        MPCr=compute_reduced_matrices(MPC,Pi_r);

        %eps_reg = 1e-6;
        %MPCr.H = MPCr.H + eps_reg * eye(size(MPCr.H));

        try
            if i == 1 || isempty(QP)
                [QP, p_opt, ~, exitflag] = ...
                    qpOASES_sequence('i', MPCr.H, MPCr.F, MPCr.Aineq, ...
                                     [], [], ...
                                     [], MPCr.Bineq, options);
            else
                [p_opt, ~, exitflag] = ...
                    qpOASES_sequence('h', QP, MPCr.F, ...
                                     [], [], ...
                                     [], MPCr.Bineq, options);
            end

            u=P_i(1, nu, MPC.N)*(Pi_r*p_opt);

            if exitflag ~= 0 || any(~isfinite(p_opt))
                qp_error_count = qp_error_count + 1;
            else
                qp_error_count = 0;
            end
        catch 
            qp_error_count = qp_error_count + 1;
        end

    else
        % ---------- SWING-UP ----------
        u = swingUp_energy_based_controller(lesx(i,:), dados);

        % Proteção contra deslocamento excessivo
        if abs(lesx(i,1)) >= 0.24
            u = -15 * lesx(i,1);
        end

        u = sat(u, comando_limite, -comando_limite);
    end

    % Integração do modelo não linear
    xplus = RK4_discrete(lesx(i,:), u, tau, dados);

    % Armazenamento
    lesu(i)      = u;
    MPC.ulast    = u;
    lesx(i+1,:)  = xplus;
    lesy(i+1,:)  = MPC.Cr * xplus';
end

posicao = lesx(1:nt-MPC.N,1).*100;
angulo = lesx(1:nt-MPC.N,2).*180/pi;
velocidade = lesx(1:nt-MPC.N,3).*100;
vel_angular = lesx(1:nt-MPC.N,4).*180/pi;
tempo = lest(1:nt-MPC.N);
comando = lesu(1:nt-MPC.N);

dados.controlador.MPC.ComRestricoes = MPC;

%% 7. PLOTS DOS RESULTADOS (UNIDADES FÍSICAS)

figure('Name','Posição e Vel. Linear','Color','w')

% ---------------- POSIÇÃO DO CARRINHO ----------------
subplot(2,1,1)
plot(tempo, posicao, 'LineWidth', 1.5)
hold on
yline( pos_limite*100, '--r')
yline(-pos_limite*100, '--r')
grid on
ylabel('Posição [cm]')
title('Resposta do Sistema – MPC + Swing-Up')

% ---------------- VELOCIDADE DO CARRINHO ----------------
subplot(2,1,2)
plot(tempo, velocidade, 'LineWidth', 1.5)
hold on
yline( vel_limite*100, '--r')
yline(-vel_limite*100, '--r')
grid on
ylabel('Velocidade [cm/s]')

% ---------------- ÂNGULO DO PÊNDULO ----------------

figure('Name','Ângulo e Vel. Angular','Color','w')

subplot(2,1,1)
plot(tempo, angulo, 'LineWidth', 1.5)
hold on
yline( ang_limite*180/pi + 180, '--r')
yline(-ang_limite*180/pi + 180, '--r')
grid on
ylabel('Ângulo [graus]')

% ---------------- VELOCIDADE ANGULAR ----------------
subplot(2,1,2)
plot(tempo, vel_angular, 'LineWidth', 1.5)
grid on
ylabel('Vel. Angular [graus/s]')
xlabel('Tempo [s]')

% ---------------- SINAL DE CONTROLE -----------------

figure('Name','Sinal de Controle','Color','w')
plot(tempo, comando, 'LineWidth', 1.5)
hold on
yline( comando_limite, '--r')
yline(-comando_limite, '--r')
grid on
xlabel('Tempo [s]')
ylabel('Tensão [V]')
title('Sinal de Controle')

clear ang_inicial A B Bineq err exitflag;
clear F i nt nu num_var_reguladas options pos_inicial qp_error_count QP tau tsim u usar_MPC;
clear x0 x_des xplus yref_pred p_opt lest lesu yref lesx lesy;