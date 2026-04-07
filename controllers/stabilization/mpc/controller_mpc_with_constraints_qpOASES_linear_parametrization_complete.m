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
ang_limite     = 10*(pi/180);      % desvio máximo do ângulo [rad]
vel_limite     = 50/100;           % velocidade máxima do carrinho [m/s]
comando_limite = 12;               % tensão máxima [V]

% Condições iniciais
pos_inicial = 0;
ang_inicial = 180*(pi/180);

% Setpoint
pos_spt = 0/100;

% Tempo total de simulação
tsim = 35;

% Contador de falhas do QP
qp_error_count = 0;

%% 2. CONTROLADOR DE SWING-UP BASEADO EM ENERGIA

dados.controlador.energia.k = 2;   % ganho da lei de energia
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
MPC.N  = 35;                % horizonte de predição

% Estados restringidos: posição, ângulo e velocidade
MPC.Cc = [1 0 0 0];
%          0 1 0 0;
%          0 0 1 0];

% Limites das restrições
MPC.ycmin = [-pos_limite]; %-ang_limite; -vel_limite];
MPC.ycmax = [ pos_limite];%  ang_limite;  vel_limite];

MPC.umin = -comando_limite;
MPC.umax =  comando_limite;

% Restrições sobre incremento de controle (não ativas)
MPC.ulast    = 0;
MPC.deltamin = -1e5;
MPC.deltamax =  1e5;

% Cálculo das matrizes do problema QP
MPC = compute_MPC_Matrices(MPC);

lesN = [1; 7; 14; 21; 28];
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

tempos_mpc = zeros(nt,1);

% Referência empilhada
yref = zeros(nt * num_var_reguladas, 1);

for i=1:nt
    yref((i-1)*2 + 1) = 0.15*sin(2*pi*0.2*i*tau);
end


lesx(1,:) = x0';
lesy(1,:) = MPC.Cr * x0;

x_des = [0 180*pi/180 0 0];

%% 5. CONFIGURAÇÃO DO SOLVER QP (qpOASES)

options = qpOASES_options('default');
%options.enableFarBounds        = 0;
options.maxIter                = 8;
options.terminationTolerance   = 1e-4;
%options.boundTolerance         = 1e-6;
%options.enableRegularisation   = 1;

QP = [];

%% 6. LOOP PRINCIPAL DE SIMULAÇÃO

for i = 1 : nt - MPC.N

    % Distúrbio aplicado aos 15 s
    %if lest(i) == 15.0
        %lesx(i,4) = lesx(i,4) + 15*pi/180;
    %end

    if i == 1
        lesx(i,:) = RK4_discrete(lesx(i,:), 200*12/255, tau, dados);
    end
        

    usar_MPC = (abs(lesx(i,2) - pi) < 15*pi/180) && ...
                (abs(lesx(i,4))      < 100*pi/180);

    if usar_MPC

        tic  

        % ---------- MPC ----------
        yref_pred = yref(i*num_var_reguladas + 1 : ...
                          (i+MPC.N)*num_var_reguladas);

        err = lesx(i,:) - x_des;

        MPC.F     = MPC.F1*err' + MPC.F2*yref_pred;
        MPC.Bineq = MPC.G1*err' + MPC.G2*MPC.ulast + MPC.G3;

        MPCr=compute_reduced_matrices(MPC,Pi_r);

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

        if(qp_error_count > 0)
            u = 0;
        end
        
        tempos_mpc(i) = toc;
    else
        % ---------- SWING-UP ----------
        u = swingUp_energy_based_controller(lesx(i,:), dados);

        % Proteção contra deslocamento excessivo
        if abs(lesx(i,1)) >= 0.20
            u = -15 * lesx(i,1);
        end

        u = sat(u, comando_limite, -comando_limite);
    end

    % Integração do modelo não linear
    xplus = RK4_discrete(lesx(i,:), u, tau, dados);

    lesx(i,2) = wrapTo2Pi(lesx(i,2));

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

%% ================= ESTADOS (2x2) =================
figure

% -------- POSIÇÃO --------
subplot(2,2,1)
plot(tempo, posicao, 'LineWidth', 1.5)
hold on
yline(pos_limite*100, '--r')
yline(-pos_limite*100, '--r')
plot(tempo, yref(1:2:(nt-MPC.N)*2)*100)
grid on
ylabel('Posição [cm]')
title('Posição')

ylim_auto = ylim;
ylim([min(ylim_auto(1), -pos_limite*100*1.1), ...
      max(ylim_auto(2),  pos_limite*100*1.1)])

% -------- VELOCIDADE --------
subplot(2,2,3)
plot(tempo, velocidade, 'LineWidth', 1.5)
grid on
ylabel('Vel. [cm/s]')
title('Velocidade')

ylim_auto = ylim;
ylim([-50, 50])

% -------- ÂNGULO --------
subplot(2,2,2)
plot(tempo, angulo, 'LineWidth', 1.5)
hold on
%yline(180, '--k') % equilíbrio
grid on
ylabel('Ângulo [°]')
xlabel('Tempo [s]')
title('Ângulo')

%ylim_auto = ylim;
ylim([160, 200])

% -------- VELOCIDADE ANGULAR --------
subplot(2,2,4)
plot(tempo, vel_angular, 'LineWidth', 1.5)
grid on
ylabel('Vel. Ang. [°/s]')
xlabel('Tempo [s]')
title('Velocidade Angular')

%% ================= CONTROLE + TEMPO =================
figure('Name','Controle e Tempo de Execução','Color','w')

% -------- SINAL DE CONTROLE --------
subplot(2,1,1)
plot(tempo, comando, 'LineWidth', 1.5)
hold on
yline(comando_limite, '--r')
yline(-comando_limite, '--r')
grid on
ylabel('Tensão [V]')
title('Sinal de Controle')

ylim_auto = ylim;
ylim([min(ylim_auto(1), -comando_limite*1.1), ...
      max(ylim_auto(2),  comando_limite*1.1)])

% -------- TEMPO POR ITERAÇÃO --------
subplot(2,1,2)
plot(tempo, tempos_mpc(1:nt-MPC.N)*1000, 'LineWidth', 1.5) % em ms
grid on
xlabel('Tempo [s]')
ylabel('Tempo [ms]')
title('Tempo de Execução por Iteração')

% (opcional) linha do tempo de amostragem
hold on
Ts = 0.01; % <-- ajuste para o seu caso
yline(Ts*1000, '--r', 'Tempo de Amostragem')

clear ang_inicial A B Bineq err exitflag;
clear F i nt nu num_var_reguladas options pos_inicial qp_error_count QP tau tsim u usar_MPC;
clear x0 x_des xplus yref_pred p_opt lest lesu yref lesx lesy;
