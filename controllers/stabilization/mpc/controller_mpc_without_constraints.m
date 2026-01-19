%% Controlador Preditivo Sem Restrições

close all;
clc;

%% Parâmetros do sistema

A = dados.planta.A;      % Matriz de estados
B = dados.planta.B;      % Matriz de entrada
tau = dados.geral.Ts;    % Tempo de amostragem

% Dimensões do sistema
[~,nu] = size(B);        % n = número de estados, nu = número de entradas

%% Estrutura do controlador MPC

MPC.A  = A;              % Modelo do sistema
MPC.B  = B;              
MPC.Cr = eye(4);         % Matriz de saída (todos os estados observáveis)

% Matrizes de ponderação do custo
MPC.Qy = diag([10 5 1 1]);  % Penalização dos estados
MPC.Qu = 0.01;                % Penalização do esforço de controle

MPC.N  = 200;            % Horizonte de predição

%% Cálculo das matrizes do custo

[H, F1, F2, ~] = compute_cost_matrices(MPC);

% Matrizes de ganho do MPC
KN = P_i(1,nu,MPC.N) * (H \ F1);   % Ganho de realimentação de estados
GN = -P_i(1,nu,MPC.N) * (H \ F2);  % Ganho de rastreamento da referência

%% Condições iniciais e parâmetros de simulação
x0 = [0; 185*pi/180; 0; 0];  % Estado inicial do sistema
tsim = dados.geral.Tf;       % Tempo total de simulação (s)
pos_spt = 0;

% Vetor de tempo
lest = (0:tau:tsim)';
nt = size(lest,1);

% Inicialização dos vetores de armazenamento
lesx = zeros(nt, length(B));            % Estados
lesy = zeros(nt, size(MPC.Cr,1));       % Saídas
lesu = zeros(nt, 1);                    % Entrada de controle

%% Referência do sistema

yref = zeros(length(lest) * size(MPC.Cr,1), 1);

% Referência desejada:
yref(1:4:end) = pos_spt/100;

%% Inicialização dos estados

lesx(1,:) = x0';
lesy(1,:) = (MPC.Cr * lesx(1,:)')';

% Estado desejado
x_des = [0 180*pi/180 0 0];

% Entrada inicial
u = 0;

for i=1:nt-MPC.N
    
    % Integração numérica do sistema (RK4)
    xplus = RK4_discrete(lesx(i,:), u, tau, dados);

    % Referência prevista no horizonte
    yref_pred = yref(i*size(MPC.Cr,1) + 1 : (i+MPC.N)*size(MPC.Cr,1));
    
    % Erro em relação ao estado desejado
    err = xplus - x_des;

    u=-KN*err' + GN*yref_pred;
    u = sat(u,12,-12);
    lesu(i,:)=u';
    

    lesx(i+1,:)=xplus;
    lesy(i+1,:)=MPC.Cr*xplus';
end

dados.controlador.MPC.SemRestricoes = MPC;

%% Conversão de unidades para visualização
posicao       = lesx(1:nt-MPC.N,1) .* 100;        % m -> cm
angulo        = lesx(1:nt-MPC.N,2) .* 180/pi;     % rad -> graus
velocidade    = lesx(1:nt-MPC.N,3) .* 100;        % m/s -> cm/s
vel_angular   = lesx(1:nt-MPC.N,4) .* 180/pi;     % rad/s -> graus/s

% Ajuste dos vetores
lest = lest(1:nt-MPC.N);
lesu = lesu(1:nt-MPC.N);

%% Plot dos resultados

% Sinal de controle
figure;
stairs(lest, lesu); grid on;
title('Sinal de Comando');
xlabel('Tempo (s)');
ylabel('u');

% Estados do sistema
figure;

% 1. Posição do carrinho
subplot(2,2,1);
stairs(lest, posicao);
title('Posição do Carrinho');
xlabel('Tempo (s)');
ylabel('Posição (cm)');
grid on;

% 2. Ângulo do pêndulo
subplot(2,2,2);
stairs(lest, angulo);
title('Ângulo do Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

% 3. Velocidade do carrinho
subplot(2,2,3);
stairs(lest, velocidade);
title('Velocidade do Carrinho');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');
grid on;

% 4. Velocidade angular do pêndulo
subplot(2,2,4);
stairs(lest, vel_angular);
title('Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Velocidade Ang. (°/s)');
grid on;

% Limpeza opcional do workspace
 clear A B F1 F2 F3 GN H i KN lesx lesy MPC n nt nu tau tsim var_rastreadas x0 yref yref_pred;
 clear xplus x_des err u angulo lest lesu posicao vel_angular velocidade pos_spt;