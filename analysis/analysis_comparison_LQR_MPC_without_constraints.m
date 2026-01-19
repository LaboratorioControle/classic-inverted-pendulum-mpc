%% Comparação - LQR x MPC Sem Restrições
% Este script realiza a comparação entre os ganhos obtidos pelo
% controlador LQR clássico e pelo controlador MPC sem restrições,
% avaliando a convergência dos ganhos do MPC em função do horizonte N.

%% Execução do controlador LQR
% Carrega os parâmetros do sistema e do controlador LQR
run controller_lqr.m

%% Matrizes do sistema
A = dados.planta.A;   % Matriz de estados
B = dados.planta.B;   % Matriz de entrada

%% Estrutura de saída do MPC
% Considera todos os estados como saídas rastreadas
MPC.Cr = eye(4);

%% Matrizes de ponderação do MPC
% Utiliza as mesmas matrizes Q e R do LQR para garantir
% uma comparação justa entre os controladores
MPC.Qy = dados.controlador.lqr.Q;   % Penalização dos estados rastreados
MPC.Qu = dados.controlador.lqr.R;   % Penalização do esforço de controle

%% Dimensões do sistema
[n, nu] = size(B);

% Modelo do sistema para o MPC
MPC.A = A;
MPC.B = B;

%% Cálculo dos ganhos do MPC para diferentes horizontes
Nmax = 200;               % Horizonte máximo analisado
K = zeros(Nmax, n);       % Vetor para armazenamento dos ganhos
lesN = (1:1:Nmax)';       % Vetor de horizontes de predição

for i = 1:Nmax
    % Define o horizonte de predição atual
    MPC.N = lesN(i);
    
    % Cálculo das matrizes de custo do MPC
    [H, F1, F2, F3] = compute_cost_matrices(MPC);
    
    % Cálculo do ganho equivalente do MPC
    K(i,:) = P_i(1, nu, i) * (H \ F1);
end

%% Ganho do controlador LQR
one = ones(size(lesN));               % Vetor unitário para replicação
K_LQR = dados.controlador.lqr.K;       % Ganho do LQR

%% Plot dos resultados
% Comparação dos ganhos do MPC (em função de N)
% com os ganhos constantes do LQR

figure;
sgtitle('Comparação LQR x MPC sem restrições');

% Ganho associado à posição do carrinho (x)
subplot(2,2,1);
plot(lesN, K(:,1), lesN, one*K_LQR(1), 'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da posição x');
xlabel('Horizonte de predição N');
ylabel('Ganho');

% Ganho associado ao ângulo do pêndulo (\theta)
subplot(2,2,2);
plot(lesN, K(:,2), lesN, one*K_LQR(2), 'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da posição \theta');
xlabel('Horizonte de predição N');
ylabel('Ganho');

% Ganho associado à velocidade do carrinho (ẋ)
subplot(2,2,3);
plot(lesN, K(:,3), lesN, one*K_LQR(3), 'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da velocidade x_{dot}');
xlabel('Horizonte de predição N');
ylabel('Ganho');

% Ganho associado à velocidade angular do pêndulo (\dot{\theta})
subplot(2,2,4);
plot(lesN, K(:,4), lesN, one*K_LQR(4), 'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da velocidade \theta_{dot}');
xlabel('Horizonte de predição N');
ylabel('Ganho');

clear ang_limite angulo A B comando comando_limite F1 F2 F3 H i K K_LQR lesN;
clear MPC n nu Nmax one pos_limite pos_spt posicao tempo velocidade vel_limite vel_angular;