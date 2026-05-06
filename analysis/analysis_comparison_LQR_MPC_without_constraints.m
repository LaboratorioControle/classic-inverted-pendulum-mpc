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
%MPC.Cr = [1 0 0 0; 0 1 0 0];
MPC.Cr = eye(4);
%% Matrizes de ponderação do MPC
% Utiliza as mesmas matrizes Q e R do LQR para garantir
% uma comparação justa entre os controladores
MPC.Qy = dados.controlador.lqr.Q;   % Penalização dos estados rastreados
MPC.Qu = dados.controlador.lqr.R;   % Penalização do esforço de controle

%MPC.Qy = [500 0; 100 0];

%% Dimensões do sistema
[n, nu] = size(B);

% Modelo do sistema para o MPC
MPC.A = A;
MPC.B = B;

%% Cálculo dos ganhos do MPC para diferentes horizontes
Nmax = 100;               % Horizonte máximo analisado
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
%K_LQR = [-15, 140, -80, 20];
%% Plot dos resultados
% Comparação dos ganhos do MPC (em função de N)
% com os ganhos constantes do LQR



tol = 0.7;

id_conv = zeros(1,4);

for j = 1:4
    diff = abs(K(:,1) - K_LQR(1));
    idx = find(diff < tol, 1, 'first');

    if ~isempty(idx)
        id_conv(j) = idx;
    else
        id_conv(j) = NaN;
    end
end


figure;
set(gcf, 'Units', 'centimeters', 'Position', [5 5 20 15])

for i = 1:4
    subplot(2,2,i);
    hold on;
    
    plot(lesN, K(:,i), 'LineWidth', 1.5);
    plot(lesN, K_LQR(i)*ones(size(lesN)), 'k--', 'LineWidth', 1.5);
    
    % Ponto de convergência
    if ~isnan(id_conv(i))
        plot(lesN(id_conv(i)), K(id_conv(i),i), 'o', 'MarkerSize', 6, 'LineWidth', 1.5);
        
        txt = sprintf('N = %d', id_conv(i));
    else
        txt = 'N não converge';
    end
    
    grid on;
    
    title(labels{i});
    xlabel('Horizonte N');
    ylabel('Ganho');
    
    if i == 1
        legend({'$K_{MPC}$', '$K_{LQR}$', txt}, ...
            'Interpreter', 'latex', 'Location', 'best');
    end
end
