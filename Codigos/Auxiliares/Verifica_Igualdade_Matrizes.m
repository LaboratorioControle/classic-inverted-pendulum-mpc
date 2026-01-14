%% ==============================
% IMPORTAR MATRIZ DO CSV
%% ==============================

close all;

% Nome do arquivo CSV
H = "ESP32\MPC_Inverted_Pendulum\lib\MPC\H.csv";

% Ler matriz do arquivo
matriz_csv = readmatrix(arquivo);

%% ==============================
% MATRIZ LOCAL NO MATLAB
%% ==============================

matriz_local = MPC.G1;

%% ==============================
% COMPARAÇÃO DAS MATRIZES
%% ==============================

% 1) Verificar se possuem o mesmo tamanho
if ~isequal(size(matriz_csv), size(matriz_local))
    error('As matrizes têm dimensões diferentes!');
end

% 2) Comparação exata (inteiros ou valores exatamente iguais)
iguais_exato = isequal(matriz_csv, matriz_local);

% 3) Comparação com tolerância (para valores em ponto flutuante)
tolerancia = 1e-6;
iguais_tol = all(abs(matriz_csv - matriz_local) < tolerancia, 'all');

%% ==============================
% RESULTADOS
%% ==============================

if iguais_exato
    disp('✅ Matrizes são exatamente iguais.');
elseif iguais_tol
    disp('⚠️ Matrizes são iguais dentro da tolerância.');
else
    disp('❌ Matrizes são diferentes.');

    % Mostrar onde são diferentes
    [lin, col] = find(abs(matriz_csv - matriz_local) >= tolerancia);

    fprintf('Diferenças encontradas nas posições:\n');
    for i = 1:length(lin)
        fprintf('(%d,%d): CSV = %.6f | LOCAL = %.6f\n', ...
                 lin(i), col(i), matriz_csv(lin(i),col(i)), matriz_local(lin(i),col(i)));
    end
end
