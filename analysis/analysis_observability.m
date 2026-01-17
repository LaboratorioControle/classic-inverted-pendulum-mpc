%% Observabilidade do sistema
%
% Verifica se todos os estados do sistema
% podem ser reconstruídos a partir das saídas.

matriz_observabilidade = obsv(dados.planta.A, dados.planta.C);
Rank = rank(matriz_observabilidade);

fprintf("O posto da matriz de observabilidade é: ");
disp(Rank);

if Rank == size(dados.planta.A,1)
    fprintf("O sistema é completamente observável.\n");
else
    fprintf("O sistema NÃO é completamente observável.\n");
end

clear matriz_observabilidade Rank;
