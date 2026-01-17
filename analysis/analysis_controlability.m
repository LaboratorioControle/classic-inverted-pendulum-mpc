%% Controlabilidade do sistema
%
% Verifica se todos os estados do sistema
% podem ser controlados a partir da entrada.

matriz_controlabilidade = ctrb(dados.planta.A, dados.planta.B);
Rank = rank(matriz_controlabilidade);

fprintf("O posto da matriz de controlabilidade é: ");
disp(Rank);

if Rank == size(dados.planta.A,1)
    fprintf("O sistema é completamente controlável.\n");
else
    fprintf("O sistema NÃO é completamente controlável.\n");
end

clear matriz_controlabilidade Rank;
