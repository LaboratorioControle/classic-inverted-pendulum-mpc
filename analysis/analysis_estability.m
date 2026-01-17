%% Estabilidade do sistema
%
% Analisa os polos do sistema em tempo contínuo.
% Caso exista algum polo no semi-plano direito,
% o sistema é considerado instável.

Poles = eig(dados.planta.A);

fprintf("O sistema possui os seguintes polos:\n");
disp(Poles);

if any(real(Poles) > 0)
    fprintf("Nota-se que o sistema é instável, pois possui polos no semi-plano direito.\n");
else
    fprintf("O sistema é estável, pois todos os polos estão no semi-plano esquerdo.\n");
end

clear Poles;
