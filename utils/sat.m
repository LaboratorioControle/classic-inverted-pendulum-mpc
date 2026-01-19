function u_sat = sat(u, u_max, u_min)
%SAT Função de saturação
%
%   u_sat = SAT(u, u_max, u_min)
%
%   Limita o sinal de entrada u ao intervalo [u_min, u_max].
%
%   Entradas:
%       u     - Sinal a ser saturado
%       u_max - Limite superior
%       u_min - Limite inferior
%
%   Saída:
%       u_sat - Sinal saturado

u_sat = min(u_max, max(u_min, u));

end
