function volt_out = Force2Volt(F, x_dot, motor)
%FORCE2VOLT Converte força mecânica em tensão elétrica
%
%   volt_out = FORCE2VOLT(F, x_dot, motor)
%
%   Esta função calcula a tensão necessária no motor DC
%   para gerar uma força F aplicada ao carro, considerando
%   os parâmetros eletromecânicos do motor e a velocidade
%   linear do sistema.
%
%   Entradas:
%       F      - Força aplicada ao carro [N]
%       x_dot  - Velocidade linear do carro [m/s]
%       motor  - Estrutura contendo os parâmetros do motor:
%                .Kt  - Constante de torque [N·m/A]
%                .Kb  - Constante de força contra-eletromotriz [V·s/rad]
%                .Rm  - Resistência do enrolamento [Ω]
%                .R   - Raio da polia ou roda [m]
%
%   Saída:
%       volt_out - Tensão aplicada ao motor [V]

%% Parâmetros do motor

kt = motor.Kt;   % Constante de torque
kb = motor.Kb;   % Constante de FEM
Rm = motor.Rm;   % Resistência elétrica
r  = motor.R;    % Raio da polia/roda

%% Conversão de força para tensão

volt_out = (F * Rm * r^2 + kt * kb * x_dot) / (kt * r);

end
