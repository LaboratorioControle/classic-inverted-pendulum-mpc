function force_out = Volt2Force(v, x_dot, motor)
%VOLT2FORCE Converte tensão elétrica em força mecânica
%
%   force_out = VOLT2FORCE(v, x_dot, motor)
%
%   Esta função calcula a força mecânica aplicada ao carro
%   a partir da tensão aplicada ao motor DC, considerando
%   os efeitos da força contra-eletromotriz e os parâmetros
%   eletromecânicos do motor.
%
%   Entradas:
%       v      - Tensão aplicada ao motor [V]
%       x_dot  - Velocidade linear do carro [m/s]
%       motor  - Estrutura contendo os parâmetros do motor:
%                .Kt  - Constante de torque [N·m/A]
%                .Kb  - Constante de força contra-eletromotriz [V·s/rad]
%                .Rm  - Resistência do enrolamento [Ω]
%                .R   - Raio da polia ou roda [m]
%
%   Saída:
%       force_out - Força aplicada ao carro [N]

%% Parâmetros do motor

kt = motor.Kt;   % Constante de torque
kb = motor.Kb;   % Constante de FEM
Rm = motor.Rm;   % Resistência elétrica
r  = motor.R;    % Raio da polia/roda

%% Conversão de tensão para força

force_out = (kt * v * r - kt * kb * x_dot) / (Rm * r^2);

end
