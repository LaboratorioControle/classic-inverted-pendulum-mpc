function volt_out = Force2Volt(F,x_dot,motor)

% Variáveis auxiliares
kt = motor.Kt;
kb = motor.Kb;
Rm = motor.Rm;
r = motor.R;

volt_out = (F*Rm*r^2 + kt*kb*x_dot)/(kt*r);
end