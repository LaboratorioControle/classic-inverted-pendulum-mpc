function force_out = Volt2Force(v,x_dot,motor)

% Variáveis auxiliares
kt = motor.Kt;
kb = motor.Kb;
Rm = motor.Rm;
r = motor.R;

force_out = (kt*v*r - kt*kb*x_dot)/(Rm*(r^2));
end