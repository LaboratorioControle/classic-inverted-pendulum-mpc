function vet_x_dot = Modelo_Continuo_Script_Alamir(t, x, dados, t_pulso, amplitude_degrau)

if t < t_pulso
    u = amplitude_degrau;
else
    u = 0;
end

% Variáveis necessárias para encontrar r_2dot e theta_2dot
m0 = dados.carro.m;
m1 = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
g = dados.geral.g;

alpha1 = (m1*l)/(m1*l^2+I); beta1 = g*alpha1;
alpha0 = m0+m1-alpha1*m1*l; beta0 = m1*l*beta1;

% Relembrando que vet_x_dot = [theta_dot, theta_2dot, r_dot, r_2dot] e que o
% vetor de estados é x = [theta, theta_dot, r, r_dot]
theta = x(1);
r_dot = x(4);
theta_dot = x(2);

% Para reduzir o número de termos, utilizou-se a Força (N) como sinal de
% controle
u = Volt2Force(u, r_dot, dados.motor);

theta_2dot = -alpha1/alpha0*u + (((alpha1*beta0)/alpha0)+beta1)*theta;
r_2dot = 1/alpha0*(u - beta0*theta);

vet_x_dot = [theta_dot theta_2dot r_dot r_2dot];
vet_x_dot = vet_x_dot';

end