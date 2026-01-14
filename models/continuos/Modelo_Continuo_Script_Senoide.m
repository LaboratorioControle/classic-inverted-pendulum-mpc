function vet_x_dot = Modelo_Continuo_Script_Senoide(t, x, dados, t_senoide, amplitude,frequencia)

if t < t_senoide
    u = amplitude * sin(2*pi*frequencia*t);
else
    u = 0;
end

% Variáveis necessárias para encontrar x_2dot e theta_2dot
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
b = dados.pendulo.b;

M = dados.carro.m;
c = dados.carro.c;

g = dados.geral.g;

% dados do atrito seco
Fc = dados.carro.Fc;

% Relembrando que vet_x_dot = [x_dot, theta_dot, x_2dot, theta_2dot] e que o
% vetor de estados é x = [x, theta, x_dot, theta_dot]
theta = x(2);
x_dot = x(3);
theta_dot = x(4);

% Para reduzir o número de termos, utilizou-se a Força (N) como sinal de
% controle
F = Volt2Force(u, x_dot, dados.motor);

% Cálculo do atrito seco
k_aprox = 100; % Ajuste este valor se necessário (quanto maior, mais abrupto)
F_fric = Fc * tanh(k_aprox * x_dot);

alpha_a = ((m^2)*(l^2)*((sin(theta))^2)+ M*m*l^2 +(M+m)*I);

x_2dot  = (b*m*l*theta_dot*cos(theta) + (m^2)*(l^2)*g*sin(theta)*cos(theta) ...
    + (I + m*(l^2))*(F - c*x_dot - F_fric + m*l*sin(theta)*theta_dot^2) )/alpha_a;

theta_2dot = -((F - c*x_dot - F_fric)*m*l*cos(theta) + (m^2)*(l^2)*(theta_dot^2) ...
    *sin(theta)*cos(theta)+ (M+m)*(b*theta_dot + m*g*l*sin(theta)))/alpha_a;


vet_x_dot = [x_dot theta_dot x_2dot theta_2dot];
vet_x_dot = vet_x_dot';
end