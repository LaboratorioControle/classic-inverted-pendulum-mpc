function vet_x_dot = Modelo_Continuo(input, dados)

% Coleta do vetor de estados e da entrada
x = input(1:4);
u = input(5);

% Variáveis necessárias para encontrar x_2dot e theta_2dot
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
b = dados.pendulo.b;

M = dados.carro.m;
c = dados.carro.c;

g = dados.geral.g;

% Relembrando que vet_x_dot = [x_dot, theta_dot, x_2dot, theta_2dot] e que o
% vetor de estados é x = [x, theta, x_dot, theta_dot]
theta = x(2);
x_dot = x(3);
theta_dot = x(4);

% Para reduzir o número de termos, utilizou-se a Força (N) como sinal de
% controle


F = Volt2Force(u, x_dot, dados.motor);


alpha_a = ((m^2)*(l^2)*((sin(theta))^2)+ M*m*l^2 +(M+m)*I);

x_2dot  = (b*m*l*theta_dot*cos(theta) + (m^2)*(l^2)*g*sin(theta)*cos(theta) ...
    + (I + m*(l^2))*(F-c*x_dot+ m*l*sin(theta)*theta_dot^2) )/alpha_a;

theta_2dot = -(F*m*l*cos(theta)-c*m*l*x_dot*cos(theta) + (m^2)*(l^2)*(theta_dot^2) ...
    *sin(theta)*cos(theta)+ (M+m)*(b*theta_dot + m*g*l*sin(theta)))/alpha_a;


vet_x_dot = [x_dot theta_dot x_2dot theta_2dot];

end