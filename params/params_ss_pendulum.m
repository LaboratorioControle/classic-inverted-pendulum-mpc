%% Geração das matrizes de espaço de estados

% Estados x1=x, x2=theta, x3=x_ponto, x4=theta_ponto
g = dados.geral.g;
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
b = dados.pendulo.b;
M = dados.carro.m;
c = dados.carro.c;
Rm = dados.motor.Rm;
Kb = dados.motor.Kb;
Kt = dados.motor.Kt;
r = dados.motor.R;
tau = dados.geral.Ts;

alpha = (I + m*l^2)*(M + m) - (m*l)^2;

% Matrizes de estado para utilizar o sinal de comando Tensão [V]
Ac = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (m^2*l^2*g)/alpha, -(I + m*l^2)*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*m*l/alpha;
     0, m*g*l*(M + m)/alpha, -m*l*(c + (Kt*Kb)/(Rm*r^2))/alpha, -b*(M + m)/alpha];

Bc = [0;
     0;
    (I + m*l^2)*Kt/(alpha*Rm*r);
     m*l*Kt/(alpha*Rm*r)];

C = [1 0 0 0
     0 1 0 0];

D = 0;

dados.planta.Ac = Ac;
dados.planta.Bc = Bc;
dados.planta.C = C;
dados.planta.D = D;

[A, B] = c2d(Ac,Bc,tau);

dados.planta.A = A;
dados.planta.B = B;

clear g m l I b M c Rm Kb Kt r alpha A B C D Ac Bc tau;