%% Controlador LQR + Swing up

run Codigos\P1_Modelo_Do_Sistema.m;

% Definição das matrizes de ponderamento do LQR
dados.controlador.lqr.Q = diag([1 5 1 1]); dados.controlador.lqr.R = 0.001;
dados.controlador.lqr.K = dlqr(dados.planta.A,dados.planta.B,dados.controlador.lqr.Q,dados.controlador.lqr.R);

%% Controlador baseado na lei de Energia

dados.controlador.energia.k = 44;
dados.controlador.energia.n = 1;

%% Simulação em malha fechada

% Parâmetros do sistema
m = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
g = dados.geral.g;

Ts = dados.geral.Ts;
Tf = dados.geral.Tf; 
k_lqr = dados.controlador.lqr.K;

theta_switch = 6*pi/180;   
theta_dot_switch = 20*pi/180; 

% Pré-alocação
N = round(Tf/Ts);
t        = zeros(1, N+1);
u_volt   = zeros(1, N+1);
u_force  = zeros(1, N+1);
x        = zeros(N+1, 4);
E        = zeros(1, N+1);


x(1,:) = [0 1*pi/180 0 0]; % Estado Inicial
u_volt(1) = 0;
u_force(1) = 0;
t(1) = 0;
E(1) = 0;

x_des = [0/100; 180*pi/180; 0; 0];            % Estado desejado

i = 1;
sat = @(x, x_max, x_min) min( x_max, max(x_min,x));

for k = 0:Ts:Tf
    t(i+1) = k+Ts;

    estado_novo = RK4_discrete(x(i,:),u_volt(i),Ts,dados)';

    % Aplicação de um distúrbio aos 20s
    if abs(k - 20) < Ts/2
        estado_novo(4) = estado_novo(4) + 20*pi/180; 
    end

    x(i+1,:) = estado_novo';

    theta = estado_novo(2);
    theta_dot = estado_novo(4);

    if (abs(theta-pi) < theta_switch) && (abs(theta_dot) < theta_dot_switch)
        u_volt(i+1) = -k_lqr * (estado_novo - x_des);
    else
        u_volt(i+1) = EnergySwingUpController(estado_novo, dados);
    end

    u_volt(i+1) = sat(u_volt(i+1),12,-12);
    u_force(i+1) = Volt2Force(u_volt(i+1),estado_novo(3),dados.motor);

    E(i+1) = m*g*l*(1 - cos(theta)) + 0.5*(I + m*l^2)*(theta_dot^2);

    i = i+1;
end


simulacao.tempo = t;
simulacao.angulo = wrapTo360( (180/pi).* x(:,2) );
%simulacao.angulo = (180/pi).* x(:,2);
simulacao.velocidade_pendulo = (180/pi).*x(:,4);
simulacao.posicao = x(:,1).*100;
simulacao.velocidade_carro = x(:,3).*100;
simulacao.u_force = u_force;
simulacao.u_volt = u_volt;
simulacao.energia = E;

clear k u_force u_volt i estado_novo t Tf Ts x x_des sat b c dead_zone g I l ke m M theta theta_dot theta_dot_switch theta_lqr x_dot x_pos theta_switch;

%% Plot dos dados

figure;
subplot(2,1,1);
stairs(simulacao.tempo, simulacao.angulo,'LineWidth',2); grid on;
title('Ângulo [°]');
xlabel('Tempo [s]');

subplot(2,1,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular [°/s]');
xlabel('Tempo [s]');

figure;
subplot(2,1,1);
stairs(simulacao.tempo, simulacao.posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [cm]');
xlabel('Tempo [s]');

subplot(2,1,2);
stairs(simulacao.tempo, simulacao.velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [cm/s]');
xlabel('Tempo [s]');

figure;

subplot(1,2,1);
stairs(simulacao.tempo,simulacao.u_force,'LineWidth',2); grid on;
title('Comando - Força [F]');

subplot(1,2,2);
stairs(simulacao.tempo,simulacao.u_volt,'LineWidth',2); grid on;
title('Comando - Tensão [V]');

figure;

stairs(simulacao.tempo,simulacao.energia,'LineWidth',2); grid on;
title('Energia do Sistema');