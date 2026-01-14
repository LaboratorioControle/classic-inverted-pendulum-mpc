%% Modelo do sistema do pêndulo invertido

clear; clc; close all;

%% Declaração das variáveis do sistema

% % ==============================
% % Dados gerais
% % ==============================
% dados.geral.g = 9.81;          % [m/s^2] Aceleração da gravidade
% dados.geral.tau = 10/1000;       % [s] Período de amostragem
% dados.geral.Tf = 10;            % [s] Tempo total da simulação
% dados.geral.spt = -10;         % [cm] Posição desejada do carro
% 
% % Condições Iniciais (pêndulo para baixo)
% dados.geral.inicial.theta0 = 10;      % [°] Ângulo inicial
% dados.geral.inicial.theta_dot0 = 0;   % [°/s] Velocidade angular inicial
% dados.geral.inicial.x0 = 0;           % [m] Posição inicial do carro
% dados.geral.inicial.x_dot0 = 0;       % [m/s] Velocidade inicial do carro
% 
% % ==============================
% % Dados do pêndulo
% % ==============================
% dados.pendulo.m = 55.5/1000;        % [kg]    Massa da haste do pêndulo
% dados.pendulo.l = 0.45/2;        % [m]     Distância do ponto de fixação até o centro de gravidade
% dados.pendulo.I = (1/12)*dados.pendulo.m*(dados.pendulo.l*2)^2;  % [kg·m^2] Momento de inércia da haste em relação ao centro
% dados.pendulo.b = 0.00001;   % [N·m·s] Coeficiente de amortecimento viscoso no eixo de fixação
% dados.pendulo.r_massa = 0.01; % [m] Raio da massa na ponta do pêndulo 
% 
% % ==============================
% % Dados do carro
% % ==============================
% dados.carro.m = 308.82/1000;         % [kg]    Massa do carro
% dados.carro.c = 0.002;          % [N·s/m] Coeficiente de amortecimento viscoso entre carro e guia
% dados.carro.l = 0.2;           % [m] Largura do carro
% dados.carro.h = 0.1;           % [m] Altura do carro
% 
% % ==============================
% % Dados do motor
% % ==============================
% dados.motor.J  = 3.26*10^(-8);         % [kg·m^2] Inércia do rotor
% dados.motor.Rm = 12.5;         % [Ω]      Resistência do enrolamento
% dados.motor.Kb = 0.031;         % [V·s/rad] Constante de força contra-eletromotriz
% dados.motor.Kt = 0.031;         % [N·m/A]  Constante de torque
% dados.motor.R  = 0.006;         % [-]      Relação de transmissão (se houver)

dados.geral.g = 9.81;
dados.pendulo.m = 0.2;        % [kg]    Massa da haste do pêndulo
dados.pendulo.l = 1;        % [m]     Distância do ponto de fixação até o centro de gravidade
dados.pendulo.I = 0.1;  % [kg·m^2] Momento de inércia da haste em relação ao centro 
dados.carro.m = 2;         % [kg]    Massa do carro
dados.geral.tau = 0.15;

%% Modelo da Planta

tau = dados.geral.tau;

m0 = dados.carro.m;
m1 = dados.pendulo.m;
l = dados.pendulo.l;
I = dados.pendulo.I;
g = dados.geral.g;


alpha1 = (m1*l)/(m1*l^2+I); beta1 = g*alpha1;
alpha0 = m0+m1-alpha1*m1*l; beta0 = m1*l*beta1;

% x = [theta theta_dot r r_dot]

Ac = [0 1 0 0; ((alpha1*beta0)/alpha0)+beta1 0 0 0; 0 0 0 1
                  -beta0/alpha0 0 0 0];

Bc = [0; -alpha1/alpha0; 0; 1/alpha0];

[A, B] = c2d(Ac,Bc,tau);

C = [0 0 0 1];

dados.planta.Ac = Ac; dados.planta.Bc = Bc; 
dados.planta.A = A; dados.planta.B = B; dados.planta.C = C;

clear alpha0 alpha1 beta0 beta1;
clear A Ac B Bc C g l I m0 m1 tau;

%% Controlador Preditivo

dados.controlador.Q = diag([0; 0; 1; 0]);
dados.controlador.R = exp(-3);

planta = dados.planta;
controlador = dados.controlador;

umax=1;
param.dt=0.15;param.N=10;np=param.N;
param.pmin=-umax*ones(np,1);param.pmax=umax*ones(np,1);
alpha=1*ones(np,1);param.p=zeros(np,1);
%--------------------------------------------
mv=create_mv(param.p,param.pmin,param.pmax,alpha);
mv(1).beta_plus=1.2;mv(1).beta_moins=0.5;
param.ell=1;param.rmax=0.5;mode=1;epsilon=0.00001;
%--------------------------------------------
[K, S, G] = compute_KSG(param.dt,25,planta,controlador);
param.Nev=2000;param.Niter=1;
subset = 1:3;
tsim=7;Nsim=fix(tsim/param.dt)+1;
tt=(0:1:Nsim-1)'*param.dt;
xx=zeros(4,Nsim);uu=zeros(1,Nsim);
EE=zeros(Nsim,1);SS=EE;
texec=zeros(Nsim,1);
% t1 t2 r t1d t2d rd
param.x0=[pi/6;0;0;0];xx(:,1)=param.x0;
for i=1:Nsim-1
    z=xx(:,i);
    z(1)=truncate_theta(z(1));
    SS(i)=z'*S*z;
    cond2=(max(xx(3,i)+G*z)<0.8);
    param.x0=xx(:,i);
    disp(SS(i));
    if (((SS(i)<epsilon)&&(cond2))||(mode==2))
        uu(i)=-K*z;mode=2;
    else
        %t0=clock;
        psol=update_mv(mv, subset,param, dados);
        %texec(i)=etime(clock,t0);
        uu(i)=psol(1);
    end
    xx(:,i+1)=pendulum_one_step(xx(:,i)',uu(i),param.dt, dados);
    EE(i)=compute_E(xx(:,i), dados);
end

%%

simulacao.tempo = tt;
simulacao.angulo = (180/pi) * xx(1,:);
simulacao.velocidade_pendulo = xx(2,:);
simulacao.posicao = xx(3,:);
simulacao.velocidade_carro = xx(4,:);
simulacao.u_force = uu;

%clear k u_force u_volt xx vet_tempo_simu;

%% Plot dos dados

subplot(2,2,1);
stairs(simulacao.tempo, simulacao.angulo,'LineWidth',2); grid on;
title('Posição Angular do Pêndulo [°]');

subplot(2,2,2);
stairs(simulacao.tempo, simulacao.velocidade_pendulo,'LineWidth',2); grid on;
title('Velocidade Angular do Pêndulo [rad/s]');

subplot(2,2,3);
stairs(simulacao.tempo, simulacao.posicao,'LineWidth',2); grid on;
title('Posição Linear do Carro [cm]');

subplot(2,2,4);
stairs(simulacao.tempo, simulacao.velocidade_carro,'LineWidth',2); grid on;
title('Velocidade Linear do Carro [m/s]');

