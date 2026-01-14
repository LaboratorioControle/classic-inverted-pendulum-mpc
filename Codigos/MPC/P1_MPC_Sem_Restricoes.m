%% Controlador Preditivo Sem Restrições

run Codigos\P1_Modelo_Do_Sistema.m;
close all;

A = dados.planta.A;
B = dados.planta.B;
tau = dados.geral.Ts;


[n,nu]=size(B);
MPC.A = A; MPC.B = B;
MPC.Cr=eye(4);
MPC.Qy=diag([250 100 1 1]);
MPC.Qu=0.01;
MPC.N=105;

%--------------------------------------------
[H,F1,F2,~]=compute_cost_matrices(MPC);
KN=P_i(1,nu,MPC.N)*(H\F1);
GN=-P_i(1,nu,MPC.N)*(H\F2);
%--------------------------------------------
x0=[0;185*pi/180;0;0];
tsim=10;
lest=(0:tau:tsim)'; nt= size(lest,1);
lesx=zeros(nt,length(B)); lesy=zeros(nt,size(MPC.Cr,1)); lesu=zeros(nt,1);

yref = zeros(length(lest)*size(MPC.Cr,1),1);             % inicializa vetor coluna
yref(1:4:end) = 0/100;  % índices ímpares
yref(2:4:end) = pi;                   % índices pares

lesx(1,:) = x0';
lesy(1,:)=(MPC.Cr*lesx(1,:)')';

x_des = [0 180*pi/180 0 0];
u = 0;

sat = @(x, x_max, x_min) min( x_max, max(x_min,x));

for i=1:nt-MPC.N
    
    xplus = RK4_discrete(lesx(i,:),u,tau,dados);

    yref_pred=yref(i*size(MPC.Cr,1) + 1 : (i+MPC.N)*size(MPC.Cr,1));
    
    err = xplus-x_des;
    u=-KN*err';+GN*yref_pred;
    u = sat(u,12,-12);
    lesu(i,:)=u';
    

    lesx(i+1,:)=xplus;
    lesy(i+1,:)=MPC.Cr*xplus';
end

controlador.MPC = MPC;

posicao = lesx(1:nt-MPC.N,1).*100;
angulo = lesx(1:nt-MPC.N,2).*180/pi;
velocidade = lesx(1:nt-MPC.N,3).*100;
vel_angular = lesx(1:nt-MPC.N,4).*180/pi;
lest = lest(1:nt-MPC.N);
lesu = lesu(1:nt-MPC.N);

%% Plot dos resultados

figure;
stairs(lest,lesu); grid on;
title('Sinal de Comando');

figure;
% 1. Posição do Carrinho
subplot(2, 2, 1);
stairs(lest, posicao, 'k-');
title('Posição do Carrinho');
xlabel('Tempo (s)');
ylabel('Posição (cm)');
grid on;
ylim([-30 30]);

% 2. Ângulo do Pêndulo
subplot(2, 2, 2);
stairs(lest, angulo, 'k-');
title('Ângulo do Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

% 3. Velocidade do Carrinho
subplot(2, 2, 3);
stairs(lest, velocidade, 'k-');
title('Velocidade do Carrinho');
xlabel('Tempo (s)');
ylabel('Velocidade (cm/s)');
grid on;
ylim([-50 50]);

% 4. Velocidade Angular do Pêndulo
subplot(2, 2, 4);
stairs(lest, vel_angular, 'k-');
title('Velocidade Angular');
xlabel('Tempo (s)');
ylabel('Velocidade Ang. (°/s)');
grid on;
ylim([-70 70]);

%clear A B F1 F2 F3 GN H i KN lest lesu lesx lesy MPC n nt nu tau tsim var_rastreadas x0 yref yref_pred;
