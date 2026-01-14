%% Otimização PID

%% Ponto Inicial dos Parâmetros do Controlador

Kp_ini = 0.1;Ki_ini = 0.1;Kd_ini = 0.1;
x0 = [Kp_ini Ki_ini Kd_ini];

% Limites dos Parâmetros

Kp_max = 0.5; Ki_max = 0.5; Kd_max = 0.2;
Kp_min = 0; Ki_min = 0; Kd_min = 0;

v_max = [Kp_max Ki_max Kd_max];
v_min = [Kp_min Ki_min Kd_min];

% Especificação desejada

Tsettle = 0.3;

% Tempo amostragem e simulação

Ts = 0.01;
Tsim = 1;
tempo = 0:Ts:Tsim-Ts;
% Degrau unitário

u = ones(length(tempo),1);

% Saída de referência

s = tf('s');
tau = Tsettle/4;

Gref = 1/(tau*s + 1);
yref = lsim(Gref,u,tempo);

% Modelo da planta Discreta

z = tf('z',Ts);
Gz = 1/((z-0.1)*(z-0.5));

% Definição do problema de otimização

options = optimset('Display','iter','MaxFunEvals',500);
[p,fval] = fmincon(@controle.funcao_custo, x0,[],[],[],[], v_min,v_max,[],options,yref,Ts,Tsim,Gz);

%% Simulação com os parâmetros ótimos

Gc = pid(p(1),p(2),p(3),0,Ts,'IFormula','BackwardEuler');
yopt = step(feedback(Gc*Gz,1), tempo);

%% Plot dos Resultados

plot(tempo, yopt, tempo, yref);
legend('Sintonia PID Otimizado','Referência');
title('Resposta ao Degrau Filtrado - PID');