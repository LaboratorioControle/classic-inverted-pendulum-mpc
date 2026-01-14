%% 01 - Carregamento dos dados

clear; close all; clc;

file = readtable('ensaio.xlsx');
idf.t = file.tempo;
idf.u = file.entrada;
idf.y = file.rpm;

file = readtable('validacao.xlsx');
vld.t = file.tempo;
vld.u = file.entrada;
vld.y = file.rpm;

Ts = 0.01;

%% 02 - Filtragem dos dados

fy = 20; % Janela do filtro de saída
fu = 10; % Janela do filtro de entrada

idf.uf = movmean(idf.u,fu);
idf.yf = movmean(idf.y,fy);
vld.uf = movmean(vld.u,fu);
vld.yf = movmean(vld.y,fy);

%% 03 - Offset dos dados

% Definição dos offsets
idf_uo = 0;
idf_yo = idf.yf(1);
vld_uo = 0;
vld_yo = vld.yf(1);

% Valores calculados a partir do offset
idf.uo = idf.uf - idf_uo;
idf.yo = idf.yf - idf_yo;
vld.uo = vld.uf - vld_uo;
vld.yo = vld.yf - vld_yo;

%% 04 - Prepara dados identificação e validação

id_ident = iddata(idf.yf, idf.uf,Ts);
id_valid = iddata(vld.yf, vld.uf,Ts);

%% Identificação do modelo

Opt = tfestOptions('InitMethod','svf');

% Número de polos, zeros e atraso
np = 2; 
nz = 0;
atraso = 0;

%G = tfest(id_ident,np,nz,atraso, Opt)
G = tfest(id_ident,np,nz,atraso, Opt,'Ts',Ts);
% Outras opções
% arx, armax, bj, ssest
% Discreto: tfest(id_ident,np,nz,atraso, Opt,'Ts',Ts)
%subplot(2,1,1);
%compare(id_ident, G);
%title('Comparação - Função G(s) com os dados de identificação');
%subplot(2,1,2);
%compare(id_valid, G);
%title('Comparação - Função G(s) com os dados de validação');
%figure;
%% Plotar dados

subplot(2,2,1);
plot(idf.t,idf.u);hold on;
plot(idf.t,idf.uf, 'LineWidth',2);
%plot(idf.t,idf.uo, 'LineWidth',2);
title('Dados da Entrada - Identificação');

subplot(2,2,2);
plot(idf.t,idf.y);hold on;
plot(idf.t,idf.yf, 'LineWidth',2);
%plot(idf.t,idf.yo, 'LineWidth',2);
title('Dados da Saída - Identificação');

subplot(2,2,3);
plot(vld.t,vld.u);hold on;
plot(vld.t,vld.uf, 'LineWidth',2);
%plot(vld.t,vld.uo, 'LineWidth',2);
title('Dados da Entrada - Validação');

subplot(2,2,4);
plot(vld.t,vld.y);hold on;
plot(vld.t,vld.yf, 'LineWidth',2);
%plot(vld.t,vld.yo, 'LineWidth',2);
title('Dados da Saída - Validação');

%%

%step(G*168);
T = feedback(G,1);
step(250*T);

%% Controlador PID Digital

Kp=0.5;Ki=0.3;Kd=0;N=0;
Gc = pid(Kp,Ki,Kd,N,Ts,'Iformula', 'BackwardEuler');

%% Modelos discreto em malha fechada

Gmf1 = feedback(G,1);
Gmf2 = feedback(G*Gc,1);

%% Solução das Equações Diferença

time = 10/Ts; % Tempo de simulação
r = ones(1,time); % Degrau unitário

for k = 1:time
    switch k
        case 1
            y1(k) = 0; % Saída da planta em Malha Fechada sem Controlador
            y(k) = 0; % Saída da planta em Malha Fechada com Controlador
            e(k) = r(k)-y(k); % Erro
            u(k) = (Kp + Ki * Ts + Kd/Ts)*e(k); % Sinal de saída do controlador
        case 2
            y1(k) = 0;
            y(k) = 0;
            e(k) = r(k)-y(k);
            u(k) = u(k-1) + Kp*e(k) - Kp*e(k-1) + Ki*Ts*e(k) + (Kd/Ts) * (e(k) - 2*e(k-1));
        otherwise
            y1(k) = 0.6*y1(k-1) - 1.05*y1(k-2)+r(k-2);
            y(k) = 0.6*y(k-1) - 0.05*y(k-2)+u(k-2);
            e(k) = r(k)-y(k);
            u(k) = u(k-1) + Kp*e(k) - Kp*e(k-1) + Ki*Ts*e(k) + (Kd/Ts) * (e(k) - 2*e(k-1) + e(k-2));
    end
end

%%

subplot(2,1,1);
step(Gmf1*250,10);
title("Sistema Malha Fechada sem controlador");
legend('Solução comando Step');

subplot(2,1,2);
step(Gmf2*250,10);
title("Sistema Malha Fechada com controlador");
legend('Solução comando Step');

%% Plot dos gráficos
subplot(3,1,1);
step(Gmf1,10); hold on; stairs([0:(time-1)]*Ts,y1,'rx');
title("Sistema Malha Fechada sem controlador");
legend('Solução comando Step', 'Solução Equação Diferença');

subplot(3,1,2);
step(Gmf2,10); hold on; stairs([0:(time-1)]*Ts,y,'rx');
title("Sistema Malha Fechada com controlador");
legend('Solução comando Step', 'Solução Equação Diferença');

subplot(3,1,3);
stairs(out.saida_simulink.time,out.saida_simulink.signals.values); hold on; stairs([0:(time-1)]*Ts,y,'rx');
title("Saída Simulink PID Discreto");
legend('Saída Simulink', 'Solução Equação Diferença');