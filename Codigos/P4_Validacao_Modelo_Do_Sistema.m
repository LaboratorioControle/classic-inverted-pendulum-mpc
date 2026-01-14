clear; clc; close all;

% ==============================
% CONFIGURAÇÃO SERIAL
% ==============================
porta_serial = 'COM3'; % Verifique sua porta
baud_rate = 115200;
s = serialport(porta_serial, baud_rate);
configureTerminator(s, "LF");
flush(s);

% ==============================
% COMANDO PARA APLICAR DEGRAU
% ==============================
duracaoDegrau = 300;   % ms
intensidadeDegrau = 200; % PWM 0-255
sentidoDegrau = 'R';     % 'R' ou 'L'

% Monta comando no formato esperado pelo ESP: D,500,200,R
comando = sprintf('D,%d,%d,%s\n', duracaoDegrau, intensidadeDegrau, sentidoDegrau);
writeline(s, comando);  % envia comando

pause(0.01); % Pequena espera para garantir que o ESP recebeu o comando

% ==============================
% INICIALIZAÇÃO DOS VETORES
% ==============================

tempo_total = 15; % tempo total de aquisição (s)
periodo = 10/1000; % 10ms
t_inicio = tic;  % cronômetro interno

tam = tempo_total/periodo;

tempo = zeros(1,tam);
theta = zeros(1,tam);
theta_dot = zeros(1,tam);
pos = zeros(1,tam);
pos_dot = zeros(1,tam);

i = 1;

% ==============================
% LOOP DE LEITURA SERIAL
% ==============================
while toc(t_inicio) < tempo_total
    if s.NumBytesAvailable > 0
        raw_data = readline(s); % lê linha serial
        dados = strsplit(strtrim(raw_data), ';'); % divide por vírgula

        if length(dados) == 5
            tempo(i)  = str2double(dados{1});
            theta(i) = str2double(dados{2});
            theta_dot(i)  = str2double(dados{3});
            pos(i)  = str2double(dados{4});
            pos_dot(i)  = str2double(dados{5});
            i = i+1;
        end
    end
end

clear s; % fecha serial

% ==============================
% PLOT DOS DADOS
% ==============================
figure;
subplot(2,2,1);
plot(tempo, pos); grid on;
title('Posição Carro');
xlabel('Tempo (s)');
ylabel('Posição (pulsos)');

subplot(2,2,2);
plot(tempo, pos_dot); grid on;
title('Velocidade Carro');
xlabel('Tempo (s)');
ylabel('Velocidade (pulsos/s)');

subplot(2,2,3);
plot(tempo, theta); grid on;
title('Ângulo Pêndulo');
xlabel('Tempo (s)');
ylabel('Ângulo (pulsos)');

subplot(2,2,4);
plot(tempo, theta_dot); grid on;
title('Velocidade Angular Pêndulo');
xlabel('Tempo (s)');
ylabel('Velocidade (pulsos/s)');