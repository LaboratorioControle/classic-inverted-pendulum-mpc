%% Leitura em tempo real dos dados do ESP

clear all;
clc

% Definição das variáveis
PORTA = "COM5";  
BAUD  = 115200;

% Abertura da porta serial
s = serialport(PORTA, BAUD);
flush(s);

% Função de coleta e exibição dos gráficos
realTimePlot(s);

% Fecha o serial
delete(s)
clear s
