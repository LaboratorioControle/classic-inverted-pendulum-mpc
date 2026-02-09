clear; clc;

porta = "COM8";     % ajuste para a porta do seu ESP32
baud  = 115200;

disp("📡 Abrindo porta serial...");
s = serialport(porta, baud);

flush(s); % limpa buffer

% Vetores de armazenamento
tempo = [];
x = [];
theta = [];
x_dot = [];
theta_dot = [];
u = [];

gravando = false;   % flag para indicar se deve gravar dados

disp("📥 Aguardando 'INICIO'...");

while true
    try
        linha = readline(s);
        linha = strtrim(linha);

        % Ignora linhas vazias
        if linha == ""
            continue;
        end

        % ------------------------------
        % Comandos especiais INICIO / FIM
        % ------------------------------
        if strcmpi(linha, "INICIO")
            disp("▶️  Iniciando gravação...");
            gravando = true;
            continue;
        end

        if strcmpi(linha, "FIM")
            disp("⛔ Parando gravação...");
            break;  % encerra o while
        end

        % Se ainda não recebeu INICIO → ignora
        if ~gravando
            continue;
        end

        % ------------------------------
        % Tratamento de linha de dados
        % ------------------------------
        valores = str2double(split(linha, ','));

        if length(valores) ~= 6
            fprintf("Linha inválida: %s\n", linha);
            continue;
        end
        
        tempo(end+1)        = valores(1);
        x(end+1)        = valores(2);
        theta(end+1)    = valores(3);
        x_dot(end+1)     = valores(4);
        theta_dot(end+1) = valores(5);
        u(end+1)        = valores(6);

    catch ME
        fprintf("Erro: %s\n", ME.message);
    end
end

clear s;
disp("✔️ Coleta encerrada.");


%%

figure;

subplot(2,2,1);
plot(tempo, x*100, 'LineWidth', 1.5);
ylabel('x (cm)'); grid on;

subplot(2,2,2);
plot(tempo, theta*180/pi, 'LineWidth', 1.5);
ylabel('\theta (°)'); grid on;
%ylim([-200 200]);

subplot(2,2,3);
plot(tempo, x_dot*100, 'LineWidth', 1.5);
ylabel('x dot (cm/s)'); grid on;

subplot(2,2,4);
plot(tempo, theta_dot*180/pi, 'LineWidth', 1.5);
ylabel('\theta dot (°/s)'); grid on;

figure;
plot(tempo, u, 'LineWidth', 1.5);
ylabel('u (V)'); xlabel('Amostra'); grid on;


%%

exportados.tempo = tempo;
exportados.theta = theta;
exportados.theta_dot = theta_dot;
exportados.x = x;
exportados.x_dot = x_dot;
exportados.u = u;


%timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%filename = ['data/processed/Ensaio_MPC_' timestamp '.mat'];

%save(filename, 'exportados');

clear;