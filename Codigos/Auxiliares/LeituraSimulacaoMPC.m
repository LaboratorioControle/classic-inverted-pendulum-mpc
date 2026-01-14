clear; clc;

porta = "COM8";     % ajuste para a porta do seu ESP32
baud  = 115200;

disp("📡 Abrindo porta serial...");
s = serialport(porta, baud);

flush(s); % limpa buffer

% Vetores de armazenamento
t = [];
X = [];
THETA = [];
Xdot = [];
THETAdot = [];
U = [];

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
        
        t(end+1)        = valores(1);
        X(end+1)        = valores(2);
        THETA(end+1)    = valores(3);
        Xdot(end+1)     = valores(4);
        THETAdot(end+1) = valores(5);
        U(end+1)        = valores(6);

    catch ME
        fprintf("Erro: %s\n", ME.message);
    end
end

clear s;
disp("✔️ Coleta encerrada.");


%%

figure;

subplot(2,2,1);
plot(t, X*100, 'LineWidth', 1.5);
ylabel('x (m)'); grid on;

subplot(2,2,2);
plot(t, THETA*180/pi, 'LineWidth', 1.5);
ylabel('\theta (rad)'); grid on;
%ylim([-200 200]);

subplot(2,2,3);
plot(t, Xdot*100, 'LineWidth', 1.5);
ylabel('x dot (m/s)'); grid on;

subplot(2,2,4);
plot(t, THETAdot*180/pi, 'LineWidth', 1.5);
ylabel('\theta dot (rad/s)'); grid on;

figure;
plot(t, U, 'LineWidth', 1.5);
ylabel('u (V)'); xlabel('Amostra'); grid on;


%%

exportados.tempo = t;
exportados.theta = THETA;
exportados.theta_dot = THETAdot;
exportados.x = X;
exportados.x_dot = Xdot;
exportados.u = U;
