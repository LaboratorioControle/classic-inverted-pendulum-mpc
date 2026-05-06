function erro = cost_function_pendulum_swingUp(p, dados, var, t_exp, y_exp, u_exp)

    % Ajuste dos parâmetros
     % dados.motor.Rm = p(1)*var(1);
     % dados.motor.Kb    = p(2)*var(2);
     % dados.motor.Kt  = p(3)*var(3);
     % dados.motor.R = p(4)*var(4);
     % dados.carro.m = p(5)*var(5);
     

    dados.motor.Rm = p(1)*var(1);
    %dados.carro.m = p(2)*var(2);
    dados.carro.c = p(2)*var(2);
    %dados.motor.Kb = p(4)*var(4);
    %dados.pendulo.m = p(1)*var(1);
    %dados.carro.m = p(2)*var(2);

    Ts = dados.geral.Ts;
    N  = length(t_exp);

    x_sim = zeros(N,4);
    E_sim = zeros(N,1);
    E_exp = zeros(N,1);

    % Estado inicial igual ao experimental
    x_sim(1,:) = [y_exp(1,1)/100, y_exp(1,2)*pi/180, y_exp(1,3)/100, y_exp(1,4)*pi/180];

    for k = 1:N-1

        % Usa tensão REAL medida no experimento
        u_k = u_exp(k);

        %estado = [y_exp(k,1)/100, y_exp(k,2)*pi/180, y_exp(k,3)/100, y_exp(k,4)*pi/180];
        %estado = x_sim(k,:);

        %if mod(k-1, 35) == 0
            estado = [y_exp(k,1)/100, y_exp(k,2)*pi/180, y_exp(k,3)/100, y_exp(k,4)*pi/180];
        %else
            % Continua a simulação a partir do passo anterior (malha aberta dentro da janela)
            estado = x_sim(k,:);
        %end

        % Integra modelo
        x_next = RK4_discrete(estado, u_k, Ts, dados);

        E_sim(k+1) = dados.pendulo.m * dados.geral.g * dados.pendulo.l * (1 - cos(x_next(2))) ...
           + 0.5 * (dados.pendulo.I + dados.pendulo.m * dados.pendulo.l^2) * x_next(4)^2;

        theta_exp = y_exp(k+1,2)*pi/180;
        theta_dot_exp = y_exp(k+1,4)*pi/180;

        E_exp(k+1) = dados.pendulo.m * dados.geral.g * dados.pendulo.l * (1 - cos(theta_exp)) ...
           + 0.5 * (dados.pendulo.I + dados.pendulo.m * dados.pendulo.l^2) * theta_dot_exp^2;

        x_sim(k+1,:) = x_next';
    end

    %% Conversões para comparação

    pos_sim     = x_sim(:,1) * 100;        % m → cm
    vel_pos_sim     = x_sim(:,3) * 100;    
    ang_sim = x_sim(:,2) * (180/pi);
    ang_sim = wrapTo360(ang_sim);
    vel_ang_sim = x_sim(:,4) * (180/pi);   % rad/s → deg/s

    %% PLOT DOS RESULTADOS (DINÂMICO)

% figure;
% 
% % ===================== POSIÇÃO DO CARRINHO =====================
% subplot(2,2,1); hold on; grid on;
% title('Posição do Carro');
% xlabel('Tempo (s)');
% ylabel('Posição (cm)');
% 
% stairs(t_exp, y_exp(:,1), 'LineWidth', 1.1);
% stairs(t_exp, pos_sim, 'LineWidth', 1.1);
% 
% 
% legend_entries = {};
% legend_entries{end+1} = 'Experimental'; 
% legend_entries{end+1} = 'Simulado';     
% legend(legend_entries);
% 
% % ===================== ÂNGULO DO PÊNDULO =====================
% subplot(2,2,2); hold on; grid on;
% title('Ângulo');
% xlabel('Tempo (s)');
% ylabel('Ângulo (°)');
% 
% 
%     stairs(t_exp, y_exp(:,3), 'LineWidth', 1.1);
%     stairs(t_exp, ang_sim, 'LineWidth', 1.1);
% 
% 
% legend(legend_entries);
% 
% % ===================== VELOCIDADE DO CARRINHO =====================
% subplot(2,2,3); hold on; grid on;
% title('Velocidade do Carro');
% xlabel('Tempo (s)');
% ylabel('Velocidade (cm/s)');
% 
% 
%     stairs(t_exp, y_exp(:,4), 'LineWidth', 1.1);
% 
%     stairs(t_exp, vel_pos_sim, 'LineWidth', 1.1);
% 
% 
% legend(legend_entries);
% 
% % ===================== VELOCIDADE ANGULAR =====================
% subplot(2,2,4); hold on; grid on;
% title('Velocidade Angular');
% xlabel('Tempo (s)');
% ylabel('Velocidade Angular (°/s)');
% 
% 
%     stairs(t_exp, y_exp(:,2), 'LineWidth', 1.1);
%     stairs(t_exp, vel_ang_sim, 'LineWidth', 1.1);
% 
% 
% legend(legend_entries);
% 
     erro_pos = pos_sim - y_exp(:,1);
     erro_vel = vel_ang_sim - y_exp(:,4);

    erro = [erro_pos / std(y_exp(:,1)); erro_vel / std(y_exp(:,4))];
    %erro_energia = (E_sim - E_exp) ./ (max(abs(E_exp)) + 1e-6);

    %erro_vel = (vel_ang_sim - y_exp(:,4)) ./ (std(y_exp(:,4)) + 1e-6);

    %erro = [erro_energia; 0.2 * erro_vel];
end
