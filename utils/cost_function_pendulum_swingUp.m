function erro = cost_function_pendulum_swingUp(p, dados, var, t_exp, y_exp, u_exp)

    % Ajuste dos parâmetros
    dados.motor.Rm = p(1)*var(1);
    dados.motor.Kt    = p(2)*var(2);
    dados.motor.Kb  = p(3)*var(3);
    dados.carro.c = p(4)*var(4);

    Ts = dados.geral.Ts;
    N  = length(t_exp);

    x_sim = zeros(N,4);

    % Estado inicial igual ao experimental
    x_sim(1,:) = [y_exp(1,1)/100, 0, 0, y_exp(1,2)*pi/180];

    for k = 1:N-1

        % Usa tensão REAL medida no experimento
        u_k = u_exp(k);

        % Integra modelo
        x_next = RK4_discrete(x_sim(k,:), u_k, Ts, dados);

        x_sim(k+1,:) = x_next';
    end

    %% Conversões para comparação

    pos_sim     = x_sim(:,1) * 100;        % m → cm
    vel_ang_sim = x_sim(:,4) * (180/pi);   % rad/s → deg/s

    erro_pos = pos_sim - y_exp(:,1);
    erro_vel = vel_ang_sim - y_exp(:,2);

    erro = [erro_pos / std(y_exp(:,1)); erro_vel / std(y_exp(:,2))];
end
