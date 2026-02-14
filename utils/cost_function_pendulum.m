function erro = cost_function_pendulum(p, dados, var, t_exp, y_exp, duracao, amplitude, frequencia, x0)

    % Parâmetros que serão ajustados
    dados.pendulo.b  = p(1)*var(1);
    dados.carro.m = p(2)*var(2);
    dados.pendulo.I = p(3)*var(3);

    % Simulação da planta discreta com Runge-Kutta
    x_sim(1,:) = [0 0 0 0];

    u = zeros(size(t_exp));
    idx = t_exp <= duracao;
    u(idx) = amplitude * sin(2*pi*frequencia*t_exp(idx));

    for k = 1:length(t_exp)-1
        x_sim(k+1,:) = RK4_discrete(x_sim(k,:), u(k), dados.geral.Ts, dados);
    end


    % Conversões para cm e °/s
    pos_sim = x_sim(:,1) * 100;
    vel_ang_sim = x_sim(:,4) * (180/pi);

    % Cálculo do erro do simulado com o experimental
    erro_pos = pos_sim - y_exp(:,1);
    erro_vel = vel_ang_sim - y_exp(:,2);

    % Retorna vetor de erro
    erro = [erro_pos / std(y_exp(:,1)); erro_vel / std(y_exp(:,2))];
end
