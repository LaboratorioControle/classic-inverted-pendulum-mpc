function J = simular_e_custo_lqr(p, dados)
    % p é o vetor de parâmetros que o 'ga' vai otimizar.
    % Vamos definir p = [q1, q2, q3, q4, r]
    % q1 = peso da posição (x)
    % q2 = peso do ângulo (theta)
    % q3 = peso da vel. do carro (x_dot)
    % q4 = peso da vel. angular (theta_dot)
    % r  = peso do controle (R)

    % 1. Extrair parâmetros e calcular ganho LQR
    try
        Q = diag([p(1), p(2), p(3), p(4)]);
        R = p(5);
        
        % O 'ga' pode testar valores >= 0. R deve ser > 0.
        if R <= 1e-6 
            J = 1e20; % Penalidade muito alta se R for inválido
            return;
        end

        K = dlqr(dados.planta.A, dados.planta.B, Q, R);
    catch
        % Se dlqr falhar (ex: Q/R inválidos)
        J = 1e20; % Penalidade muito alta
        return;
    end

    % 2. Configurar simulação (copiado do seu script)
    Ts = dados.geral.Ts;
    Tf = dados.geral.Tf;
    x(1,:) = [dados.geral.inicial.x0 dados.geral.inicial.theta0*pi/180 0 0];
    u_volt(1) = 0;
    u_force(1) = 0;
    x_des = [0; 0; 0; 0];
    sat = @(x, x_max, x_min) min( x_max, max(x_min,x));
    dead_zone = 0.5*pi/180;

    % 3. Rodar a simulação (versão simplificada)
    % Pré-alocar vetores para eficiência
    num_steps = round(Tf / Ts);
    x_hist = zeros(num_steps + 1, 4);
    u_hist = zeros(num_steps + 1, 1);
    x_hist(1,:) = x(1,:);
    u_volt = zeros(num_steps + 1, 1);
    u_force = zeros(num_steps + 1, 1);

    i = 1;
    for k = 0:Ts:Tf
        estado_atual = x_hist(i,:)';

        % Calcular novo estado (assume que RK4_discrete e Volt2Force existem)
        novo_estado = RK4_discrete(estado_atual', u_force(i), Ts, dados)';
        x_hist(i+1,:) = novo_estado';

        % Calcular controle
        if abs(novo_estado(2)) < dead_zone
            u_volt(i+1) = 0;
        else
            u_volt(i+1) = -K * (novo_estado - x_des);
            u_volt(i+1) = sat(u_volt(i+1), 12, -12);
        end

        u_force(i+1) = Volt2Force(u_volt(i+1), novo_estado(3), dados.motor);
        u_hist(i+1) = u_volt(i+1); % Guardar tensão

        i = i+1;
    end
    t = (0:Ts:(Tf+Ts))'; % Vetor de tempo

    % 4. Calcular o Custo (J)
    % Esta é a parte mais importante. 
    % Queremos minimizar o erro e o esforço de controle.
    
    % Pesos para a nossa função de custo de *desempenho*
    % (Não confundir com Q e R)
    W_posicao = 1.0;  % Penalidade no erro de posição do carro
    W_angulo = 10.0;  % Penalidade alta no erro do ângulo do pêndulo
    W_controle = 0.01; % Penalidade no esforço de controle (tensão)

    % Calcular o Integral Square Error (ISE) e o esforço
    % x_hist(:,1) = posição (x)
    % x_hist(:,2) = ângulo (theta)
    
    % Usar 'trapz' para integrar o erro ao quadrado ao longo do tempo
    custo_posicao = trapz(t, x_hist(:,1).^2);
    custo_angulo = trapz(t, x_hist(:,2).^2);
    custo_controle = trapz(t, u_hist.^2);

    % Custo total = soma ponderada dos custos
    J = W_posicao * custo_posicao + W_angulo * custo_angulo + W_controle * custo_controle;

    % (Opcional) Adicionar custo por instabilidade
    if any(isnan(J)) || any(abs(x_hist(:)) > 100) % Se a simulação "explodiu"
        J = 1e30; % Penalidade massiva
    end
end