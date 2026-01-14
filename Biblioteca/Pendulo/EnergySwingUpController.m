function u_volt = EnergySwingUpController(x, dados)

    % -----------------------------
    % Extrair estados
    theta     = x(2);
    x_dot     = x(3);
    theta_dot = x(4);

    % -----------------------------
    % Parâmetros do sistema
    m = dados.pendulo.m;
    l = dados.pendulo.l;
    I = dados.pendulo.I;
    g = dados.geral.g;

    % Ganhos do controlador
    ke = dados.controlador.energia.k;

    % -----------------------------
    % Energia
    E = m*g*l*(1 - cos(theta)) + 0.5*(I + m*l^2)*(theta_dot^2);
    E_des = 2*m*g*l;
    diffE = E - E_des;

    % -----------------------------
    % Forçar primeiro movimento caso θ = 0, θ̇ = 0
    arg = theta_dot * cos(theta);

    %if abs(arg) < 1e-4
    %    sign_arg = 1.0;      % sua escolha original
    %else
        
    %end

    sign_arg = sign(arg);

    x_ddot_des = diffE * sign_arg;
    x_ddot_des = ke * g * x_ddot_des;
    
    
    u_force = Compute_Force_Energy(x, x_ddot_des, dados);
    u_volt = Force2Volt(u_force, x_dot, dados.motor);
end


% ==========================
% Função auxiliar embutida
% ==========================
function y = sat(x, hi, lo)
    y = min(hi, max(lo, x));
end