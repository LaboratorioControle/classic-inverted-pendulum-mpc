function F = Compute_Force_Energy(estados,x_2dot_desejado,dados)

    % Estados so sistema
    theta = estados(2);
    x_dot = estados(3);
    theta_dot = estados(4);
    
    % Variáveis da planta
    m = dados.pendulo.m;
    l = dados.pendulo.l;
    I = dados.pendulo.I;
    b = dados.pendulo.b;
    
    M = dados.carro.m;
    c = dados.carro.c;
    
    g = dados.geral.g;
    
    % Cálculo da Força
    theta_2dot = (-b*theta_dot -m*l*cos(theta)*x_2dot_desejado -m*g*l*sin(theta))/(I+m*l^2); 
    F = (M+m)*(x_2dot_desejado) + m*l*cos(theta)*theta_2dot -m*l*sin(theta)*theta_dot^2 + c*x_dot;

end
