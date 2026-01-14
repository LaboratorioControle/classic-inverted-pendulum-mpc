function EE = compute_E(x_vet, dados)

    theta = x_vet(1);
    theta_dot = x_vet(2);
    r = x_vet(3);
    r_dot = x_vet(4);

    m1 = dados.pendulo.m;
    l = dados.pendulo.l;
    I = dados.pendulo.I;
    g = dados.geral.g;
    alpha1 = (m1*l)/(m1*l^2+I); beta1 = g*alpha1;

    EE = 0.5*theta_dot^2 + beta1*(cos(theta)-1);
end