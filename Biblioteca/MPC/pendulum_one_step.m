function x_d = pendulum_one_step(x, u, tau, dados)

    z1 = pendulum_model([x, u], dados); 
    z2 = pendulum_model([x + 0.5*tau*z1, u], dados);
    z3 = pendulum_model([x + 0.5*tau*z2, u], dados);
    z4 = pendulum_model([x + tau*z3, u], dados);
    
    x_d = x + tau*(z1+2*z2+2*z3+z4)/6;
end