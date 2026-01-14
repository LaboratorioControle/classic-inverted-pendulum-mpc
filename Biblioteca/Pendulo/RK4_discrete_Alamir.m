function x_d = RK4_discrete_Alamir(x,u,tau, dados)

    z1 = Modelo_Continuo_Alamir([x, u], dados); 
    z2 = Modelo_Continuo_Alamir([x + 0.5*tau*z1, u], dados);
    z3 = Modelo_Continuo_Alamir([x + 0.5*tau*z2, u], dados);
    z4 = Modelo_Continuo_Alamir([x + tau*z3, u], dados);
    
    x_d = x + tau*(z1+2*z2+2*z3+z4)/6;
end