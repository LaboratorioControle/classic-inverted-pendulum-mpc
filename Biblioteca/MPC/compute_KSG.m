function [K,S,G] = compute_KSG(tau, Np, planta, controlador)

    A = planta.A;
    B = planta.B;
    Q = controlador.Q;
    R = controlador.R;
    
    [K, S, ~] = dlqr(A,B,Q,R); S = S/norm(S);
    
    Abar = A/B*K; inter1 = zeros(Np,Np); inter2 = zeros(Np,5); inter3 = C;

    for i=1:Np
        inter2(i,:) = inter3;
        inter1(i,1:i) = ones(1,i)*tau;
        inter3 = inter3*A;
    end

    G = inter1*inter2;
    return;
end