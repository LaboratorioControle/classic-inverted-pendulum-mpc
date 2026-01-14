%% Comparação - LQR x MPC Sem Restrições

A = dados.planta.A;
B = dados.planta.B;

MPC.Cr = eye(length(B));
MPC.Qy = controlador.lqr.Q;
MPC.Qu = controlador.lqr.R;

[n, nu] = size(B);
MPC.A = A; 
MPC.B = B;

Nmax=200;
K=zeros(Nmax,n);
lesN=(1:1:Nmax)';

for i=1:Nmax
    MPC.N=lesN(i);
    [H,F1,F2,F3]=compute_cost_matrices(MPC);
    K(i,:)=P_i(1,nu,i)*(H\F1);
end

one=ones(size(lesN));
K_LQR = controlador.lqr.K;

%% Plot dos resultados

figure;
sgtitle('Comparação LQR x MPC sem restrições');
subplot(2,2,1);
plot(lesN,K(:,1),lesN,one*K_LQR(1),'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da posição x');

subplot(2,2,2);
plot(lesN,K(:,2),lesN,one*K_LQR(2),'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da posição \theta');

subplot(2,2,3);
plot(lesN,K(:,3),lesN,one*K_LQR(3),'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da velocidade x_{dot}');

subplot(2,2,4);
plot(lesN,K(:,4),lesN,one*K_LQR(4),'k-.');
legend('Ganho K_n - Controlador MPC', 'Ganho K_{lqr} - Controlador LQR');
title('Ganhos da velocidade \theta_{dot}');