figure;
sgtitle('Comparação LQR x MPC');

% ===============================
% 1) Posição do carrinho (x)
% ===============================
subplot(2,2,1);
plot(simulacao.lqr.tempo, simulacao.lqr.posicao, ...
     simulacao.mpc.exp.tempo, simulacao.mpc.exp.posicao);
legend('LQR', 'MPC');
title('Posição do carrinho (x)');
xlabel('Tempo (s)');
ylabel('Posição (m)');
grid on;

% ===============================
% 2) Ângulo do pêndulo (θ)
% ===============================
subplot(2,2,2);
plot(simulacao.lqr.tempo, simulacao.lqr.angulo, ...
     simulacao.mpc.exp.tempo, simulacao.mpc.exp.angulo);
legend('LQR', 'MPC');
title('Ângulo do pêndulo (\theta)');
xlabel('Tempo (s)');
ylabel('Ângulo (rad)');
grid on;

% ===============================
% 3) Velocidade do carrinho (ẋ)
% ===============================
subplot(2,2,3);
plot(simulacao.lqr.tempo, simulacao.lqr.velocidade_carro, ...
     simulacao.mpc.exp.tempo, simulacao.mpc.exp.velocidade);
legend('LQR', 'MPC');
title('Velocidade do carrinho (x_{dot})');
xlabel('Tempo (s)');
ylabel('Velocidade (m/s)');
grid on;

% ===============================
% 4) Velocidade angular (θ̇)
% ===============================
subplot(2,2,4);
plot(simulacao.lqr.tempo, simulacao.lqr.velocidade_pendulo, ...
     simulacao.mpc.exp.tempo, simulacao.mpc.exp.vel_angular);
legend('LQR', 'MPC');
title('Velocidade angular (\theta_{dot})');
xlabel('Tempo (s)');
ylabel('Velocidade angular (rad/s)');
grid on;