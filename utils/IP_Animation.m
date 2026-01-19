function animation =  IP_Animation(x,th, dados)

persistent j
persistent pjx
persistent pjy
    if isempty(j)
        j = 0;
    end
    j = j+1 

W  = dados.carro.l*100; % width of cart
H  = dados.carro.h*100; % hight of Cart
L  = dados.pendulo.l*2*100; % length of pendulum  
wr = 2; % right wheel 
guia = dados.geral.guia;

% Position coordination
y = H/2+wr/2;
w1x = x -0.9*W/2;
w1y = 0;
w2x = x+0.9*W/2-wr;
w2y = 0;

% position of pendulum 
px = x + L*sind(th);
py = y - L*cosd(th);

pjx(j) = px;
pjy(j) = py;

base = plot([-guia/2 guia/2],[0 0],'k','LineWidth',2); % base line
hold on;
cart = rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',0.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]);
left_wheel  = rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);
right_wheel = rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);
    
pendulum = plot([x px],[y py],'b','LineWidth',2.5); % Pendulum rod 
p_cir = viscircles([px py],0.02,'Color',[1 0.1 0.1],'LineWidth',2.5); % Pendulum Trajectory(Circle)
p_cir1 = viscircles([x y],0.02,'Color','w','LineWidth',0.2); % center of Cart
  
%line_traj = plot(pjx(1:j),pjy(1:j), 'm--','LineWidth',1);  % Pendulum Trajectory (Line)
    xlabel('X (cm)');
    ylabel('Y (cm)');
    title('GAT125 (22A) - Inverted Pendulum')
    %axis(gca,'equal');
    xlim([-80 80]);
    ylim([-50 50]);
    %set(gcf,'Position',[10 900 800 400])
    grid on;
    %drawnow
    %pause(0.01);
   
   
   