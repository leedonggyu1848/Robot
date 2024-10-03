clc;
clear all;
close all;

% create figure
hfig = figure(20);
grid on;
axis([-2, 25, -2, 25   ])    %[xmin xmax  ymin ymax  zmin zmax]

% create line object
hMyCar1 = line(inf,inf);
set(hMyCar1, 'linestyle', 'none',    'marker', 'o',    'markeredgecolor', 'k',...
    'markerfacecolor', 'r',    'markersize', 18); 
hMyCar2 = line(inf,inf);
set(hMyCar2, 'linestyle', 'none',    'marker', 'o',    'markeredgecolor', 'k',...
    'markerfacecolor', 'g',    'markersize', 24); 

%------------------------------------------------------------------------
% dynamics, kinematics, and so on : calculation; 

car1_x = zeros(100,2);
car2_x = zeros(100,2);

car1_vx1 = 0.05; car1_vx2 = 1.0; car1_omega = 0.1; 

car2_vx1 = 0.05; car2_vx2 = 1.0; car2_omega = 0.5; 

finT = 1000
for t=1:finT
   car1_x(t+1,1) = car1_x(t,1) + car1_vx1;    % x ÁÂÇ¥ 
   car1_x(t+1,2) = car1_x(t,2) + car1_vx2*sin(car1_omega*t);    % y ÁÂÇ¥ 
   
   car2_x(t+1,1) = car2_x(t,1) + car2_vx1;    % x ÁÂÇ¥ 
   car2_x(t+1,2) = car2_x(t,2) + car2_vx2*sin(car2_omega*t);    % y ÁÂÇ¥ 
end

figure(2)
plot(car1_x(:,1),car1_x(:,2))
plot(car2_x(:,1),car2_x(:,2))
% pause;

% --------------- You can safely ignore this !-------------------
% Let draw figure!
xx = car1_x(:,1);
yy = car1_x(:,2);

xxx = car2_x(:,1);
yyy = car2_x(:,2);
T = 1000;

for t=1:T    
    set(hMyCar1, 'xdata', xx(t), 'ydata', yy(t));
    set(hMyCar2, 'xdata', xxx(t), 'ydata', yyy(t));
    pause(0.05);
    drawnow;
end

