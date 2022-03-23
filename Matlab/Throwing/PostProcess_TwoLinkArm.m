% Solve a two-link manipulator for given intial conditions
function []= PostProcess_TwoLinkArm(pinput,params)

global numel 

theta = params.init_theta';
dtheta = params.init_dtheta';

display ('Post Process');
for i = 1:(numel - 1)
%     display ('within loop');
    torques(1) = pinput(i);
    torques(2) = pinput(numel - 1 + i);
        
    outputs = Root_TwoLinkArm(torques,params);
    params.init_theta  = [outputs(end,1);outputs(end,2)];
    params.init_dtheta = [outputs(end,3);outputs(end,4)];
    theta = [theta; outputs(2:end,1:2)];
    dtheta = [dtheta; outputs(2:end,3:4)];
            
end

theta1 = theta(:,1); dtheta1 = dtheta(:,1);
theta2 = theta(:,2); dtheta2 = dtheta(:,2);

HandVelocity_x = -dtheta(end,1)*params.l1*sin(theta(end,1))...
                -(dtheta(end,1)+dtheta(end,2))*params.l2*sin(theta(end,1)+theta(end,2))

HandVelocity_y = dtheta(end,1)*params.l1*cos(theta(end,1))...
               +(dtheta(end,1)+dtheta(end,2))*params.l2*cos(theta(end,1)+theta(end,2))

velocity_angle = atan2(HandVelocity_y, HandVelocity_x);


figure(1);
subplot(2,1,1)
plot(theta1);
xlabel('Index');
ylabel('Theta 1 (rad)');
title('Angles');

subplot(2,1,2)
plot(theta2);
xlabel('Index');
ylabel('Theta 2 (rad)');

figure(2);
subplot(2,1,1)
plot(dtheta1);
xlabel('Index');
ylabel('Omega 1 (rad s^-^1)');
title('Angular Velocity');

subplot(2,1,2)
plot(dtheta2);
xlabel('Index');
ylabel('Omega 2 (rad s^-^1)');

% Animate
%Animating the movement of the double pendulum
x0 = 0;
y0 = 0;

P1 = [x0+params.l1*cos(theta1),y0+params.l1*sin(theta1)];
P2 = P1 + [params.l2*cos(theta1+theta2), params.l2*sin(theta1+theta2)];

x1 = P1(1,1);
y1 = P1(1,2);

x2 = P2(1,1);
y2 = P2(1,2);

figure(4);
plot(x0,y0,'^r','markersize',10,'markerfacecolor','r'); hold on;
firstlink = line([x0, x1],[y0, y1],'marker','o','markerfacecolor','r','markeredgecolor','r');
secondlink = line([x1, x2],[y1, y2],'marker','o','markerfacecolor','r','markeredgecolor','r');

axis([-1.5*(params.l1+params.l2), 1.5*(params.l1+params.l2), -1.5*(params.l1+params.l2), 1.5*(params.l1+params.l2)]);
axis square

for i = 1:length(theta1)
    x1 = P1(i,1);
    y1 = P1(i,2);
    
    x2 = P2(i,1);
    y2 = P2(i,2);
    
    set(firstlink,'xdata',[x0, x1],'ydata',[y0, y1]);
    set(secondlink,'xdata',[x1, x2],'ydata',[y1, y2]);
    
    pause(0.1);
end
end