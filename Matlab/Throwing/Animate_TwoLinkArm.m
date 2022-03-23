% Solve a two-link manipulator for given intial conditions
function [theta,dtheta]= Animate_TwoLinkArm(torques,time,theta,dtheta,params)

hold off;
% Unpack useful things
theta1  = theta(1);
theta2  = theta(2);

dtheta1 = dtheta(1);
dtheta2 = dtheta(2);

% Manipulator parameters
l1 = params.l1;
l2 = params.l2;

% Input parameters
params.tau1 = torques(1); params.tau2 = torques(2);

% Set up initial conditions theta1, theta2, dtheta1, dtheta2 and time
inputvar = [theta1, theta2, dtheta1, dtheta2];
tspan = linspace(0,time,40);

% Set up solver function
twolink_ode = @(t,vars) twolink_dynamics(t,vars,params);
options = odeset('abstol',10^-9,'reltol',10^-9);

% Solve the problem
[tlist, outvars] = ode45(twolink_ode,tspan,inputvar,options);

theta1 = outvars(:,1);
theta2 = outvars(:,2);

dtheta1 = outvars(:,3);
dtheta2 = outvars(:,4);


% Determine total energy of the system
P1 = [l1*cos(theta1),l1*sin(theta1)];
P2 = P1 + [l2*cos(theta1+theta2), l2*sin(theta1+theta2)];

%Animating the movement of the double pendulum
x0 = 0;
y0 = 0;

x1 = P1(1,1);
y1 = P1(1,2);

x2 = P2(1,1);
y2 = P2(1,2);

figure(3);
plot(x0,y0,'^r','markersize',10,'markerfacecolor','r'); hold on;
plot(1 + cos(0:0.01:2*pi+0.01),sin(0:0.01:2*pi+0.01),'r');
firstlink = line([x0, x1],[y0, y1],'marker','o','markerfacecolor','r','markeredgecolor','r');
secondlink = line([x1, x2],[y1, y2],'marker','o','markerfacecolor','r','markeredgecolor','r');

axis([-1.5*(l1+l2), 1.5*(l1+l2), -1.5*(l1+l2), 1.5*(l1+l2)]);
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

theta = [theta1(end);theta2(end)];
dtheta = [dtheta1(end);dtheta2(end)];
end