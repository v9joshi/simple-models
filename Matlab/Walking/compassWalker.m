% Compass walker
% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the foot, concentrated at a single point
M = 10;           % Mass of the HAT, concentrated at a single point

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.M = M;

% Initial conditions
x10 = 0;                vx10 = 0;
x20 = -0.5;             vx20 = 0;
y10 = 1;                vy10 = 0;
y20 = 1 - sqrt(3)/2;    vy20 = 0;

% Initial foot position
footX = 0;  
footY = 0;              

% Time settings
t0 = 0;     % Starting time
tmax = 30;  % This must be larger than step time

% Pack the states together
state0   = [x10; x20; y10; y20; vx10; vx20; vy10; vy20; footX; footY];
tSpan    = linspace(t0,tmax,tmax*100);

% Set the ODE files
ODE_walk = @(t,statevar) ODE_2DCompass(t,statevar,params);

%% Simulate the motion
stateStore = [];
timeStore  = [];

% simulate a movement
options = odeset('reltol',1e-9,'abstol',1e-9);
[tListOut,stateListOut] = ode45(ODE_walk,tSpan,state0,options);

stateStore = [stateStore; stateListOut(1:end-1,:)]; % leaving out the last point to avoid repetition
timeStore = [timeStore; t0 + tListOut(1:end-1)];

%% Unpack the state variables
x1  = stateStore(:,1); x2  = stateStore(:,2);
y1  = stateStore(:,3); y2  = stateStore(:,4);
vx1 = stateStore(:,5); vx2 = stateStore(:,6);
vy1 = stateStore(:,7); vy2 = stateStore(:,8);
xf  = stateStore(:,9); yf  = stateStore(:,10);

figure(1)
set(gcf, 'color','w')
link1 = plot([xf(1), x1(1)],[yf(1), y1(1)],'b-','marker','o','markerfacecolor','b');
hold on
link2 = plot([x1(1), x2(1)],[y1(1), y2(1)],'r-','marker','o','markerfacecolor','r');
hold off
xlim([-3,3])
ylim([-3,3])
axis square
set(gca, 'visible','off')


for i = 1:length(timeStore)
    set(link1, 'xdata',[xf(i), x1(i)],'ydata',[yf(i), y1(i)])
    set(link2, 'xdata',[x1(i), x2(i)],'ydata',[y1(i), y2(i)])
    pause(0.01)
end