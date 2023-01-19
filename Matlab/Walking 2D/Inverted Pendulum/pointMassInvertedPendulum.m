    % 2-D inverted pendlum walker
% The walker has a point-mass and point-feet. In each step the mass moves
% in an inverted pendulum motion over the stance-foot. Push-off and
% heel-strike happen simultaneously such that the double-support period is
% instantaneous.
close all;

% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point

stepLength = 0.7;        % The target step length
% Step length can't be larger than sqrt(20)*L0/3 
stepLength = min(stepLength,sqrt(20)*L0/3); 

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.stepLength = stepLength;

% Initial conditions
x0 = 0;  vx0 = 1.2;         % Start at mid-stance with some forward velocity
y0 = sqrt(L0^2 - x0^2);   % Enforce inverted pendulum constraint
vy0 = 0;                  % No velocity along the leg                          
footX = 0;                % Initial foot position

% Froude number considerations: v < sqrt(g*L0)
% Mid-stance velocity lower than froude number doesn't guarantee that
% the leg will never pull on the ground, however it does provide
% an upper bound. 
% We can calculate the exact upper bound using the step-length.
hmin = sqrt(L0^2 - stepLength^2/4);
vx0  = min(vx0, sqrt(3*g*hmin - 2*g*L0));

if vx0 == 0
    disp("can't walk forward due to low speed")
end

t0 = 0;                   % Starting time
tmax = 2*stepLength/vx0;  % This must be larger than step time
nSteps = 100;             % Simulating these many steps

% Pack the states together
state0   = [x0; y0; vx0; vy0; footX];
tSpan    = linspace(t0,tmax,1000);

%% Setup ode and contact functions
ODE_walk        = @(t,statevar) ODE_2DInvertedPendulum(t,statevar,params);
Event_walk      = @(t,statevar) HSEvent_2DInvertedPendulum(t,statevar,params);
contactFunction = @(t,statevar) Contact_2DInvertedPendulum(t,statevar,params);

%% Simulate the steps
stateStore = [];
timeStore  = [];
for currStep = 1:nSteps
    % simulate a step
    options = odeset('reltol',1e-9,'abstol',1e-9,'Events',Event_walk);
    [tListOut,stateListOut] = ode45(ODE_walk,tSpan,state0,options);

    stateStore = [stateStore; stateListOut(1:end-1,:)]; % leaving out the last point to avoid repetition
    timeStore = [timeStore; t0 + tListOut(1:end-1)];
    
    % Apply heel-strike and push-off impulses
    t0 = t0 + tListOut(end);
    state0 = contactFunction(t0, stateListOut(end,:));
    
    % Check to see if the CoM went underground during this step
    if min(stateListOut(:,2)) < 0
        disp("The walker fell")
        break
    end
end

%% Unpack the state variables
x  = stateStore(:,1);
y  = stateStore(:,2);
vx = stateStore(:,3);
vy = stateStore(:,4);
xf = stateStore(:,5);

%% Plot the motion of the system
figure(1)
set(gcf,'color','w')
plot(x, y);
hold on
line([x(1:nSteps:end), xf(1:nSteps:end)]', [y(1:nSteps:end), 0*y(1:nSteps:end)]','color','k');
xlim([-1, max(x) + 1]);
axis equal
hold off

%% Plot timeseries of leg force, leg length etc.
figure(2)
set(gcf,'color','w')
F = (g*y - vx.^2 - vy.^2)*m/L0;
plot(timeStore,F);
xlabel('Time (s)')
ylabel('Leg Force (N)')

figure(3)
subplot(3,1,1)
set(gcf,'color','w')
L = sqrt((x - xf).^2 + y.^2);
plot(timeStore,L);
xlabel('Time (s)')
ylabel('Leg length (m)')

subplot(3,1,2)
Ldot = (x - xf).*vx + y.*vy;
plot(timeStore,Ldot);
xlabel('Time (s)')
ylabel('Leg length rate (m/s^-^1)')

subplot(3,1,3)
Ldotdot = F*L0/m - g*y + vx.^2 + vy.^2;
plot(timeStore,Ldotdot);
xlabel('Time (s)')
ylabel('Leg length rate rate (m/s^-^2)')

%% Plot the system energy.
% As this is a conservative system, total energy should remain constant.
figure(4)
set(gcf,'color','w')
E = m*g*y + 0.5*m*(vx.^2 + vy.^2);
plot(timeStore,E);
xlabel('Time (s)')
ylabel('System energy (J)')

%% Animate the bipedal walker
figure(5)
set(gcf, 'color','w'); %,'Position', get(0, 'Screensize'));

% Plot the initial state
hold on
leg_stance   = plot([xf(1),x(1)],[0,y(1)],'k-','linewidth',2);
mass_point = plot(x(1),y(1),'ro','markerfacecolor','r','markersize',20);
leg_swing  = plot([xf(1),x(1)],[0,y(1)],'b-','linewidth',2);

% Make some ground
ground_pre = plot([xf(1), stepLength],[0,0],'color',[0,0.5,0],'LineStyle','--','linewidth',3);
ground_mid = plot([xf(1),xf(1) + stepLength],[0,0],'color',[0,0.8,0],'LineStyle','--','linewidth',3);
ground_post = plot([xf(1) + stepLength,xf(1) + 2*stepLength],[0,0],'color',[0,0.5,0],'LineStyle','--','linewidth',3);

% Set axis properties
set(gca,'visible','off')
hold off
axis([-2*stepLength, 2*stepLength, -1, 2]);
axis equal

% Set other useful animation properties
currFoot = xf(1);
avgSpeed = max(x)/length(timeStore);

% Run the animation
for i = 2:20:length(timeStore)
    if xf(i) ~= currFoot
       % Swap legs at step
       swap = leg_stance;
       leg_stance = leg_swing;
       leg_swing = swap;
       
       % Change ground colors at step
       ground_pre.Color = ground_mid.Color;
       ground_mid.Color = ground_post.Color;
       ground_post.Color = ground_pre.Color;
       
       % Update the foot position
       currFoot = xf(i);
    end
        
    % Change the mass locations
    set(mass_point,'ydata',y(i));
    set(mass_point,'xdata',x(i));
 
    % Change the leg end-points
    set(leg_stance,'ydata',[0, y(i)])
    set(leg_stance,'xdata',[xf(i), x(i)])
    
    % Change the leg end-points
    set(leg_swing,'ydata',[0, y(i)])
    set(leg_swing,'xdata',[2*x(i) - xf(i), x(i)])
    
    % Change the ground line
    set(ground_pre,'xdata',[xf(i),xf(i) - stepLength])
    set(ground_mid,'xdata',[xf(i),xf(i) + stepLength])
    set(ground_post,'xdata',[xf(i) + stepLength,xf(i) + 2*stepLength])

    axis([avgSpeed*i - 2*stepLength,avgSpeed*i + 2*stepLength, -1, 2]);
        
    pause(0.01);
end