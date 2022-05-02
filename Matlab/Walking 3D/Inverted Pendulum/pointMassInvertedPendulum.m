% 3-D inverted pendlum walker
% The walker has a point-mass and point-feet. In each step the mass moves
% in an inverted pendulum motion over the stance-foot. Push-off and
% heel-strike happen simultaneously such that the double-support period is
% instantaneous.
close all;

% Directions - X = Forward +ve, Y = Up +ve, Z = Right +ve
% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point

stepLength = 0.7;        % The target step length
stepWidth  = 0;        % The target step width

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; 
params.stepLength = stepLength; params.stepWidth = stepWidth;

% Initial conditions
footX = 0; footY = 0; footZ = stepWidth*0.5;      % Initial foot position
x0 = 0;      vx0 = 1.2;                           % Start at mid-stance with some forward velocity
z0 = 0;      vz0 = 0;

% Enforce inverted pendulum constraint
% Leg length must be L0
y0 = footY + sqrt(L0^2 - (x0 - footX)^2 - (z0 - footZ)^2);       
% No velocity along the leg                          
vy0 = (-vx0*(x0 - footX) - vz0*(z0 - footZ))/(y0 - footY);                             

% Froude number considerations: v < sqrt(g*L0)
% Mid-stance velocity lower than froude number doesn't guarantee that
% the leg will never pull on the ground, however it does provide
% an upper bound. 
% We can calculate the exact upper bound using the step-length.
% hmin = sqrt(L0^2 - stepLength^2/4);
% vx0  = min(vx0, sqrt(3*g*hmin - 2*g*L0));

if vx0 == 0
    disp("can't walk forward due to low speed")
end

t0 = 0;                   % Starting time
tmax = 2*stepLength/vx0;  % This must be larger than step time
nSteps = 20;             % Simulating these many steps

% Pack the states together
state0   = [x0; y0; z0; vx0; vy0; vz0; footX; footY; footZ];
tSpan    = linspace(t0,tmax,1000);

%% Setup ode and contact functions
ODE_walk        = @(t,statevar) ODE_3DInvertedPendulum(t,statevar,params);
Event_walk      = @(t,statevar) HSEvent_3DInvertedPendulum(t,statevar,params);
contactFunction = @(t,statevar) Contact_3DInvertedPendulum(t,statevar,params);

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
z  = stateStore(:,3);
vx = stateStore(:,4);
vy = stateStore(:,5);
vz = stateStore(:,6);
xf = stateStore(:,7);
yf = stateStore(:,8);
zf = stateStore(:,9);

%% Try plotting something
figure(1)
set(gcf,'color','w')
plot3(x, z, y);
hold on
line([x(1:nSteps:end), xf(1:nSteps:end)]', [z(1:nSteps:end), zf(1:nSteps:end)]',[y(1:nSteps:end), yf(1:nSteps:end)]','color','k');
xlim([-1, max(x) + 1]);
view(0,80)
hold off

%%
figure(2)
set(gcf,'color','w')
F = (g*y - vx.^2 - vy.^2 - vz.^2)*m/L0;
plot(timeStore,F);
xlabel('Time (s)')
ylabel('Leg Force (N)')

figure(3)
subplot(3,1,1)
set(gcf,'color','w')
L = sqrt((x - xf).^2 + (y - yf).^2 + (z - zf).^2);
plot(timeStore,L);
xlabel('Time (s)')
ylabel('Leg length (m)')

subplot(3,1,2)
Ldot = (x - xf).*vx + (y - yf).*vy + (z - zf).*vz;
plot(timeStore,Ldot);
xlabel('Time (s)')
ylabel('Leg length rate (m/s^-^1)')

subplot(3,1,3)
Ldotdot = F*L0/m - g*y + vx.^2 + vy.^2 + vz.^2;
plot(timeStore,Ldotdot);
xlabel('Time (s)')
ylabel('Leg length rate rate (m/s^-^2)')

figure(4)
set(gcf,'color','w')
E = m*g*y + 0.5*m*(vx.^2 + vy.^2 + vz.^2);
plot(timeStore,E);
xlabel('Time (s)')
ylabel('System energy (J)')

%% animate
figure(5)
set(gcf, 'color','w'); %,'Position', get(0, 'Screensize'));
clf;

% Plot the mass
hold on
leg_stance = plot3([xf(1),x(1)],[zf(1),z(1)],[yf(1),y(1)],'k-','linewidth',2);
mass_point = plot3(x(1),z(1),y(1),'ro','markerfacecolor','r','markersize',20);
leg_swing  = plot3([xf(1),x(1)],[zf(1),z(1)],[yf(1),y(1)],'b-','linewidth',2);

% Make some ground
ground_pre  = fill3([xf(1), xf(1), stepLength, stepLength],[-2, 2, 2, -2],[0,0,0,0],[0,0.5,0],'LineStyle','--','linewidth',1);
ground_mid  = fill3([xf(1), xf(1), xf(1) + stepLength, xf(1) + stepLength],[-2, 2, 2, -2],[0,0, 0, 0],[0,0.8,0],'LineStyle','--','linewidth',1);
ground_post = fill3([xf(1) + stepLength,xf(1) + stepLength,xf(1) + 2*stepLength,xf(1) + 2*stepLength],[-2,2,2,-2],[0,0,0,0],[0,0.5,0],'LineStyle','--','linewidth',1);

% Set axis properties
set(gca,'visible','off')
hold off
axis([-2*stepLength, 2*stepLength, -1, 1, -1, 2]);
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
       ground_pre.FaceColor = ground_mid.FaceColor;
       ground_mid.FaceColor = ground_post.FaceColor;
       ground_post.FaceColor = ground_pre.FaceColor;
       
       % Update the foot position
       currFoot = xf(i);
    end
        
    % Change the mass locations
    set(mass_point,'xdata',x(i));
    set(mass_point,'zdata',y(i));
    set(mass_point,'ydata',z(i));
     
    % Change the leg end-points
    set(leg_stance,'xdata',[xf(i), x(i)])
    set(leg_stance,'zdata',[yf(i), y(i)])
    set(leg_stance,'ydata',[zf(i), z(i)])
    
    % Change the leg end-points
    set(leg_swing,'xdata',[2*x(i) - xf(i), x(i)])
    set(leg_swing,'zdata',[yf(i), y(i)])
    set(leg_swing,'ydata',[2*z(i) - zf(i), z(i)])
        
    % Change the ground line
    set(ground_pre,'xdata',[xf(i),xf(i),xf(i) - stepLength,xf(i) - stepLength])
    set(ground_mid,'xdata',[xf(i),xf(i),xf(i) + stepLength,xf(i) + stepLength])
    set(ground_post,'xdata',[xf(i) + stepLength,xf(i) + stepLength,xf(i) + 2*stepLength,xf(i) + 2*stepLength])

    set(gca,'xlim',[avgSpeed*i - 2*stepLength,avgSpeed*i + 2*stepLength]);
    axis equal
    view(0,10)
    pause(0.01);
end