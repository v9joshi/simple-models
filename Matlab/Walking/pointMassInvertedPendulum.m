% 2-D inverted pendlum walker
% The walker has a point-mass and point-feet. In each step the mass moves
% in an inverted pendulum motion over the stance-foot. Push-off and
% heel-strike happen simultaneously such that the double-support period is
% instantaneous.

% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point

stepLength = 0.1; % The target step length

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.stepLength = stepLength;


% Initial conditions
x0 = 0;  vx0 = 1.5;         % Start at mid-stance with some forward velocity
y0 = sqrt(L0^2 - x0^2);     % Enforce inverted pendulum constraint
vy0 = 0;                    % No velocity along the leg

vx0 = min(vx0, sqrt(g*L0)); % Froude number considerations:
                            % Mid-stance velocity lower than froude number
                            % doesn't guarantee that the leg will never
                            % pull on the ground, however it does provide
                            % an upper bound.

footX = 0;                  % Initial foot position

t0 = 0;                     % Starting time
tmax = 2*stepLength/vx0;    % This must be larger than step time
nSteps = 100;               % Simulating these many steps


% Pack the states together
state0   = [x0; y0; vx0; vy0; footX];
tSpan    = linspace(t0,tmax,1000);

%% Setup ode and contact functions
ODE_walk        = @(t,statevar) ODE_2DInvertedPendulum(t,statevar,params);
Event_walk      = @(t,statevar) HeelStrike_2DInvertedPendulum(t,statevar,params);
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

%% Try plotting something
figure(1)
set(gcf,'color','w')
plot(x, y);
hold on
for index = 1:length(timeStore)
    line([x(index), xf(index)], [y(index), 0]);
end
xlim([-1, max(x) + 1]);
axis equal
hold off

%%
figure(2)
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

figure(3)
set(gcf,'color','w')
F = (g*y - vx.^2 - vy.^2)*m/L0;
plot(timeStore,F);
xlabel('Time (s)')
ylabel('Leg Force (N)')

figure(4)
set(gcf,'color','w')
E = m*g*y + 0.5*m*(vx.^2 + vy.^2);
plot(timeStore,E);
xlabel('Time (s)')
ylabel('System energy (J)')