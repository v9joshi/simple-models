% 2-D inverted pendlum walker
% The walker has a point-mass and point-feet. In each step the mass moves
% in an inverted pendulum motion over the stance-foot. Push-off and
% heel-strike happen simultaneously such that the double-support period is
% instantaneous.

% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point

stepLength = 0.5; % The target step length

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.stepLength = stepLength;


% Initial conditions
x0 = 0;  vx0 = 1.5;       % Start at mid-stance with some forward velocity
y0 = sqrt(L0^2 - x0^2); % Enforce inverted pendulum constraint
vy0 = 0;                % No velocity along the leg
footX = 0;              % Initial foot position

t0 = 0;                 % Starting time
tmax = 3;              % This must be larger than step time
nSteps = 100;            % Simulating these many steps


% Pack these together
state0   = [x0; y0; vx0; vy0; footX];
tSpan    = linspace(t0,tmax,1000);

%% Setup ode and contact functions
ODE_walk        = @(t,statevar) ODE_2DInvertedPendulum(t,statevar,params);
Event_walk      = @(t,statevar) HeelStrike_2DInvertedPendulum(t,statevar,params);
contactFunction = @(t, statevar)   Contact_2DInvertedPendulum(t,statevar,params);


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
end


%% Try plotting something
figure(1)
plot(stateStore(:,1), stateStore(:,2));
hold on
for index = 1:length(timeStore)
    line([stateStore(index,1), stateStore(index,5)], [stateStore(index,2), 0]);
end
xlim([-1, max(stateStore(:,1)) + 1]);
axis equal