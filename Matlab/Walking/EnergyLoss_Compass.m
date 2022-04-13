function EnergyLoss = EnergyLoss_Compass(inputParams)
    % Set the parameters
    g = 10;           % acceleration due to gravity
    L0 = 1;           % Length of the leg
    m  = 1;           % Mass of the foot, concentrated at a single point
    M = 70;           % Mass of the HAT, concentrated at a single point
    gamma = -0.01;    % Slope of the ground, in radians

    % Pack parameters
    params.g = g; params.L0 = L0; params.m = m; params.M = M; params.gamma = gamma;

    % Initial conditions
    x00 = 0;             y00  = 0;                  
    x10 = 0;             vx10 = inputParams(2);
    x20 = inputParams(1);vx20 = inputParams(3);
    
    % Enforce leg length constraints
    % Stance leg
    y10 =  y00 + sqrt(L0^2 - (x10 - x00)^2);             
    vy10 = -(x10 - x00)*vx10/(y10 - y00);

    % Swing leg
    y20 = y10 - sqrt(L0^2 - (x20 - x10)^2);
    vy20 = vy10 - (x20 - x10)*(vx20 - vx10)/(y20 - y10);

    % Time settings
    t0 = 0;         % Starting time.
    tmax = 5;      % This must be larger than step time.
    
    % Pack the states together
    state0   = [x10; x20; y10; y20; vx10; vx20; vy10; vy20; x00; y00];
    tSpan    = linspace(t0,tmax,tmax*1000);

    % Set the ODE files
    ODE_walk        = @(t,statevar) ODE_2DCompass(t,statevar,params);
    Event_walk      = @(t,statevar) HSEvent_2DCompass(t,statevar,params);
    contactFunction = @(t,statevar) Contact_2DCompass(t,statevar,params);

    %% Simulate the motion
    stateStore = [];
    timeStore  = [];
    collTimeStore = [];
    % simulate a movement
    options = odeset('reltol',1e-9,'abstol',1e-9,'Events',Event_walk);

    [tListOut,stateListOut] = ode15s(ODE_walk,tSpan,state0,options);

    stateStore = [stateStore; stateListOut(1:end-1,:)]; % leaving out the last point to avoid repetition
    timeStore = [timeStore; t0 + tListOut(1:end-1)];

    % Apply the collision law
    state0 = contactFunction(t0, stateListOut(end,:));
    stateStore = [stateStore; state0];
    
    %% Unpack the state variables
    x1  = stateStore(:,1); x2  = stateStore(:,2);
    y1  = stateStore(:,3); y2  = stateStore(:,4);
    vx1 = stateStore(:,5); vx2 = stateStore(:,6);
    vy1 = stateStore(:,7); vy2 = stateStore(:,8);
    xf  = stateStore(:,9); yf  = stateStore(:,10);
    
    %% Find the kinetic energy
    kineticEnergy = 0.5*M*(vx1.^2 + vy1.^2) + 0.5*m*(vx2.^2 + vy2.^2);

    EnergyLoss =  (1 - (kineticEnergy(end)/kineticEnergy(1))^2);
end
    
    
