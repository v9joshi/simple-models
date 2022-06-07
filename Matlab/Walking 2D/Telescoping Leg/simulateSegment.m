function [tListOut, stateListOut] = simulateSegment(input, params)
    % Read some params
    footX = params.footX;
    footY = params.footY;

    % Read the state vector and add on the foot position
    state0 = [input(1:end - 2), footX, footY, 0];

    % Set the force rate for this segment
    params.delF = input(end-1);
    
    % Set the time length for this segment
    params.delT = input(end);
    
    %% Setup ode and contact functions
    ODE_walk = @(t,statevar) ODE_2DTelescoping(t,statevar,params);
    tSpan    = [0, params.delT];

    % simulate a step
    options = odeset('reltol',1e-9,'abstol',1e-9);
    [tListOut,stateListOut] = ode45(ODE_walk,tSpan,state0,options);
end