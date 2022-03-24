% Solve a two-link manipulator for given intial conditions
function [outvars] = Root_TwoLinkArm(torques,params)

    % Unpack useful things
    % Initial state
    theta1  = params.init_theta(1);
    theta2  = params.init_theta(2);

    dtheta1 = params.init_dtheta(1);
    dtheta2 = params.init_dtheta(2);

    % Simulation time
    time = params.time;

    % Input torques for shoulder and elbow
    params.tau1 = torques(1); params.tau2 = torques(2);

    % Set up initial conditions theta1, theta2, dtheta1, dtheta2 and time
    inputvar = [theta1, theta2, dtheta1, dtheta2];
    tspan = linspace(0,time,5);

    % Set up solver function
    twolink_ode = @(t,vars) ODE_TwoLinkArm(t,vars,params);
    options = odeset('abstol',10^-9,'reltol',10^-9);

    % Solve the problem
    [~, outvars] = ode45(twolink_ode,tspan,inputvar,options);
end