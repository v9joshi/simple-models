% Simulate a time-step
function [newStates] = simulateDT(dt, currStates, params)

% Unpack parameters
[x, y, vx, vy, fLeg] = translateStates(currStates);

% Setup the ODE
tSpan = [0, dt];
states0 = [x, y, vx, vy];

params.fLeg1 = fLeg(1);
params.fLeg2 = fLeg(2);

% Call the ODE
[~, statesOut] = ode45(@ODE_2DTelescoping, tSpan, states0, options);

newStates = statesOut(end,1:4);
end