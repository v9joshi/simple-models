% Function to detect the heel-strike event for a 2D point-mass inverted
% pendulum walker.
function [value,isterminal,direction] = HeelStrike_2DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; stepLength = params.stepLength;    
    
    % Unpack the state
    x = statevar(1); y = statevar(2); vx = statevar(3); vy = statevar(4);
    
    % Previous foot position
    oldFootX = statevar(5);
    
    % New foot position
    newFootX = oldFootX + stepLength;
    
    % Distance from next foot position
    d = sqrt((x - newFootX)^2 + y^2);
    
    % Evaluate the heel-strike condition
    value = d - L0;  % what goes to zero when the event happens
    isterminal = 1; % whether the simulation should be stopped or not
    direction = -1; % we want to stop only when d - L0 is decreasing
end