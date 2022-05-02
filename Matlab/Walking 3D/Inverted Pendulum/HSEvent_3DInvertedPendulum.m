% Function to detect the heel-strike event for a 2D point-mass inverted
% pendulum walker. This event also checks for falls.
function [value,isterminal,direction] = HSEvent_3DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; 
    stepLength = params.stepLength; stepWidth = params.stepWidth;
    
    % Unpack the state
     x = statevar(1);  y = statevar(2);  z = statevar(3);
    vx = statevar(4); vy = statevar(5); vz = statevar(6);
    
    % Previous foot position
    oldFootX = statevar(7);
    oldFootY = statevar(8);
    oldFootZ = statevar(9);
    
    % New foot position
    newFootX = oldFootX + stepLength;
    newFootY = oldFootY;
    newFootZ = -oldFootZ;
    
    % Distance from next foot position
    d = sqrt((x - newFootX)^2 + (y - newFootY)^2 + (z - newFootZ)^2);
    
    % Evaluate the heel-strike condition
    value      = [d - L0; y];   % what goes to zero when the event happens
    isterminal = [1; 1];        % whether the simulation should be stopped or not
    direction  = [-1; -1];      % we want to stop only when d - L0 is decreasing
end