% Function to apply heel-strike and push-off impulses to a 2D inverted
% pendulum walker and ensure it continues to move in a 2D inverted
% pendulum.
function stateOut =  Contact_2DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; stepLength = params.stepLength;    
        
    % Unpack the state
    x = statevar(1); y = statevar(2); vx = statevar(3); vy = statevar(4);
    
    % Old foot position
    oldFootX = statevar(5);
    newFootX = oldFootX + params.stepLength;
           
    % Form the new state
    stateOut = [x; y; vx; -vy; newFootX];
end