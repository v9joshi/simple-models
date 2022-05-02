% Function to apply heel-strike and push-off impulses to a 2D inverted
% pendulum walker and ensure it continues to move in a 2D inverted
% pendulum.
function stateOut =  Contact_3DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; 
    stepLength = params.stepLength; stepWidth = params.stepWidth;
        
    % Unpack the state
    x = statevar(1);   y = statevar(2);  z = statevar(3); 
    vx = statevar(4); vy = statevar(5); vz = statevar(6);
    
    % Old foot position
    oldFootX = statevar(7); oldFootY = statevar(8); oldFootZ = statevar(9);
    newFootX =  oldFootX + stepLength;
    newFootY =  oldFootY;
    newFootZ = -oldFootZ;    
           
    % Form the new state
    if abs(z - newFootZ) > 0
        vz = (-vx*(x - newFootX) - (-vy)*(y - newFootY))/(z - newFootZ);
    else
        vz = 0;
    end
    
    stateOut = [x; y; z; vx; -vy; vz; newFootX; newFootY; newFootZ];
end