% Event file to detect collision of the swing foot with the ground
function [value, isterminal, direction] = HSEvent_2DCompass(~, statevar, params)
    % Unpack the useful parameters
    gamma = params.gamma;
    
    % Stance foot position
    x0  = statevar(9); y0  = statevar(10);
    
    % CoM position
    x1  = statevar(1); y1  = statevar(3);
    
    % Swing foot position
    x2  = statevar(2); y2  = statevar(4);
    
    % If the swing foot is behind the stance foot, the angle will be 
    % -gamma, if the swing foot is in front, it will be +gamma.
    % In all other cases, the foot is not on the ground.
    expectedXFoot = (y2 - y0)/tan(gamma);
    
    % In addition, the stance angle and swing angle must be equal and
    % non-zero.
    compassAngleStance = atan2(y0 - y1, x0 - x1);
    compassAngleSwing  = atan2(y2 - y1, x2 - x1);
    
    angleCond = (compassAngleSwing - compassAngleStance) - 2*(-pi/2 - compassAngleStance);
    
    % Set the event parameters.
    % When the foot angle reaches +gamma this simulation should terminate.
    value = (x2 - x0 -  expectedXFoot);         % Is the foot angle gamma? 
    isterminal = expectedXFoot > (x0 + 0.05);            % Terminate at collision.
    direction  = 0;                             % Foot must be swinging back
end