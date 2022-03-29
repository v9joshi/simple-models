% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_2DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m;
    
    % Unpack the state
    x = statevar(1); y = statevar(2); vx = statevar(3); vy = statevar(4);
    
    % Previous foot position
    oldFootX = statevar(5);
    
    % DAE formulation
    coeffs = [eye(4), [0; 0; -(x - oldFootX)/L0; -y/L0];
              zeros(1,4), 1/m];
    
    RHS = [vx; vy; 0; -g; g*y - vx^2 - vy^2];
    
    soln = coeffs\RHS;
    
    % Find the force along the leg
    % f = m*g*y + m*(vx^2 + vy^2);
    
    % Find the state derivative
    dStateVar = [soln(1:4); 0];    
end