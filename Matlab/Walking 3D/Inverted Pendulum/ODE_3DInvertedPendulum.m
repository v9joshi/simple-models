% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_3DInvertedPendulum(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m;
    
    % Unpack the state
     x = statevar(1);  y = statevar(2);  z = statevar(3);
    vx = statevar(4); vy = statevar(5); vz = statevar(6);
    
    % Previous foot position
    oldFootX = statevar(7);
    oldFootY = statevar(8);
    oldFootZ = statevar(9);
    
    oldFootXdot = params.speedX;
    oldFootYdot = params.speedY;
    oldFootZdot = params.speedZ;
    
    % Find the force along the leg
    % f = (m*g*y - m*(vx^2 + vy^2 + vz^2))/L;
    
    % DAE formulation - coeffs*[vx; vy; vz; ax; ay; az; f/m] = RHS
    coeffs = [eye(6), [0; 0; 0; -(x - oldFootX)/L0;  -(y - oldFootY)/L0; -(z - oldFootZ)/L0;];
              zeros(1,6), L0];
    
    RHS = [vx; vy; vz; 0; -g; 0; g*y - (vx - oldFootXdot)^2 - (vy - oldFootYdot)^2 - (vz - oldFootZdot)^2];
    
    soln = coeffs\RHS;
    
    % Find the state derivative
    dStateVar = [soln(1:6); oldFootXdot; oldFootYdot; oldFootZdot];    
end