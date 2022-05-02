% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_2DTelescoping(t,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m;
    
    % Unpack the state
    x = statevar(1); y = statevar(2); vx = statevar(3); vy = statevar(4);
    
    % Foot position
    footX = statevar(5);
    footY = statevar(6);
    
    % Find the force components
    Fx = (x - footX)/L0;
    Fy = (y - footY)/L0;    
    
    % DAE formulation - coeffs*[vx; vy; ax; ay; f/m] = RHS
    xdot    = vx;
    ydot    = vy;
    xdotdot = Fx/m;
    ydotdot = Fy/m - g;    
    
    % Find the state derivative
    dStateVar = [xdot; ydot; xdotdot; ydotdot; 0; 0];    
end