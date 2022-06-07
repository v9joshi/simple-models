% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_2DTelescoping(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m;
    delF = params.delF; delT = params.delT;
    
    % Unpack the state
    x = statevar(1); y = statevar(2); vx = statevar(3); vy = statevar(4);
    
    % Calculate the leg force
    Fleg = statevar(5);
    
    % Foot position
    footX = statevar(6);
    footY = statevar(7);
    
    % Work cost
    workDone = statevar(8);
    
    % Find the force components
    Fx = (x - footX)*Fleg/L0;
    Fy = (y - footY)*Fleg/L0;    
    
    % DAE formulation - coeffs*[vx; vy; ax; ay; f/m] = RHS
    xdot    = vx;
    ydot    = vy;
    xdotdot = Fx/m;
    ydotdot = Fy/m - g;
    
    % Find the leg length rate
    ldot = (x - footX)*vx + (y - footY)*vy;
    power = ldot*Fleg;
    power = -0.8*0.5*(1 - tanh(1000*power))*power + 4*0.5*(1 + tanh(1000*power))*power;
    
    % Find the state derivative
    dStateVar = [xdot; ydot; xdotdot; ydotdot; delF/delT; 0; 0; power];
end