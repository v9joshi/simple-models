% State transition matrix for the 2D compass walker. 
function newStatevar = Contact_2DCompass(~, statevar, params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; M = params.M;
    
    % Unpack the state
    x1  = statevar(1); x2  = statevar(2); 
    y1  = statevar(3); y2  = statevar(4); 
    vx1 = statevar(5); vx2 = statevar(6);
    vy1 = statevar(7); vy2 = statevar(8);
    
    % Foot position
    x0  = statevar(9); y0  = statevar(10);
    vx0 =           0; vy0 =            0;
    
    % Initialize the new states
    newStatevar = statevar;
        
    % Angular momentum balance about the HAT
    oldAM_HAT   = 0;
    
    % Angular momentum balance about the swing foot
    oldAM_Swing = M*(x1 - x2)*vy1 - M*(y1 - y2)*vx1;
    
    % Solve for new values of vx0,vx1, vy0 amd vy1
    coeffs = [ -m*(y0 - y1),            0,  m*(x0 - x1),            0;  % New HAT angular momentum
               -m*(y0 - y2), -M*(y1 - y2),  m*(x0 - x2),  M*(x1 - x2);  % New swing foot angular momentum
                  (x0 - x1),    (x1 - x0),    (y0 - y1),    (y1 - y0);  % leg length constraint 1 
                          0,    (x1 - x2),            0,    (y1 - y2)]; % leg length constraint 2


    RHS = [oldAM_HAT; oldAM_Swing; 0; 0];
    
    soln = coeffs\RHS;
    
    % Swap the stance and swing foot positions
    newStatevar(2) = x0;
    newStatevar(4) = y0;
    
    newStatevar(9)  = x2;
    newStatevar(10) = y2;
    
    % Set new CoM velocities
    newStatevar(5) = soln(2);
    newStatevar(7) = soln(4);
    
    % Set the new swing foot velocities
    newStatevar(6) = soln(1);
    newStatevar(8) = soln(3);
end