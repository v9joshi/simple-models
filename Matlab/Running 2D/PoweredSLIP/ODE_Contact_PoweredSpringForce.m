% ODE file for contact phase of hopping with an actuated springy leg
function dstatevar = ODE_Contact_PoweredSpringForce(~,statevar,params)
    % Unpack the important parameters
    g = params.g; k = params.k; L0 = params.L0; m = params.m;
    delF = params.delF;

    % Read the state
    x = statevar(1);            vx = statevar(3);
    y = statevar(2);            vy = statevar(4);
    xf = statevar(5);           yf = statevar(6);

    % Calculate the contact force
    L = sqrt((x - xf)^2 + (y - yf)^2);
    Fspring = k*(L0 - L);
    FAct    = statevar(7);
    
    % Work done by actuator
    lDot     = (x - xf)*vx + (y - yf)*vy;
    power    = FAct*lDot;
    power    = -0.8*0.5*(1 - tanh(1000*power))*power + 4*0.5*(1 + tanh(1000*power))*power;
        
    % Get force components
    Fx = Fspring*(x - xf)/L;
    Fy = Fspring*(y - yf)/L;
    
    % Calculate the state derivative
    dstatevar = [vx; vy; Fx/m; -g + Fy/m; 0; 0; delF; power];
end

