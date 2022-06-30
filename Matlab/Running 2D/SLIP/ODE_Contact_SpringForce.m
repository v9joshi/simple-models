% ODE file for contact phase of hopping with a springy leg
function dstatevar = ODE_Contact_SpringForce(~,statevar,params)
    % Unpack the important parameters
    g = params.g; k = params.k; L0 = params.L0; m = params.m;

    % Read the state
    x = statevar(1);            vx = statevar(3);
    y = statevar(2);            vy = statevar(4);
    xf = statevar(5);           yf = statevar(6);

    % Calculate the contact force
    L = sqrt((x - xf)^2 + (y - yf)^2);
    F = k*(L0 - L);
    
    % Get force components
    Fx = F*(x - xf)/L;
    Fy = F*(y - yf)/L;
    
    % Calculate the state derivative
    dstatevar = [vx; vy; Fx/m; -g + Fy/m; 0; 0];
end

