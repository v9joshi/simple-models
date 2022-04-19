% ODE file for contact phase of hopping with a springy leg
function dstatevar = ODE_Contact_SpringForce(~,statevar,params)

    % Unpack the important parameters
    g = params.g; k = params.k; L0 = params.L0; m = params.m;

    % Read the state
    y = statevar(1);
    v = statevar(2);

    % Calculate the contact force
    F = k*(L0 - y);
    
    % Calculate the state derivative
    dstatevar = [v; -g + F/m];
end

