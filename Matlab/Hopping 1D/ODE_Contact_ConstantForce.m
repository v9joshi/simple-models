% ODE file for contact phase of hopping with a constant contact force
function dstatevar = ODE_Contact_ConstantForce(~,statevar,params)

    % Unpack the important parameters
    g = params.g; F0 = params.F0; L0 = params.L0; m = params.m;

    % Read the state
    y = statevar(1);
    v = statevar(2);
    
    % Calculate the contact force
    F = F0;

    % Calculate the state derivative
    dstatevar = [v; -g + F/m];    
end

