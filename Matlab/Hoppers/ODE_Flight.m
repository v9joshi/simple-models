% ODE file for flight phase of hopping
function dstatevar = ODE_Flight(~,statevar,params)
    % Unpack the important parameters
    g = params.g;

    % Read the state
    y = statevar(1);
    v = statevar(2);

    % Calculate the state derivative
    dstatevar = [v; -g];
end

