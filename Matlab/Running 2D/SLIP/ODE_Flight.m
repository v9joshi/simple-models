% ODE file for flight phase of hopping
function dstatevar = ODE_Flight(~,statevar,params)
    % Unpack the important parameters
    g = params.g;

    % Read the state
    x = statevar(1);            vx = statevar(3);
    y = statevar(2);            vy = statevar(4);
    xf = statevar(5);           yf = statevar(6);

    % Calculate the state derivative
    dstatevar = [vx; vy; 0; -g; 0; 0];
end

