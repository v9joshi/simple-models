function [value,isterminal,direction] = Event_FlightToContact(~,statevar,params)
    %ODE1_FLIGHT 
    
    % Unpack the parameters of interest
    L0 = params.L0;

    % Read the state
    x = statevar(1);            vx = statevar(3);
    y = statevar(2);            vy = statevar(4);
    xf = statevar(5);           yf = statevar(6);

    % Calculate leg length
    L = sqrt((x - xf)^2 + (y - yf)^2);
    
    % Put the event together
    value = L-L0;   % what goes to zero when the event happens
    isterminal = 1; % whether the simulation should be stopped or not
    direction = -1; % we want to stop only when L-L0 is decreasing
end

