function [value,isterminal,direction]  = Event_FlightToContact(t,statevar,params)
%ODE1_FLIGHT 

L0 = params.L0;

y = statevar(1);
v = statevar(2);

value = y-L0;  % what goes to zero when the event happens
isterminal = 1; % whether the simulation should be stopped or not
direction = -1; % we want to stop only when y-L0 is decreasing

end

