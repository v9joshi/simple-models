% ODE file for flight phase of hopping
function dstatevar = ODE_Flight(t,statevar,params)
g = params.g;

y = statevar(1);
v = statevar(2);

dstatevar = [v; -g];

end

