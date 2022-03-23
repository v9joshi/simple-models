% ODE file for contact phase of hopping with Sinusoidal Contact Force
function dstatevar = ODE_Contact_SinusoidalForce(t,statevar,params)

g = params.g; F0 = params.F0; L0 = params.L0; m = params.m; T0 = params.T0;

F = F0*sin(t/T0);

y = statevar(1);
v = statevar(2);

dstatevar = [v; -g + F/m];

end

