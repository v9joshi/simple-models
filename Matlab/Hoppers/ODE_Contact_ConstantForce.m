% ODE file for contact phase of hopping with a constant force
function dstatevar = ODE_Contact_ConstantForce(~,statevar,params)

g = params.g; F0 = params.F0; L0 = params.L0; m = params.m;

y = statevar(1);
v = statevar(2);

dstatevar = [v; -g + F0/m];

end

