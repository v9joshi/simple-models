% ODE file for contact phase of hopping
function dstatevar = ODE_Contact_SpringForce(~,statevar,params)

g = params.g; k = params.k; L0 = params.L0; m = params.m;

y = statevar(1);
v = statevar(2);

dstatevar = [v; -g + k*(L0-y)/m];

end

