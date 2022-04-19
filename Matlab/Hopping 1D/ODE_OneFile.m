% ODE file for hopping motion using if statements rather than event
% detection
function dstatevar = ODE_OneFile(t,statevar,params)
% Unpack
m = params.m; k = params.k; L0 = params.L0; g = params.g;

y = statevar(1);
v = statevar(2);

if y > L0
    dstatevar = [ v; -g];
else
    dstatevar = [ v; -g + k/m*(L0-y)];
end

end

