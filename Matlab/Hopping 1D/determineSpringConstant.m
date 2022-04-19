% solve for the perfect spring constant so that the spring hopper has the
% same time constant as the constant hopper
kStart = 2800;
[kIdeal, errorVal] = fsolve(@springHopperTimeError, kStart);

function timeError = springHopperTimeError(springConstant)
    k = springConstant;
    y0 = 1.5; v0 = 0; % Initial state
    L0 = 1;  % Length of the leg
    m  = 70; % body mass
    g  = 10; % gravitational acc
    F0_const = 1400;
    
    springHopper     = Root_Hopping_SpringForce(y0, v0, m, g, L0, k, 0);
    constantHopper   = Root_Hopping_ConstantForce(y0, v0, m, g, L0, F0_const, 0);
    
    perfectTime = constantHopper.tlist(end);
    currentTimeConst = springHopper.tlist(end);
    
    timeError = currentTimeConst - perfectTime;
end