% Hill type muscle force with a given activation
function force = hillMuscle(activation, L, Ldot, Fmax, W, kSE)
    % Calculate the force
    FCE = activation*Fmax*exp(-((L - 1)/W)^2);
    FSE = kSE*(


end