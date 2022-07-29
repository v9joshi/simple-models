% This hill muscle model assumes dynamic equillibrium between tendon and
% contractile element, i.e. the force in the tendon equals the force in the
% muscle.
function [ccForce, ccVelocity, aDot] = hillMuscle(excitation, activation, musLen, mtuLen, params)
    %% Unpack the parameters
    % Activation dynamics
    tau_act = params.tau_act;
    tau_deact = params.tau_deact;
    
    % General tendon parameters
    tendonStiffness   = params.tendonStiffness;   % Typically 1e6
    tendonConstant    = params.tendonConstant;    % Constant term in eqn ~ 0.0258
    tenSlackLength    = params.tenSlackLength;       % The length of an the unstretched tendon
        
    % General muscle parameters
    optMusFibLength   = params.optMusFibLength;      % probably 10 or 12
    maxIsoForce       = params.maxIsoForce;       % Depends on the muscle
    
%     fracSlowTwitch    = params.fracSlowTwitch;    % Default 0.5?
%     fracFastTwitch    = 1 - fracSlowTwitch;

    % Hill muscle parameters for CE
    % F-V curve parameters
    hill_a            = params.hill_a;            % A = 0.25?
    hill_b            = params.hill_b;            % Related to vmax
    
    % F-L curve parameters
    hill_c            = params.hill_c;            % Width = 0.63?
        
    % Max shortening rate = maxIsoForce*b/a, typically about 1.5 for slow
    % twitch and 3.75 for fast twitch muscles
%     maxShorteningRate = params.maxShorteningRate; 
    
    % Eccentric force enhancement
    eccentricEnhance  = params.eccentricForceEnhancement; % Can be 1.1 ish?
    
    %% Ensure activation is non-zero
    activation = max(activation, 1e-3); % This prevents divide by 0 issues
    
    %% Use mtu length and muscle length to get tendon length
    tenLen = mtuLen - musLen;
    
    % Use tendon stiffness parameter to calculate muscle force
    tenScaler = tendonConstant*exp(tendonStiffness*(tenLen/tenSlackLength - 1)) - tendonConstant;
    tenForce  = max(maxIsoForce*tenScaler, 0);
    
    %% Use muscle length to get scaling parameter
    lenNormalized = musLen/optMusFibLength;
    lenScaling    = hill_c*(lenNormalized - 1)^2 + 1;
    
    % Find velocity scaling term using total force, activation and length scaling
    ccForce = tenForce; % Muscle and tendon are in series
    reducedIsoForce = lenScaling*activation*maxIsoForce; % The stretched muscle with the given muscle activation has a lower maximum force

    % Now we can use the velocity scaling to estimate the velocity    
    if ccForce < reducedIsoForce
        % Use concentric contraction equation - muscle shortening
        % opt 1. Linear velocity scaling: 
        %        Force goes down linearly with velocity upto ~ maxShorteningRate
        %        max shortening rate in opt fiber length/s
        %        velScaling = (velNormalized - vmax)*1/vmax
        % velocityNormalized  = maxShorteningRate - velScaling*maxShorteningRate;

        % opt 2 rectangular hyperbola
        % (P + a)*(v + b) = (P0 + a)*b
        % Note: hill_b = maxShorteningRate*hill_a/maxIsoForce
        ccVelocity  = hill_b*(reducedIsoForce - ccForce)/(ccForce + hill_a) - hill_b;
        
    else
        % Use eccentric contraction equation - muscle lengthening while producing force
        hill_S = hill_b*reducedIsoForce*(eccentricEnhance - 1)/(reducedIsoForce + hill_a);
        ccVelocity  = hill_S - hill_S*(eccentricEnhance - 1)/(eccentricEnhance - ccForce/reducedIsoForce);
    end
        
    % Activation dynamics
    c2 = 1/(tau_deact);
    c1 = 1/(tau_act) - c2;
    
    aDot = (c1*excitation + c2)*(excitation - activation);
end