% Function to calculate the metabolic power of a given opensim model with
% dgf muscles given the states and controls at an instance of time.
% Using the Umberger muscle metabolics model.
function metabolicPower = umb22_SingleMuscle(muscleParameters, muscleStates)
    % Muscle properties
    lceopt       = muscleParameters.LceOpt;
    ft           = muscleParameters.FTratio;
    muscleMass   = muscleParameters.Mass; 
    
    vmax_ft = muscleParameters.Vmax;
    vmax_st = muscleParameters.Vmax*(0.05^0.3);
    
    % Metabolic properties
    aerobicScalingFactor = 1.5; % muscleParameters.aerobicScalingFactor;
    activationExponent   = 1;   % muscleParameters.activationExponent;
    
    % Muscle states
    lce = muscleStates.l_ce;
    vce = muscleStates.v_ce;
  
    MusForce_active     = muscleStates.F_act;
    MusForce_multiplier = muscleStates.F_act/muscleStates.F_iso;
    
    excitation = muscleStates.exc;

    % Calculate muscle power
    lce_rel  = lce/lceopt;

%     % Uchida correction
%     excitation_FT = ft*(1 - cos(0.5*pi*excitation));
%     excitation_ST = (1 - ft)*sin(0.5*pi*excitation);
    
    % Find activation and maintenance cost
    % Fiber length dependence
    hdot_activation     = 1.28*ft*100 + 25.0;
    hdot_scaling_factor = excitation^activationExponent;
    
    if lce_rel <= 1.0
        fiber_length_dependence = 1;
    else
        fiber_length_dependence = (0.4 + 0.6*MusForce_multiplier);
    end      

    hdot_activation = fiber_length_dependence*hdot_activation*hdot_scaling_factor*aerobicScalingFactor;

    % Mechanical work rate
    if vce <= 0
        wdot = -1.0 * (vce * MusForce_active) / muscleMass;
    else
        wdot = 0;
    end       

    % Clamp muscle power to 1.0 W/kg minimum
    hdot_muscle = hdot_activation;
    hdot_muscle = max(1.0, hdot_muscle);

    % Add it all up
    hdot_total = wdot + hdot_muscle;
    
    % Store the results
    metabolicPower(1) = hdot_total*muscleMass;        % total power
    metabolicPower(2) = wdot*muscleMass;              % Mechanical power
    metabolicPower(3) = hdot_activation*muscleMass;   % Activation power
    metabolicPower(4) = 0;                            % Shortening power
    metabolicPower(5) = hdot_muscle*muscleMass;       % Muscle power
end