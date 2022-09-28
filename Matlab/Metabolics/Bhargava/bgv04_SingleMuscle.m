% Function to calculate the metabolic power of a given opensim model with
% dgf muscles given the states and controls at an instance of time.
function metabolicPower = bgv04_SingleMuscle(muscleParameters, muscleStates)
    % Muscle properties
    decay        = 1.0;
    lceopt       = muscleParameters.LceOpt;
    ft           = muscleParameters.FTratio;
    muscleMass   = muscleParameters.Mass; 
    
    % Metabolic properties
    Afast  = muscleParameters.Afast;     Aslow  = muscleParameters.Aslow; 
    Mfast  = muscleParameters.Mfast;     Mslow  = muscleParameters.Mslow; 
    
    % Muscle states
    lce = muscleStates.l_ce;
    vce = muscleStates.v_ce;

    MusForce_active = muscleStates.F_act;
    MusForce_total  = muscleStates.F_act + muscleStates.F_pas;
    
    excitation = muscleStates.exc;

    % Calculate muscle power
    lce_rel  = lce/lceopt;

    % Find activation cost
    excitation_FT = ft*(1 - cos(0.5*pi*excitation));
    excitation_ST = (1 - ft)*sin(0.5*pi*excitation);

    hdot_activation = decay*(excitation_FT*Afast + excitation_ST*Aslow);

    % Find maintenance cost
    % Fiber length dependence
    if lce_rel < 0.5
        fiber_length_dependence = 0.5;
    elseif lce_rel < 1.0
        fiber_length_dependence = lce_rel;
    elseif lce_rel < 1.5
        fiber_length_dependence = -2*lce_rel + 3;
    elseif lce_rel < 2.0
        fiber_length_dependence = 0.0;
    end      

    hdot_maintenance = fiber_length_dependence*(excitation_FT*Mfast + excitation_ST*Mslow);

    % Shortening cost
    if vce <= 0
        alpha = 0.25*MusForce_total;
    else
        alpha = 0;
    end   

    hdot_shortening = -alpha*vce/muscleMass;

    % Mechanical work rate
    if vce <= 0
        wdot = -1.0 * (vce * MusForce_active) / muscleMass;
    else
        wdot = 0;
    end       

    wdot_include_neg = -1.0 * (vce * MusForce_active) / muscleMass;

    hdot_muscle = hdot_activation + hdot_maintenance + hdot_shortening;

    % Check negative total power and prevent it
    if (hdot_muscle + wdot_include_neg) < 0
        hdot_muscle = hdot_activation + hdot_maintenance + hdot_shortening - (hdot_muscle + wdot_include_neg);
    end

    % Add it all up
    hdot_total = wdot_include_neg + max(1.0, hdot_muscle);

    % If we want to apply tanh smoothing
    % shortening heat
    smooth_velocity = 0.5 + 0.5*tanh(10*vce);
    alpha_low  = 0.25*MusForce_total;
    alpha_high = 0;
    alpha_smooth = alpha_low + (alpha_high - alpha_low)*smooth_velocity;

    hdot_shortening_smooth = -alpha_smooth*vce/muscleMass;

    % mechanical work        
    wdot_smooth = -1.0 * (vce * MusForce_active) / muscleMass;

    % Calculate total power
    hdot_total_smooth = hdot_activation + hdot_maintenance + hdot_shortening_smooth +wdot_smooth;

    smooth_power = 0.5 + 0.5*tanh(10*(-hdot_total_smooth));
    hdot_low  = 0;
    hdot_high = hdot_total_smooth;
    tmp = hdot_low + (hdot_high - hdot_low)*smooth_power;

    % Apply this correction if total power goes negative
    hdot_shortening_smooth = hdot_shortening_smooth - tmp;

    % Prevent total heat rate from dropping below 1.0
    hdot_muscle = hdot_activation + hdot_maintenance + hdot_shortening_smooth;
    smooth_hdot = 0.5 + 0.5*tanh(10*(hdot_muscle - 1));
    hdot_muscle_high = hdot_muscle;
    hdot_muscle_low  = 1;
    hdot_muscle_smooth = hdot_muscle_low + (hdot_muscle_high - hdot_muscle_low)*smooth_hdot;

    % Find the total power
    hdot_total_smooth = hdot_muscle_smooth + wdot_smooth;

    % Store the results
    metabolicPower(1) = hdot_total*muscleMass;        % total power
    metabolicPower(2) = wdot*muscleMass;              % Mechanical work
    metabolicPower(3) = hdot_activation*muscleMass;   % activation heat
    metabolicPower(4) = hdot_maintenance*muscleMass;  % maintenance heat
    metabolicPower(5) = hdot_shortening*muscleMass;   % shortening heat
    metabolicPower(6) = hdot_total_smooth*muscleMass; % smoothed total power  
end