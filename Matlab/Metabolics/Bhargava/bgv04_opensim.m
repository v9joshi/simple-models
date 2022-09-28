% Function to calculate the metabolic power of a given opensim model with
% dgf muscles given the states and controls at an instance of time.
function [costBreakdown, muscleStates] = bgv04_opensim(t, states, controls, osimModel, osimState)
    % Initialize state
    nMuscles  = osimModel.getNumControls;
    nStates   = osimState.getNY();
       
    % Muscle state value storage
    muscle_vel = zeros(1, nMuscles);
    muscle_len = zeros(1, nMuscles);
    
    muscle_force_Active  = zeros(1, nMuscles);
    muscle_force_Passive = zeros(1, nMuscles);
    
    fisoList =  zeros(nMuscles,1);
    
    % Muscle properties
    fMaxIsoList= zeros(nMuscles,1);    lceOptList = zeros(nMuscles,1);
    lSlackList = zeros(nMuscles,1);    vmaxList   = zeros(nMuscles,1);
    
    muscleMassList   = zeros(nMuscles,1);
    muscleFTratioList = zeros(nMuscles,1);
        
    % Muscle metabolics properties
    Aslow = zeros(nMuscles,1);     Afast = zeros(nMuscles,1);
    Mslow = zeros(nMuscles,1);     Mfast = zeros(nMuscles,1);
    
    % Initialize storage variables
    metabolics  = zeros(6, nMuscles);
    costBreakdown   = zeros(1,7);
    
    %% Import the OpenSim modeling classes
    import org.opensim.modeling.*
    
    % Set the time
    osimState.setTime(t);

    % Set state values
    y = Vector(nStates, 0.0);
    for i = 0:(nStates-1)
        y.set(i, states(i+1));
    end
    
    osimState.setY(y);

    % Update the state velocity calculations
    osimModel.computeStateVariableDerivatives(osimState);
    
    %% Calculate Muscle Forces and properties
    metabolicsProbe = osimModel.getProbeSet.get(1);
    probePointer = Bhargava2004MuscleMetabolicsProbe().safeDownCast(metabolicsProbe);
        
    % Go through all the muscles
    for currMuscle = 1:nMuscles
       % Get the selected muscle
       muscle = osimModel.getMuscles().get(currMuscle - 1);
       dgfMuscle = DeGrooteFregly2016Muscle.safeDownCast(muscle);
       muscleName = muscle.getName();
            
       % Get properties from the muscle
       fMaxIsoList(currMuscle)= dgfMuscle.get_max_isometric_force();
       lceOptList(currMuscle) = dgfMuscle.get_optimal_fiber_length();
       lSlackList(currMuscle) = dgfMuscle.get_tendon_slack_length();
       vmaxList(currMuscle)   = dgfMuscle.get_max_contraction_velocity();
       
       % Get physical properties from the probe
       muscleSTratio = probePointer.getRatioSlowTwitchFibers(muscleName);
       muscleFTratioList(currMuscle) = (1 - muscleSTratio);
       
       muscleMassList(currMuscle) = probePointer.getMuscleMass(muscleName);
       
       % Get metabolic properties from probe
       Afast(currMuscle) = probePointer.getActivationConstantFastTwitch(muscleName);
       Aslow(currMuscle) = probePointer.getActivationConstantSlowTwitch(muscleName);
       Mfast(currMuscle) = probePointer.getMaintenanceConstantFastTwitch(muscleName);
       Mslow(currMuscle) = probePointer.getMaintenanceConstantSlowTwitch(muscleName);
             
       % Get muscle state values
       fisoList(currMuscle)   = muscle.getActiveForceLengthMultiplier(osimState)*muscle.get_max_isometric_force()*muscle.getActivation(osimState);
       muscle_vel(currMuscle) = muscle.getFiberVelocity(osimState);
       muscle_len(currMuscle) = muscle.getFiberLength(osimState);
       
       muscle_force_Active(currMuscle)  = muscle.getActiveFiberForce(osimState);
       muscle_force_Passive(currMuscle) = muscle.getPassiveFiberForce(osimState);
       
       % Make an table of states for this muscle
       try
           muscleName = muscleName.toCharArray';
       catch
           muscleName = muscleName;
       end
     
       muscleStates.(muscleName) = table;
      
       % Store the states
       muscleStates.(muscleName).time = t;
       muscleStates.(muscleName).exc   = controls(currMuscle);
       muscleStates.(muscleName).l_ce  = muscle_len(currMuscle);
       muscleStates.(muscleName).v_ce  = muscle_vel(currMuscle);
       muscleStates.(muscleName).F_iso = fisoList(currMuscle);
       muscleStates.(muscleName).F_act = muscle_force_Active(currMuscle);
       muscleStates.(muscleName).F_pas = muscle_force_Passive(currMuscle);  
    end
    
    %% Calculate metabolic cost
    decay = 1.0;
    basalCoeff = probePointer.get_basal_coefficient;
    basalExp   = probePointer.get_basal_exponent;

    % Loop through each muscle
    for currMuscle = 1:nMuscles
        lceopt = lceOptList(currMuscle);

        lce = muscle_len(currMuscle);
        vce = muscle_vel(currMuscle);

        MusForce_active = muscle_force_Active(currMuscle);
        MusForce_total  = muscle_force_Active(currMuscle) + muscle_force_Passive(currMuscle);
        % f_iso = fisoList(currMuscle); % Only need this for force dependent shortening

        ft = muscleFTratioList(currMuscle);
        
        % excitations from the controls  >>>> change here
        excitation = controls(currMuscle); 
        % activationIndex = nStates - 2*nMuscles - 1;
        % activation = states(activationIndex+(2*currMuscle)); % activations from the states >>>> change here

        %=============Calculating=============
        lce_rel  = lce/lceopt;

        % Find activation cost
        excitation_FT = ft*(1 - cos(0.5*pi*excitation));
        excitation_ST = (1 - ft)*sin(0.5*pi*excitation);
        
        hdot_activation = decay*(excitation_FT*Afast(currMuscle) + excitation_ST*Aslow(currMuscle));
        
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
        
        hdot_maintenance = fiber_length_dependence*(excitation_FT*Mfast(currMuscle) + excitation_ST*Mslow(currMuscle));
        
        % Shortening cost
        if vce <= 0
            alpha = 0.25*MusForce_total;
        else
            alpha = 0;
        end   
        
        hdot_shortening = -alpha*vce/muscleMassList(currMuscle);
        
        % Mechanical work rate
        if vce <= 0
            wdot = -1.0 * ((vce * MusForce_active) / muscleMassList(currMuscle));
        else
            wdot = 0;
        end       
        
        wdot_include_neg = -1.0 * ((vce * MusForce_active) / muscleMassList(currMuscle));
        
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

        hdot_shortening_smooth = -alpha_smooth*vce/muscleMassList(currMuscle);

        % mechanical work        
        wdot_smooth = -1.0 * ((vce * MusForce_active) / muscleMassList(currMuscle));
                
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
        metabolics(1,currMuscle) = hdot_total;        % total power
        metabolics(2,currMuscle) = wdot;              % Mechanical work
        metabolics(3,currMuscle) = hdot_activation;   % activation heat
        metabolics(4,currMuscle) = hdot_maintenance;  % maintenance heat
        metabolics(5,currMuscle) = hdot_shortening;   % shortening heat
        metabolics(6,currMuscle) = hdot_total_smooth; % smoothed total power
    end
    
    % Basal Rate
    bDot = basalCoeff*(osimModel.getTotalMass(osimState)^basalExp);
    
    % Calculate total breakdown
    costBreakdown(1) = sum(metabolics(1,:)'.*muscleMassList) + bDot;
    costBreakdown(2) = sum(metabolics(2,:)'.*muscleMassList);
    costBreakdown(3) = sum(metabolics(3,:)'.*muscleMassList);
    costBreakdown(4) = sum(metabolics(4,:)'.*muscleMassList);
    costBreakdown(5) = sum(metabolics(5,:)'.*muscleMassList);
    costBreakdown(6) = sum(metabolics(6,:)'.*muscleMassList) + bDot;
    costBreakdown(7) = bDot;
end