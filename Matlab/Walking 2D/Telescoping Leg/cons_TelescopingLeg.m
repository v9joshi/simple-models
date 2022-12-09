function [ineqCons, eqCons] = cons_TelescopingLeg(input, params)   
    % Unpack parameters
    nSteps      = params.nSteps;
    nSeg        = params.nSeg;
    nVars       = params.nVars;
    nStates     = params.nStates;
    nInputs     = params.nInputs;
    L0          = params.L0;
    Fmax        = params.Fmax;
    Speed       = params.Speed;
    stepLength  = params.stepLength;
    stepTime   = stepLength/Speed;
    
    % Find the resulting states
    [tStore, outputStateStore] = nSteps_TelescopingLeg(input, params);
    
    % Remove the work done variable
    outputStateStore = outputStateStore(:,1:nStates);
    
    % Make an empty storage matrix
    inputStateStore = [];
    
    % Reshape the inputs to read them
    stepVars = reshape(input, nVars, nSteps); 
    
    % Loop through the steps
    for stepNum = 1:nSteps
       currStepVar = stepVars(:, stepNum);
       
       % Read the impulses and foot positions
       currXf = currStepVar(end-1);
       currYf = currStepVar(end);

       % Read the seg vars for each segment
       segVars = reshape(currStepVar(1:end-2)', nSeg, nInputs);  
       
       for segNum = 1:nSeg
          currSegVar =  segVars(segNum,:);
          
          % Read delF and delT
          currDelF   =  currSegVar(end-1);
          currDelT   =  currSegVar(end)*stepTime;
          
          % Read the states
          currSegState = [currSegVar(1:end-2), currXf, currYf];
          inputStateStore = [inputStateStore; currSegState];
       end
    end
    
    % Use the parameters to generate constraints
    % eq 1.0 Start at the origin
    % Set the first foot position to be (0,0)
    startPos = inputStateStore(1,6:7);
    startCons = startPos(:);
    
    % eq 2.0 Continuity across segments
    % the end of one segment should match the beginning of the next
    continuityCons = [];
    
    for currStep = 1:nSteps
        indexStart = nSeg*(currStep - 1);
        indexEnd = nSeg*currStep;
        % End of one segment is the beginning of the next segment
        continuityCons1 = outputStateStore(indexStart+1:indexEnd-1,:) - inputStateStore(indexStart+2:indexEnd,:);
        continuityCons = [continuityCons; continuityCons1(:)];
        
        % The position and velocity at the end of one step should match
        % that at the beginning of the next step.
        if currStep + 1 <= nSteps
           continuityCons2 = outputStateStore(indexEnd,1:4) - inputStateStore(indexEnd+1,1:4);
           continuityCons = [continuityCons; continuityCons2(:)];
        end
    end  

    % eq 3.0 Periodiciy of state from beginning to end
    % All states except forward position of CoM, and foot position
    % should be the same at the start as at the end.
    periodicityCons = outputStateStore(end,2:end-2) - inputStateStore(1,2:end-2);
    periodicityCons = periodicityCons(:);
    
    % eq 4.0 Make sure the speed matches the set value
    speedVal = (outputStateStore(end,1) - inputStateStore(1,1))/(tStore(end) - tStore(1));
    speedCon = speedVal - Speed;
    
    % eq 5.0 Symmetry?
    step1Displacement  = outputStateStore(nSeg,1) - inputStateStore(1,1);
    step2Displacement  = outputStateStore(end,1) - inputStateStore(nSeg+1,1);
    symmetryCon        = step1Displacement - step2Displacement;
            
    % ineq 1.0 Leg length should not exceed max leg length
    % Measure leg length as (x - xf)^2 + (y - yf)^2
    legLengthInit = (inputStateStore(1,1) - inputStateStore(1,6)).^2 + (inputStateStore(1,2) - inputStateStore(1,7)).^2;
    legLengthOut  = (outputStateStore(:,1) - outputStateStore(:,6)).^2 + (outputStateStore(:,2) - outputStateStore(:,7)).^2;
    legLengthConUp  = [legLengthInit; legLengthOut] - L0^2;
    
    % ineq 2.0 Leg length should not be less than 0.7*L0
    legLengthConLo  = (0.7*L0)^2 - [legLengthInit; legLengthOut];
    legLengthCon  = [legLengthConUp(:); legLengthConLo];
    
    % ineq 3.0 Force should not exceed max force
    % Force grows linearly, so if input and output don't exceed max
    % no force can exceed max - already handled by UB and LB
    % forceVal = [inputStateStore(1,5); outputStateStore(:,5)];
    % forceCon = forceVal - Fmax;
    
    % ineq 4.0 Step length matches target
    measuredStepLength  = outputStateStore(end,6) - inputStateStore(1,6);
%     strideLength        = outputStateStore(end,1) - inputStateStore(1,1);
    stepLengthCon       = stepLength - measuredStepLength;
            
    % Put these together
    eqCons   = [startCons; continuityCons; periodicityCons; speedCon; symmetryCon];
    ineqCons = [legLengthCon; stepLengthCon];% forceCon];
end