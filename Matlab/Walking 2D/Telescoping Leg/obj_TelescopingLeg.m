function objVal = obj_TelescopingLeg(input, params)
    % Unpack parameters
    nSteps  = params.nSteps;
    nSeg    = params.nSeg;
    nVars   = params.nVars;
    nStates = params.nStates;
    nInputs = params.nInputs;
    L0      = params.L0;
    Fmax    = params.Fmax;
    Speed   = params.Speed;
        
    % Make an empty storage matrix
    inputStateStore = [];
    impulseStore    = [];
    legLengthRate   = [];
    timeStore       = [];
    delFStore       = [];
    
    % Reshape the inputs to read them
    stepVars = reshape(input, nVars, nSteps); 
    currTime = 0;
    
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
          currDelT   =  currSegVar(end);
          
          delFStore = [delFStore; currDelF];
          
          % Save the time
          timeStore = [timeStore; currTime];
          
          % Update the time
          currTime = currTime + currDelT;
                    
          % Read the states
          currSegState = [currSegVar(1:end-2), currXf, currYf];
          inputStateStore = [inputStateStore; currSegState];
          
          % Curr leg length rate
          currX = currSegState(1);
          currY = currSegState(2);
          currVx = currSegState(3);
          currVy = currSegState(4);
          
          currLLrate = (currX - currXf)*currVx + (currY - currYf)*currVy;
          legLengthRate = [legLengthRate; currLLrate];
       end
    end
    
    % Run the simulation  
    [~, outputStateStore] = nSteps_TelescopingLeg(input, params);
    
    % Whats the step length?
    stepLengthVal = outputStateStore(end,1) - inputStateStore(1,1);
    
    % What are the input forces?
    Fval = inputStateStore(:,5) + 0.5*delFStore;
    
    % Calculate the work done by the leg forces
    legWork = outputStateStore(:,end);
        
    % Sum and square the forces
    objVal = sum(legWork); %sum(Fval.^2) + sum(impulseStore.^2);
    objVal = objVal/stepLengthVal;
end