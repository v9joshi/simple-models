function [tStoreStep, stateStore] = singleStep_TelescopingLeg(input,params)
    % Read the foot position
    params.footX = input(end-1);
    params.footY = input(end);  
    
    % Re-shape the remaining state according to the number of segments
    nSeg        = params.nSeg;
    nStates     = params.nStates;
    nMeasures   = params.nMeasures;
    nInputs     = params.nInputs;
    newInput    = reshape(input(1:end-2)', nSeg, nInputs);  
            
    % Make storage arrays
    tStoreStep = zeros(nSeg,1);
    stateStore = zeros(nSeg, nMeasures);
            
    % Loop through the segments
    parfor segNum = 1:nSeg
        currInput = newInput(segNum,:);
        
        [tlistOut, statesOut] = simulateSegment(currInput, params);
                        
        % Store the results of the simulation
        tStoreStep(segNum) = tlistOut(end);
        stateStore(segNum,:) = statesOut(end,:);
    end            
end