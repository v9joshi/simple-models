function [tStore, stateStore] = singleStep_TelescopingLeg(input,params)
    % Read the foot position
    params.footX = input(end-3);
    params.footY = input(end-2);  
    
    % Read the impulses
    hsImpulse = input(end-1);
    poImpulse = input(end);  
    
    % Re-shape the remaining state according to the number of segments
    nSeg = params.nSeg;
    nStates = params.nStates;
    nInputs = params.nInputs;
    newInput = reshape(input(1:end-4)', nSeg, nInputs);  
            
    % Make storage arrays
    tStore = [];
    stateStore = [];
    
    % Apply the heel-strike impulse     
%     currState = [newInput(1, 1:nInputs - 2), params.footX, params.footY];
%     postHS = applyImpulse(currState, hsImpulse/params.m);
    
    % Replace the initial state with the post-hs state
%     newInput(1, 1:nStates-2) = postHS(1:nStates-2);
    
    % Store the result
    % tStore = 0;
    % stateStore = postHS;
        
    % Loop through the segments
    for segNum = 1:nSeg
        currInput = newInput(segNum,:);
        
        [tlistOut, statesOut] = simulateSegment(currInput, params);
        
        % Shift the time forward based on previous segments
        if ~isempty(tStore)
            tlistOut = tlistOut + tStore(end);
        end
                
        % Store the results of the simulation
        tStore = [tStore, tlistOut(end)];       
        stateStore = [stateStore; statesOut(end,:)];        
    end
        
    % Apply the push-off impulse
%     currState = stateStore(end,:);
%     postPO = applyImpulse(currState, poImpulse/params.m);
%     
    % Store the post-po state
%     stateStore(end,:) = postPO;
end