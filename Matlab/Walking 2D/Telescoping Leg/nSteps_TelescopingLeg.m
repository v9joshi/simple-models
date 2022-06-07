function [tStore, stateStore] = nSteps_TelescopingLeg(input, params)
    % Use persistant variables to reduce function calls
    persistent prevInput prevOutputState prevOutputTime

    % Check if this set of inputs has already been evaluated
    if isequal(input, prevInput)
        stateStore = prevOutputState;
        tStore = prevOutputTime;
    % If not, then evaluate it and store the results
    else
        % Unpack some parameters
        nSteps      = params.nSteps;
        nVars       = params.nVars;
        nSeg        = params.nSeg;
        nMeasures   = params.nMeasures;

        % Reshape the input: each step has nVars*nSeg variables 
        %                    + step length and step height
        %                    + heel-strike and push-off impulse
        newInput = reshape(input, nVars, nSteps);

        % Initialize some storage variables
        tStore = zeros(nSteps*nSeg,1);
        stateStore = zeros(nSteps*nSeg,nMeasures);

        % Loop through the steps
        for stepNum = 1:nSteps
            % Get the state at the start of the step
            currInput = newInput(:, stepNum);

            % Simulate the step
            [tOut, stateOut] = singleStep_TelescopingLeg(currInput, params);

            % Store the results
            tStore((stepNum-1)*nSeg + 1:(stepNum)*nSeg) = tOut;
            stateStore((stepNum-1)*nSeg + 1:(stepNum)*nSeg,:) = stateOut;
        end

        % Make the time cumulative
        tStore = cumsum(tStore);        
        
        % Store the input and result for use in a later function call
        prevInput = input;
        prevOutputState = stateStore;
        prevOutputTime = tStore;
    end
end