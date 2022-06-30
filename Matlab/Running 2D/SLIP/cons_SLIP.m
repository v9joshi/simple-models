function [cineq, ceq] = cons_SLIP(inputs, params)
    % Unpack important inputs
    params.tTotal = inputs(7);
    params.dutyFactor = inputs(8);
    params.k = inputs(9);
    params.tStart = 0;
    
    % Use the rest of the inputs as the input state
    inputs = inputs(1:6);
    
    % Call the single hop function
    outputStruct = simulateAHop(inputs, params);
    
    % Determine leg length in contact
    contactX  = outputStruct.contactStates(:,2);
    contactY  = outputStruct.contactStates(:,3);
    contactVx = outputStruct.contactStates(:,4);
    contactVy = outputStruct.contactStates(:,5);

    contactXf = outputStruct.contactStates(:,6);
    contactYf = outputStruct.contactStates(:,7);
    
    % Calculate the leg length
    contactLegLength = sqrt((contactX - contactXf).^2 + (contactY - contactYf).^2);
    
    % Set up constraint
    % 1. Leg length in contact must be less than max leg length
    legLengthConstraint = contactLegLength - params.L0;
    
    % 2. Total distance travelled must equal distance constraint
    flightX  = outputStruct.flightStates(:,2);
    flightY  = outputStruct.flightStates(:,3);
    flightVx = outputStruct.flightStates(:,4);
    flightVy = outputStruct.flightStates(:,5);

    distanceConstraint = (flightX(end) - contactX(1)) - params.stepLength;
    
    % 3. Mass can never go below the ground
    aboveGroundConstraint = -[contactY(:); flightY(:)];
    
    % 4. Initial foot placement at origin
    startAtOrigin = [contactXf(1); contactYf(1)];
    
    % 5. Periodicity
    periodicY  = flightY(end) - contactY(1);
    periodicVx = flightVx(end) - contactVx(1);
    periodicVy = flightVy(end) - contactVy(1);
    
    periodicState = [periodicY; periodicVx; periodicVy];

    % Assemble the constraints
    cineq = [legLengthConstraint(:); aboveGroundConstraint];
    ceq = [distanceConstraint(:); startAtOrigin(:); periodicState(:)];    
end