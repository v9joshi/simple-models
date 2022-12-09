% Simulate a hop
function outputStruct = simulateAPoweredHop(input, params)
    % Unpack the input
    state0 = input(:);
    
    % Unpack useful parameters
    tTotal = params.tTotal; 
    dutyFactor = params.dutyFactor;

    % Set the flight time and contact time
    tContact = dutyFactor*tTotal;
    tFlight = (1 - dutyFactor)*tTotal;    

    % Initialize things
    tStart = params.tStart;
    tspanContact = linspace(0,tContact,100);
    tspanFlight = linspace(0,tFlight,100);

    %% Plug in the ode files for flight/contact and event functions for phase switching
    ODE_flight = @(t,statevar) ODE_Flight(t,statevar,params);
    ODE_contact = @(t,statevar) ODE_Contact_SpringForce(t,statevar,params);

    % Simulate the contact phase
    options = odeset('reltol',1e-9,'abstol',1e-9);
    [tlist2,StateVarlist2] = ode45(ODE_contact,tspanContact,state0,options);
    tlist2 = tlist2 + tStart;
    
    % Update the start time
    tStart = tStart + tContact;
   
    % Set the next initial state
    state0      = StateVarlist2(end,:)';

    % Simulate the flight phase
    options = odeset('reltol',1e-9,'abstol',1e-9);
    [tlist1,StateVarlist1] = ode45(ODE_flight,tspanFlight,state0,options);
    tlist1 = tlist1 + tStart;
    
    % Store the results
    outputStruct.contactStates = [tlist2(:), StateVarlist2];
    outputStruct.flightStates  = [tlist1(:), StateVarlist1];       
end
