function newState = applyImpulse(currState, delV)    
    % Unpack the state
    x = currState(1);
    y = currState(2);
    vx = currState(3);
    vy = currState(4);
    
    xf = currState(6);
    yf = currState(7);
    
    % Calculate the leg length
    legLength = sqrt((x - xf)^2 + (y - yf)^2);
    
    % Determine impulse components
    delVx = delV*(x - xf)/legLength;
    delVy = delV*(y - yf)/legLength;
    
    % Change the state
    vx_new = vx + delVx;
    vy_new = vy + delVy;
    
    % Store the new state
    newState = currState;
    newState(3) = vx_new;
    newState(4) = vy_new;  
end