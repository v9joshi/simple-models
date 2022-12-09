% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_KneeAndMuscles(t,statevar,params)
    % Unpack the parameters     
    m1 = params.m1; m2 = params.m2; I1 = params.I1; I2 = params.I2;
    l1 = params.l1; l2 = params.l2; r1 = params.r1; r2 = params.r2;
    g = params.g; 
    
    % Unpack the state
     theta0 = statevar(1);  theta1 = statevar(2);
    dtheta0 = statevar(3); dtheta1 = statevar(4);
    
    % Muscle state
    lce = statevar(5);
        
    % Solve for angular accelerations   
    tau1 = 0;
    tau2 = params.tau2a + t*(params.tau2b - params.tau2a)/params.time;
    
    % Determine current state
    theta1 = var(1); theta2 = var(2);
    dtheta1 = var(3); dtheta2 = var(4);

    % Set up equations
    M(1,1) = I1 + I2 + m2*((r2*cos(theta1 + theta2) + l1*cos(theta1))^2 + (r2*sin(theta1 + theta2) + l1*sin(theta1))^2) + m1*(r1^2*cos(theta1)^2 + r1^2*sin(theta1)^2);
    M(1,2) = I2 + m2*(r2*sin(theta1 + theta2)*(r2*sin(theta1 + theta2) + l1*sin(theta1)) + r2*cos(theta1 + theta2)*(r2*cos(theta1 + theta2) + l1*cos(theta1)));
    M(2,1) = I2 + m2*(r2*sin(theta1 + theta2)*(r2*sin(theta1 + theta2) + l1*sin(theta1)) + r2*cos(theta1 + theta2)*(r2*cos(theta1 + theta2) + l1*cos(theta1)));
    M(2,2) = I2 + m2*(r2^2*cos(theta1 + theta2)^2 + r2^2*sin(theta1 + theta2)^2);

    C(1,1) = tau1 - g*m2*r2*cos(theta1 + theta2) - g*l1*m2*cos(theta1) - g*m1*r1*cos(theta1) + dtheta2^2*l1*m2*r2*sin(theta2) + 2*dtheta1*dtheta2*l1*m2*r2*sin(theta2);
    C(2,1) = tau2 - g*m2*r2*cos(theta1 + theta2) - dtheta1^2*l1*m2*r2*sin(theta2);

    % Solve for accelerations
    ddthetalist = M\C;
        
    % Store the derivatives
    theta0dot = dtheta0; theta1dot = dtheta1l; theta2dot = dtheta2;
    theta0dotdot = ddthetalist(1); theta1dotdot = ddthetalist(2);
    
    % Find the state derivative
    dStateVar = [   theta0dot;    theta1dot;    theta2dot; 
                 theta0dotdot; theta1dotdot; theta2dotdot;
                 lcedot];
end