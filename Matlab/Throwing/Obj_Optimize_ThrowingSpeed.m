function objval = Obj_Optimize_ThrowingSpeed(pinput,params)

    % Number of elements in the piecewise-constant torque inputs
    global numel 

    % Initial angle and angular velocity of the arm
    theta = params.init_theta';
    dtheta = params.init_dtheta';

    % Loop for each segment of the discretized motion
    for i = 1:(numel - 1)
        % Set the torques for the shoulder and arm
        torques(1) = pinput(i);
        torques(2) = pinput(numel - 1 + i);

        % Run the simulation for the given torque inputs
        outputs = Root_TwoLinkArm(torques,params);
        
        % Unpack the outputs from the simulation
        params.init_theta  = [outputs(end,1);outputs(end,2)];
        params.init_dtheta = [outputs(end,3);outputs(end,4)];
        
        % Store the outputs
        theta = [theta; outputs(2:end,1:2)];
        dtheta = [dtheta; outputs(2:end,3:4)];
    end

    % Calculate hand position given angles
    HandPosition_x = params.l1*cos(theta(end,1))+params.l2*cos(theta(end,1)+theta(end,2));
    HandPosition_y = params.l1*sin(theta(end,1))+params.l2*sin(theta(end,1)+theta(end,2));

    % Calculate hand velocity
    HandVelocity_x = -dtheta(end,1)*params.l1*sin(theta(end,1))...
                    -(dtheta(end,1)+dtheta(end,2))*params.l2*sin(theta(end,1)+theta(end,2));

    HandVelocity_y = dtheta(end,1)*params.l1*cos(theta(end,1))...
                   +(dtheta(end,1)+dtheta(end,2))*params.l2*cos(theta(end,1)+theta(end,2));

    % Use projectile motion to estimate distance travelled by the thrown
    % object.
    FallingTime = (HandVelocity_y/params.g...
                 +sqrt(HandVelocity_y/params.g - 2*HandPosition_y/params.g));

    DistanceTravelled = HandVelocity_x*FallingTime+HandPosition_x-params.l1-params.l2;

    % Maximize the distance travelled by a projectile throw by this arm
    objval = -DistanceTravelled; 
end