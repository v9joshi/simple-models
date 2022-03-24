function [cineq,ceq] = Cons_Optimize_ThrowingSpeed(pinput,params)

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

    % Make sure that joint angle constraints are met
    cineq = [-theta(:,2); theta(:,2)-pi; theta(:,1)-pi/2;-pi/2-theta(:,1)];
    ceq   = [];
end