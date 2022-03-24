% Solve a two-link manipulator for given intial conditions
function []= PostProcess_TwoLinkArm(pinput,params)

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
    
    % Unpack outputs
    theta1 = theta(:,1); dtheta1 = dtheta(:,1);
    theta2 = theta(:,2); dtheta2 = dtheta(:,2);
    
    tlist = linspace(0,1, length(theta1));      
    
    % Calculate hand position given angles
    HandPosition_x = params.l1*cos(theta(end,1))+params.l2*cos(theta(end,1)+theta(end,2));
    HandPosition_y = params.l1*sin(theta(end,1))+params.l2*sin(theta(end,1)+theta(end,2));

    % Calculate hand velocity
    HandVelocity_x = -dtheta(:,1)*params.l1.*sin(theta(:,1))...
                    -(dtheta(end,1)+dtheta(:,2))*params.l2.*sin(theta(:,1)+theta(:,2));

    HandVelocity_y = dtheta(:,1)*params.l1.*cos(theta(:,1))...
                   +(dtheta(:,1)+dtheta(end,2))*params.l2.*cos(theta(:,1)+theta(:,2));

    % Use projectile motion to estimate distance travelled by the thrown
    % object.
    FallingTime = (HandVelocity_y(end)/params.g...
                 +sqrt(HandVelocity_y(end)/params.g - 2*HandPosition_y(end)/params.g));

    DistanceTravelled = HandVelocity_x(end)*FallingTime+HandPosition_x(end)-params.l1-params.l2;

    % Estimate the velocity angle and magnitude for the hand
    velocity_angle = atan2(HandVelocity_y, HandVelocity_x);
    velocity_magnitude = sqrt(HandVelocity_x.^2 + HandVelocity_y.^2);

    % Plot the angles of each segment - upper-arm, arm
    figure(1);
    subplot(2,2,1)
    plot(tlist, theta1);
    xlabel('Time (s)');
    ylabel('Theta 1 (rad)');
    title('Angles');

    subplot(2,2,2)
    plot(tlist, theta2);
    xlabel('Time (s)');
    ylabel('Theta 2 (rad)');

    % Plot the angular velocities of each segment - upper-arm, arm
    subplot(2,2,3)
    plot(tlist, dtheta1);
    xlabel('Time (s)');
    ylabel('Omega 1 (rad s^-^1)');
    title('Angular Velocity');

    subplot(2,2,4)
    plot(tlist, dtheta2);
    xlabel('Time (s)');
    ylabel('Omega 2 (rad s^-^1)');

    % Plot the angle and speed of the hand
    figure(2)
    subplot(1,2,1)
    plot(tlist, velocity_magnitude)
    xlabel('Time (s)');
    ylabel('Hand velocity (m s^-^1)');

    subplot(1,2,2)
    plot(tlist, velocity_angle*180/pi)
    xlabel('Time (s)');
    ylabel('Hand angle (degrees)');

    % Animate
    %Animating the movement of the double pendulum
    x0 = 0;
    y0 = 0;

    P1 = [x0+params.l1*cos(theta1),y0+params.l1*sin(theta1)];
    P2 = P1 + [params.l2*cos(theta1+theta2), params.l2*sin(theta1+theta2)];

    x1 = P1(1,1);
    y1 = P1(1,2);

    x2 = P2(1,1);
    y2 = P2(1,2);

    figure(3)
    set(gcf, 'color','w')
    plot(x0,y0,'^r','markersize',10,'markerfacecolor','r'); hold on;
    firstlink = line([x0, x1],[y0, y1],'marker','o','markerfacecolor','r','markeredgecolor','r');
    secondlink = line([x1, x2],[y1, y2],'marker','o','markerfacecolor','r','markeredgecolor','r');

    axis([-1.5*(params.l1+params.l2), 1.5*(params.l1+params.l2), -1.5*(params.l1+params.l2), 1.5*(params.l1+params.l2)]);
    axis square
    set(gca, 'visible','off')

    for i = 1:length(theta1)
        x1 = P1(i,1);
        y1 = P1(i,2);

        x2 = P2(i,1);
        y2 = P2(i,2);

        set(firstlink,'xdata',[x0, x1],'ydata',[y0, y1]);
        set(secondlink,'xdata',[x1, x2],'ydata',[y1, y2]);

        pause(0.1);
    end
end