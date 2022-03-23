function [cineq,ceq] = Cons_Optimize_ThrowingSpeed(pinput,params)

% Ensure that the manipulator reaches the desired point
global numel
theta = params.init_theta';
dtheta = params.init_dtheta';

% display ('call to Constraint');
for i = 1:(numel - 1)
%     display ('within loop');
    torques(1) = pinput(i);
    torques(2) = pinput(numel - 1 + i);
        
    outputs = Root_TwoLinkArm(torques,params);
    params.init_theta  = [outputs(end,1);outputs(end,2)];
    params.init_dtheta = [outputs(end,3);outputs(end,4)];
    theta = [theta; outputs(2:end,1:2)];
    dtheta = [dtheta; outputs(2:end,3:4)];
            
end

HandPosition_y = params.l1*sin(theta(end,1))+params.l2*sin(theta(end,1)+theta(end,2));
% 
% HandVelocity_x = -dtheta(end,1)*params.l1*sin(theta(end,1))...
%                 -(dtheta(end,1)+dtheta(end,2))*params.l2*sin(theta(end,1)+theta(end,2));
% 
% HandVelocity_y = dtheta(end,1)*params.l1*cos(theta(end,1))...
%                +(dtheta(end,1)+dtheta(end,2))*params.l2*cos(theta(end,1)+theta(end,2));
% 
% velocity_angle = atan2(HandVelocity_y, HandVelocity_x);
               
cineq = [-theta(:,2); theta(:,2)-pi; theta(:,1)-pi/2;-pi/2-theta(:,1)];
ceq   = [];%velocity_angle-pi/4];
    
end