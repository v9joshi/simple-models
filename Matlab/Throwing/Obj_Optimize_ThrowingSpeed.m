function objval = Obj_Optimize_ThrowingSpeed(pinput,params)

global numel 

theta = params.init_theta';
dtheta = params.init_dtheta';

% display ('call to Objective');
for i = 1:(numel - 1)
%   display ('within loop');    
    torques(1) = pinput(i);
    torques(2) = pinput(numel - 1 + i);
        
    outputs = Root_TwoLinkArm(torques,params);
    params.init_theta  = [outputs(end,1);outputs(end,2)];
    params.init_dtheta = [outputs(end,3);outputs(end,4)];
    theta = [theta; outputs(2:end,1:2)];
    dtheta = [dtheta; outputs(2:end,3:4)];
            
end

HandPosition_x = params.l1*cos(theta(end,1))+params.l2*cos(theta(end,1)+theta(end,2));
HandPosition_y = params.l1*sin(theta(end,1))+params.l2*sin(theta(end,1)+theta(end,2));

HandVelocity_x = -dtheta(end,1)*params.l1*sin(theta(end,1))...
                -(dtheta(end,1)+dtheta(end,2))*params.l2*sin(theta(end,1)+theta(end,2));

HandVelocity_y = dtheta(end,1)*params.l1*cos(theta(end,1))...
               +(dtheta(end,1)+dtheta(end,2))*params.l2*cos(theta(end,1)+theta(end,2));

FallingTime = (HandVelocity_y/params.g...
             +sqrt(HandVelocity_y/params.g - 2*HandPosition_y/params.g));
         
DistanceTravelled = HandVelocity_x*FallingTime+HandPosition_x-params.l1-params.l2;

objval = -DistanceTravelled; % Maximize

end