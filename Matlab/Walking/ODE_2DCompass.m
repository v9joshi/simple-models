% Function to perform one-time step of the simulation of a 2D point-mass
% inverted pendulum walker
function dStateVar = ODE_2DCompass(~,statevar,params)
    % Unpack the parameters
    g = params.g; L0 = params.L0; m = params.m; M = params.M;
    
    % Unpack the state
    x1  = statevar(1); x2  = statevar(2); 
    y1  = statevar(3); y2  = statevar(4); 
    vx1 = statevar(5); vx2 = statevar(6);
    vy1 = statevar(7); vy2 = statevar(8);
    
    % Foot position
    x0 = statevar(9); y0 = statevar(10);
    
    % Find the force along the legs
    % F1*L0 - F2*((x1 - x0)*(x2 - x1) + (y1 - y0)*(y2 - y1))/L0 = M*(g*(y1 - y0) - (vx1^2 + vy1^2));
    % F1*((x1 - x0)*(x1 - x2)/L0 + (y1 - y0)*(y1 - y2)/L0)*1/M + F2*L0*(1/m + 1/M) = - (vx1 - vx2)^2 - (vy1 - vy2)^2;
    
    % DAE formulation - coeffs*[vx1; vx2; vy1; vy2; ax1; ax2; ay1; ay2; F1; F2] = RHS
    F1coeffs = [0; 0; 0; 0; -(x1 - x0)/(M*L0); 0; -(y1 - y0)/(M*L0); 0];
    F2coeffs = [0; 0; 0; 0; -(x1-x2)/(M*L0); -(x2-x1)/(m*L0); -(y1-y2)/(M*L0); -(y2-y1)/(m*L0)];
    C1coeffs = [zeros(1,8), L0/M, ((x1 - x0)*(x1 - x2) + (y1 - y0)*(y1 - y2))/(M*L0)]; 
    C2coeffs = [zeros(1,8), -((x1 - x0)*(x2 - x1) + (y1 - y0)*(y2 - y1))/(M*L0), L0*(1/m + 1/M)];
    
    % Assemble the coefficients
    coeffs = [eye(8), F1coeffs, F2coeffs;
              C1coeffs;
              C2coeffs];
    
    RHS = [                            vx1;
                                       vx2;
                                       vy1;
                                       vy2;
                                         0;
                                         0;
                                        -g;
                                        -g;
               g*(y1 - y0) - vx1^2 - vy1^2;
          - (vx1  - vx2)^2 - (vy1 - vy2)^2];
    
    % Solve the equations
    soln = coeffs\RHS;
    
    % Find the state derivative
    dStateVar = [soln(1:8); 0; 0];    
end