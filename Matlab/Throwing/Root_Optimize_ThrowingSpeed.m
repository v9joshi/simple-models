clear all; close all; clc;

global numel count

count   = 0;  %number of iterations
numel   = 8;  %number of elements

params.init_theta  = [-pi/2+0.01;0];
params.init_dtheta = [0;0];

% Data from D.A. Winter Anthropometric tables for 70Kg Human 50% percentile
% Manipulator parameters and physics
params.m1 = 1.54; params.m2 = 1.96;
params.l1 = 0.361; params.l2 = 0.481;
params.r1 = 0.436*params.l1; params.r2 = 0.682*params.l2;
params.g =  10; % Non-dimensionalize gravity to change time scale 

% radius of gyration is 0.542 from distal end for upper arm and 0.827 from
% distal end for forearm+hand
params.I1 = params.m1*(0.542^2 - params.r1^2); 
params.I2 = params.m2*(0.827^2 - params.r2^2);

params.time = 1/numel; % Time taken in each segment

% Holding your arm out straight forward produces a torque requirement of 15
mintorque = -30; % minimum torque
maxtorque =  30; % maximum torque

% Seed torques
pinput0 = 3*rand(2*numel-2,1); 

% Fmincon inputs
Aineq = []; Bineq = []; % No linear equalities
Aeq = []; Beq = []; % No non-linear equalities

% Torque Bounds
LB = mintorque*ones(2*numel-2,1);
UB = maxtorque*ones(2*numel-2,1);

Obj_Function = @(pinput) Obj_Optimize_ThrowingSpeed(pinput,params);
Cons_Function = @(pinput) Cons_Optimize_ThrowingSpeed(pinput,params);

options = optimset('display','iter','MaxFunEvals',10000,'MaxIter',600);

% Optimize
[presult,optfval] = fmincon(Obj_Function,pinput0,Aineq,Bineq,Aeq,Beq,LB,UB,Cons_Function,options); 

%% Solve and animate the link
close all;
PostProcess_TwoLinkArm(presult,params);