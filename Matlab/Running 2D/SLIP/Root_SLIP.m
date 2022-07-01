% Program to Simulate Spring-Mass hopper
clear all; close all; clc;

% Set some parameters
g = 1; L0 = 1; m = 1;  stepLength = 3; maxAir = 1.6*L0;
tTotal = 2;

% unacking params
params.g = g; params.L0 = L0; params.m = m;
params.stepLength = stepLength; params.maxAir = maxAir;
params.tTotal = tTotal;

%% Set some initial guess
oldSol = load('guessStores.mat');
% var0 = [0,1,0,-1,1e-9,1e-9,0.5, 4];
var0 = oldSol.xOut;

%% Set up the linear constraints and bounds
Aineq = [];
Bineq = [];
A = [];
B = [];
LB = [-1,0.2,-10,-5,0,0,0,1];
UB = [1,1,10,5,0.1,0.1,1,1e6];

% Run an optimization
%% Set up fmincon
objFun = @(inputVar) 1;
conFun = @(inputVar) cons_SLIP(inputVar, params);

% Run the optimization
options = optimoptions('fmincon','Display','iter','MaxFunEvals',1e6,'MaxIter',8000);
xOut = fmincon(objFun,var0,Aineq,Bineq,A,B,LB,UB,conFun, options);

%% Unpack important inputs
params.dutyFactor = xOut(7);
params.k = xOut(8);
params.tStart = 0;

% Use the rest of the inputs as the input state
inputs = xOut(1:6);

% Storage variables
stateStore = [];
tStore = [];

% Simulate the hopper for n steps
nSteps = 4;
for stepNum = 1:nSteps
    outputStruct = simulateAHop(inputs, params);

    outputStruct.contactStates(:,end+1) = 0;
    outputStruct.flightStates(:,end+1)  = 1;
    
    stateStore = [stateStore; outputStruct.contactStates(1:end-1,2:end); outputStruct.flightStates(1:end-1,2:end)];
    tStore = [tStore; outputStruct.contactStates(1:end-1,1); outputStruct.flightStates(1:end-1,1)];
    
    inputs = outputStruct.flightStates(end,2:end-1);
    inputs(5) = inputs(5) + stepLength;
    params.tStart = params.tStart + params.tTotal;
end

%% Unpack the data and save it
xlist   = stateStore(:,1);        vxlist    = stateStore(:,3);
ylist   = stateStore(:,2);        vylist    = stateStore(:,4);
xflist  = stateStore(:,5);       yflist     = stateStore(:,6);
cflist  = stateStore(:,7);


%% Calculate the leg length and leg force
Llist  = sqrt((xlist - xflist).^2 + (ylist - yflist).^2);

% Calculate the contact force
Flist = zeros(length(tStore),1);
for i = 1:length(tStore)
    if cflist(i)
        Flist(i) = 0;
    else
        Flist(i) = params.k*(L0 - Llist(i));
    end
end

%% Interpolate everything to get more even time-steps
tStoreNew = linspace(tStore(1), tStore(end),1000*nSteps);

xlist  = interp1(tStore, xlist, tStoreNew);
ylist  = interp1(tStore, ylist, tStoreNew);
vxlist = interp1(tStore, vxlist, tStoreNew);
vylist = interp1(tStore, vylist, tStoreNew);
xflist = interp1(tStore, xflist, tStoreNew);
yflist = interp1(tStore, yflist, tStoreNew);
Flist  = interp1(tStore, Flist, tStoreNew);

%% Plot the data
figure(2)
subplot(4,1,1)
plot(tStoreNew,xlist,'b');
ylim([0, 8])
xlabel('Time (s)');
ylabel('X Co-ordinate (m)');


subplot(4,1,2)
plot(tStoreNew,ylist,'b');
ylim([0, max(ylist)*1.1])
xlabel('Time (s)');
ylabel('Y Co-ordinate (m)');

subplot(4,1,3)
plot(tStoreNew,vxlist,'b');

xlabel('Time (s)');
ylabel('X speed (m s ^-^1)');

subplot(4,1,4)
plot(tStoreNew,vylist,'b');

xlabel('Time (s)');
ylabel('Y speed (m s ^-^1)');

figure(3)
plot(tStoreNew,Flist)
title('Ground Reaction Force vs Time')
xlabel('Time (s)');
ylabel('Force (N)');

%% Animate the hopper
figure(1)
set(gcf, 'color','w')
mass_handle = plot(xlist(1),ylist(1),'ro','markerfacecolor','r');
hold on
leg_handle = plot([xflist(1),xlist(1)],[yflist(1), ylist(1)],'k-');
plot([-stepLength nSteps*stepLength],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
xlim([-stepLength nSteps*stepLength])
ylim([-0.01, 2])
axis equal
set(gca,'visible','off','YLimMode','manual')
hold off

% Loop throught he time-points
for i = 2:20:length(tStoreNew)
    set(mass_handle,'xdata',xlist(i),'ydata',ylist(i));
    if Flist(i) > max(Flist/100)
        set(leg_handle,'xdata',[xflist(i),xlist(i)],'ydata',[yflist(i), ylist(i)])
    else
        set(leg_handle,'xdata',[xlist(i),xlist(i)],'ydata',[ylist(i), ylist(i)])
    end
    pause(0.01);
end
