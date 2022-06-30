% Program to Simulate Spring-Mass hopper
clear all; close all; clc;

% Set some parameters
g = 10; L0 = 1; m = 1;  stepLength = 3;

% unacking params
params.g = g; params.L0 = L0; params.m = m;
params.stepLength = stepLength;

doPlots = 1;

%% Set some initial guess
oldSol = load('guessStores.mat');
var0 = [0,0.99,0,0,1e-6,1e-6,2,0.5,2800];
% var0 = oldSol.xOut;

%% Set up the linear constraints and bounds
Aineq = [];
Bineq = [];
A = [];
B = [];
LB = [-1,0.2,-5,-5,0,0,0,0,0];
UB = [1,1,5,5,0.1,0.1,10,1,1e6];

% Run an optimization
%% Set up fmincon
objFun = @(inputVar) -inputVar(8);
conFun = @(inputVar) cons_SLIP(inputVar, params);

% Run the optimization
options = optimoptions('fmincon','Display','iter','MaxFunEvals',1e6,'MaxIter',8000);
xOut = fmincon(objFun,var0,Aineq,Bineq,A,B,LB,UB,conFun, options);

%% Unpack important inputs
params.tTotal = xOut(7);
params.dutyFactor = xOut(8);
params.k = xOut(9);
params.tStart = 0;

% Use the rest of the inputs as the input state
inputs = xOut(1:6);

% Storage variables
stateStore = [];
tStore = [];

% Simulate the hopper for n steps
for stepNum = 1:3
    outputStruct = simulateAHop(inputs, params);

    stateStore = [stateStore; outputStruct.contactStates(1:end-1,2:end); outputStruct.flightStates(1:end-1,2:end)];
    tStore = [tStore; outputStruct.contactStates(1:end-1,1); outputStruct.flightStates(1:end-1,1)];
    
    inputs = outputStruct.flightStates(end,2:end);
    inputs(5) = inputs(5) + stepLength;
    params.tStart = params.tStart + params.tTotal;
end

%% Unpack the data and save it
xlist   = stateStore(:,1);        vxlist    = stateStore(:,3);
ylist   = stateStore(:,2);        vylist    = stateStore(:,4);
xflist  = stateStore(:,5);       yflist     = stateStore(:,6);

%% Interpolate everything to get more even time-steps
tStoreNew = linspace(tStore(1), tStore(end),1000);
xlist = interp1(tStore, xlist, tStoreNew);
ylist = interp1(tStore, ylist, tStoreNew);
vxlist = interp1(tStore, vxlist, tStoreNew);
vylist = interp1(tStore, vylist, tStoreNew);
xflist = interp1(tStore, xflist, tStoreNew);
yflist = interp1(tStore, yflist, tStoreNew);

tStore = tStoreNew;

%% Calculate the leg length
Llist = sqrt((xlist - xflist).^2 + (ylist - yflist).^2);

% Calculate the contact force
Flist = zeros(length(tStore),1);
for i = 1:length(tStore)
    if Llist(i) > L0
        Flist(i) = 0;
    else
        Flist(i) = params.k*(L0 - Llist(i));
    end
end

%% Plot the data
figure(2)
subplot(4,1,1)
plot(tStore,xlist,'b');
ylim([0, 8])
xlabel('Time (s)');
ylabel('X Co-ordinate (m)');


subplot(4,1,2)
plot(tStore,ylist,'b');
ylim([0, max(ylist)*1.1])
xlabel('Time (s)');
ylabel('Y Co-ordinate (m)');

subplot(4,1,3)
plot(tStore,vxlist,'b');

xlabel('Time (s)');
ylabel('X speed (m s ^-^1)');


subplot(4,1,4)
plot(tStore,vylist,'b');

xlabel('Time (s)');
ylabel('Y speed (m s ^-^1)');

figure(3)
plot(tStore,Flist)
title('Ground Reaction Force vs Time')
xlabel('Time (s)');
ylabel('Force (N)');

%% Animate the hopper
figure(1)
set(gcf, 'color','w')
mass_handle = plot(xlist(1),ylist(1),'ro','markerfacecolor','r');
hold on
leg_handle = plot([xflist(1),xlist(1)],[yflist(1), ylist(1)],'k--');
plot([-3,10],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
axis([-3 10 -1 max(ylist)+1]);
set(gca,'visible','off')
hold off

for i = 2:10:length(tStore)
    set(mass_handle,'xdata',xlist(i),'ydata',ylist(i));
    if Llist(i) < L0
        set(leg_handle,'xdata',[xflist(i),xlist(i)],'ydata',[yflist(i), ylist(i)])
    else
        set(leg_handle,'xdata',[xlist(i),xlist(i)],'ydata',[max(ylist(i)-1,0), ylist(i)])
    end
    pause(0.05);
end
