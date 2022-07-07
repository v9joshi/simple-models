% Program to Simulate Spring-Mass hopper
clear all; close all; clc;
set(0,'DefaultAxesColorOrder',linspecer(7));
set(0,'DefaultFigureColor','w');

% Set some parameters
% Gravity, leg length, body mass
g = 1; L0 = 1; m = 1;

% Step length and step time
stepLength = 3;     tTotal = 3;

% Maximum CoM height
 maxAir = 1.6*L0;
 
% Packing params
params.g = g; params.L0 = L0; params.m = m;
params.stepLength = stepLength; params.maxAir = maxAir;
params.tTotal = tTotal;

%% Set some initial guess
oldSol = load('guessStores.mat');
% var0 = [0,1,0,-1,1e-9,1e-9,0.5, 4];
var0 = oldSol.xOut;

%% Set up the linear constraints and bounds
Aineq = [];     Bineq = [];
A     = [];         B = [];

% Set bounds
minX0   =  -1;      maxX0   =   1;
minY0   = 0.2;      maxY0   =   1;
minVx   = -10;      maxVx   =  10;
minVy   =  -5;      maxVy   =   5;
minXf   =   0;      maxXf   = 0.1;
minYf   =   0;      maxYf   = 0.1;
minDuty =   0;      maxDuty =   1;
minK    =   1;      maxK    = 1e6;

% Store bounds
LB = [minX0, minY0, minVx, minVy, minXf, minYf, minDuty, minK];
UB = [maxX0, maxY0, maxVx, maxVy, maxXf, maxYf, maxDuty, maxK];

%% Run an optimization
% Set up fmincon
objFun = @(inputVar) 1;     % No cost to solve for, just fitting to constraints
conFun = @(inputVar) cons_SLIP(inputVar, params);

% Run the optimization
options = optimoptions('fmincon','Display','iter','MaxFunEvals',1e6,'MaxIter',8000);
xOut = fmincon(objFun,var0,Aineq,Bineq,A,B,LB,UB,conFun, options);

%% Post-process: Use the optimization solution to make an animation
% Convert solution values to parameters
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

%% Calculate the leg length, leg angle and leg force
Llist  = sqrt((xlist - xflist).^2 + (ylist - yflist).^2);
Langle = atan2(ylist - yflist, xlist - xflist);
Flist  = params.k*(params.L0 - Llist);

% Correct these values for contact
Flist(cflist == 1) = 0;
Llist(cflist == 1) = params.L0;

% Make the leg angle interpolate during flight
Langle(cflist == 1) = [];
Langle(end + 1) = Langle(1);
tempTime = tStore;
tempTime(cflist == 1) = [];
tempTime(end+1) = tStore(end);
Langle = interp1(tempTime, Langle, tStore);

%% Interpolate everything to get uniform time-steps
tStoreNew = linspace(tStore(1), tStore(end),1000*nSteps);

xlist  = interp1(tStore, xlist, tStoreNew);
ylist  = interp1(tStore, ylist, tStoreNew);
vxlist = interp1(tStore, vxlist, tStoreNew);
vylist = interp1(tStore, vylist, tStoreNew);
xflist = interp1(tStore, xflist, tStoreNew);
yflist = interp1(tStore, yflist, tStoreNew);
Flist  = interp1(tStore, Flist, tStoreNew);
Llist  = interp1(tStore, Llist, tStoreNew);
Langle = interp1(tStore, Langle, tStoreNew);

% Calculate energy
KElist          = 0.5*params.m*(vxlist.^2 + vylist.^2);
PElist_spring   = 0.5*params.k*(Llist - params.L0).^2;
PElist_gravity  = params.m*params.g*ylist;

% Correct the foot position during flight
xflist_flight = xlist - params.L0*cos(Langle);
yflist_flight = ylist - params.L0*sin(Langle);

%% Plot important data
% Plot states
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

% Plot force and energy
figure(3)
subplot(1,2,1)
plot(tStoreNew,Flist)
title('Ground Reaction Force vs Time')
xlabel('Time (s)');
ylabel('Force (N)');

subplot(1,2,2)
plot(tStoreNew,KElist)
hold on
plot(tStoreNew,PElist_spring);
plot(tStoreNew,PElist_gravity);
title('Energy vs Time')
xlabel('Time (s)');
ylabel('Energy (J)');
legend('KE','PE spring','PE gravity')

%% Animate the hopper
figure(1)
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
        set(leg_handle,'xdata',[xflist(i),xlist(i)],'ydata',[yflist(i), ylist(i)],'color','b','linewidth',(params.L0/Llist(i))^3)
    else
        set(leg_handle,'xdata',[xflist_flight(i),xlist(i)],'ydata',[yflist_flight(i), ylist(i)],'color','k','linewidth',1)
    end
    pause(0.01);
end
