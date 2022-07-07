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

    outputStruct.contactStates(:,end+1) = (-1)^stepNum;
    outputStruct.flightStates(:,end+1)  = 0;
    
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
Flist(cflist == 0) = 0;
Llist(cflist == 0) = params.L0;

% Find the right and left leg angles
Langle_left = Langle(cflist == 1);
tStore_left = tStore(cflist == 1);

Langle_right = Langle(cflist == -1);
tStore_right = tStore(cflist == -1);

% Add additional points to the start and/or end of the Langle vectors
if ~mod(nSteps,2)  % Even steps - start with right end with left
    Langle_left = [min(Langle_left); Langle_left; min(Langle_left)];
    tStore_left = [tStore(1); tStore_left; tStore_left(end) + params.tTotal];
    
    Langle_right = [Langle_right; max(Langle_right)];
    tStore_right = [tStore_right; tStore(end)];
else              % Odd steps - start with right end with right
    Langle_left = [min(Langle_left); Langle_left; max(Langle_left)];
    tStore_left = [tStore(1); tStore_left; tStore(end)];
end

% Interpolate the leg angle during flight
Langle_left  = interp1(tStore_left, Langle_left, tStore);
Langle_right = interp1(tStore_right, Langle_right, tStore);

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
cflist = interp1(tStore, cflist, tStoreNew);

Langle_left  = interp1(tStore, Langle_left, tStoreNew);
Langle_right = interp1(tStore, Langle_right, tStoreNew);

% Calculate energy
KElist          = 0.5*params.m*(vxlist.^2 + vylist.^2);
PElist_spring   = 0.5*params.k*(Llist - params.L0).^2;
PElist_gravity  = params.m*params.g*ylist;

% Find the left and right foot positions
xflist_left = xlist - params.L0*cos(Langle_left);
yflist_left = ylist - params.L0*sin(Langle_left);

xflist_right = xlist - params.L0*cos(Langle_right);
yflist_right = ylist - params.L0*sin(Langle_right);

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
leg_handle_left = plot([xflist_left(1),xlist(1)],[yflist_left(1), ylist(1)],'k-');
leg_handle_right = plot([xflist_right(1),xlist(1)],[yflist_right(1), ylist(1)],'k-');

plot([-stepLength nSteps*stepLength],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
xlim([-stepLength nSteps*stepLength])
ylim([-0.01, 2])
axis equal
set(gca,'visible','off','YLimMode','manual')
hold off

% Loop throught he time-points
for i = 2:20:length(tStoreNew)
    
    % Update the coordinates
    set(mass_handle,'xdata',xlist(i),'ydata',ylist(i));
    set(leg_handle_left,'xdata',[xflist_left(i),xlist(i)],'ydata',[yflist_left(i), ylist(i)],'color','k','linewidth',1)
    set(leg_handle_right,'xdata',[xflist_right(i),xlist(i)],'ydata',[yflist_right(i), ylist(i)],'color','k','linewidth',1)
    
    % Change leg width when in contact
    if cflist(i) == -1
        set(leg_handle_right, 'linewidth', (params.L0/Llist(i))^3, 'color','r');
    elseif cflist(i) == 1
        set(leg_handle_left, 'linewidth', (params.L0/Llist(i))^3,'color','r');
    else
        set(leg_handle_left, 'linewidth', 1);
        set(leg_handle_right, 'linewidth', 1);
    end

    pause(0.01);
end
