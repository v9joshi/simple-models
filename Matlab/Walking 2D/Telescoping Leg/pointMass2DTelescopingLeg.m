% Root file
close all;

% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point
Fmax = 200*m*g;
stepLength = 0.5;        % The target step length - Unused
Speed = 1.2;

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.stepLength = stepLength;
params.Fmax = Fmax; params.Speed = Speed;

% Initial conditions
x0 = 0;  vx0 = 0.01;      % Start at mid-stance with some forward velocity
y0 = 0.7;                 % Start somewhere that doesn't break leg length constraints
vy0 = 0;                  % No velocity along the leg                          
footX = 0; footY = 0;     % Initial foot position
F0 = m*g;
delF = 0.01;
delT = 0.01;

% Pack initial state
state0 = [x0, y0, vx0, vy0, F0, footX, footY];
    
% Set up segment and step parameters
nSteps      = 2;
nStates     = 7;
nInputs     = 7;
nMeasures   = 8;
nSeg        = 20;
nVars       = nInputs*nSeg + 2;

% Pack these params
params.nSteps = nSteps;
params.nStates = nStates;
params.nSeg = nSeg;
params.nVars = nVars;
params.nInputs = nInputs;
params.nMeasures = nMeasures;

%% Set the initial guess
input0 = [state0(1:nStates - 2), delF, delT];
var0 = repmat(input0, nSeg, 1);
var0 = reshape(var0, [], 1);
impPO = 1;
impHS = 1;
var0 = [var0; footX; footY];
var0 = repmat(var0, nSteps, 1);

var0(1:nSeg) = linspace(-0.2,0.2,nSeg);
var0(nVars + 1:nVars+nSeg) = linspace(0.2,0.4,nSeg);

% Shift initial point to make sure step length for first guess is non-zero
var0(end-3) = 0.2;

%% Make Bounds
xmin    = -1;            xmax   = 1*nSteps;
ymin    = 0.5;           ymax   = 1;
vxmin   = 0;            vxmax   = 3;
vymin   = -3;           vymax   = 3;
Fmin    = 0;        
xfmin   = -1;            xfmax   = 2*nSteps;
yfmin   = -1e-6;         yfmax   = 1e-6;
delFmin = -Fmax*1000;   delFmax = Fmax*1000;
delTmin = 0.0001;       delTmax = 0.5;

LBstate = [xmin,ymin,vxmin,vymin,Fmin,xfmin,yfmin];
UBstate = [xmax,ymax,vxmax,vymax,Fmax,xfmax,yfmax];

LBinput = [LBstate(1:nStates-2), delFmin, delTmin];
UBinput = [UBstate(1:nStates-2), delFmax, delTmax];

LBvar = repmat(LBinput, nSeg,1);
LBvar = reshape(LBvar, [], 1);
LBvar = [LBvar; LBstate(end-1); LBstate(end)]; 
LB = repmat(LBvar, nSteps,1);

UBvar = repmat(UBinput, nSeg,1);
UBvar = reshape(UBvar, [], 1);
UBvar = [UBvar; UBstate(end-1); UBstate(end)]; 
UB = repmat(UBvar, nSteps,1);

% Ineq and eq matrices
A = [];
B = [];
Aineq = [];
Bineq = [];

% Testing code
[eqCons, ineqCons] = cons_TelescopingLeg(var0, params);
objVal = obj_TelescopingLeg(var0, params);

%% Guess value
randVal = rand();
% var0 = randVal*LB + (1-randVal)*UB;
sampleVal = load('InvertedPendulumSol.mat');
var0 = sampleVal.xOut;
xOut = sampleVal.xOut;

%% Set up fmincon
objFun = @(inputVar) obj_TelescopingLeg(inputVar, params);
conFun = @(inputVar) cons_TelescopingLeg(inputVar, params);

% Run the optimization
options = optimoptions('fmincon','Display','iter','MaxFunEvals',1e6,'MaxIter',4000,'UseParallel','Always');
xOut = fmincon(objFun,var0,Aineq,Bineq,A,B,LB,UB,conFun, options);

%% Unpack the input
inputStateStore = [];

% Reshape the inputs to read them
stepVars = reshape(xOut, nVars, nSteps); 
delF = zeros(nSteps*nSeg,1);
delT = zeros(nSteps*nSeg,1);

% Loop through the steps
for stepNum = 1:nSteps
   currStepVar = stepVars(:, stepNum);

   % Read the impulses and foot positions
   currXf = currStepVar(end-1);
   currYf = currStepVar(end);

   % Read the seg vars for each segment
   segVars = reshape(currStepVar(1:end-2)', nSeg, nInputs);  

   for segNum = 1:nSeg
      currSegVar =  segVars(segNum,:);

      % Read delF and delT
      delF((stepNum - 1)*segNum + segNum)   =  currSegVar(end-1);
      delT((stepNum - 1)*segNum + segNum)   =  currSegVar(end);

      % Read the states
      currSegState = [currSegVar(1:end-2), currXf, currYf];
      inputStateStore = [inputStateStore; currSegState];
   end
end

%% Run the result
[tStore, outputStateStore] = nSteps_TelescopingLeg(xOut, params);

%% Interpolate the states
tInput          = [0;tStore(1:end-1)];
tOutput         = tStore;
% tInput_inter    = tInput(1):1e-3:tInput(end);
% tOutput_inter   = tOutput(1):1e-3:tOutput(end);
% 
% inputStateStore  = interp1(tInput, inputStateStore, tInput_inter);%,'method','linear');
% outputStateStore = interp1(tStore, outputStateStore, tOutput_inter);%,'method','linear');

% tInput = tInput_inter;
% tOutput = tOutput_inter;

%% Plot the states
figure(1)
set(gcf,'color','w')
subplot(5,1,1);
plot(tInput, inputStateStore(1:end,1),'r');
hold on
plot(tOutput,outputStateStore(:,1),'b');
xlabel('Time (s)')
ylabel('X position (m)');

subplot(5,1,2);
plot(tInput, inputStateStore(1:end,2),'r');
hold on
plot(tOutput,outputStateStore(:,2),'b');
xlabel('Time (s)')
ylabel('Y position (m)');

subplot(5,1,3);
plot(tInput, inputStateStore(1:end,3),'r');
hold on
plot(tOutput,outputStateStore(:,3),'b');
xlabel('Time (s)')
ylabel('X velocity (m)');

subplot(5,1,4);
plot(tInput, inputStateStore(1:end,4),'r');
hold on
plot(tOutput,outputStateStore(:,4),'b');
xlabel('Time (s)')
ylabel('Y velocity (m)');

subplot(5,1,5);
plot(tInput, inputStateStore(1:end,5),'r');
hold on
plot(tOutput,outputStateStore(:,5),'b');
xlabel('Time (s)')
ylabel('Force (N)');

%% Plot the entire trajectory
figure(2)
plot(outputStateStore(:,1), outputStateStore(:,2),'ko')
hold on
plot(outputStateStore(:,6), outputStateStore(:,7),'ro')
axis equal

%% Unpack the state variables
x  = outputStateStore(:,1);
y  = outputStateStore(:,2);
vx = outputStateStore(:,3);
vy = outputStateStore(:,4);
F  = outputStateStore(:,5);
xf = outputStateStore(:,6);
yf = outputStateStore(:,7);
wl = outputStateStore(:,8);

%% Some more plotting
figure(3)
subplot(3,1,1)
set(gcf,'color','w')
L = sqrt((x - xf).^2 + y.^2);
plot(tOutput,L);
xlabel('Time (s)')
ylabel('Leg length (m)')
ylim([0.5,1.1])

subplot(3,1,2)
Ldot = (x - xf).*vx + y.*vy;
plot(tOutput,Ldot);
xlabel('Time (s)')
ylabel('Leg length rate (m/s^-^1)')

subplot(3,1,3)
Ldotdot = F*L0/m - g*y + vx.^2 + vy.^2;
plot(tOutput,Ldotdot);
xlabel('Time (s)')
ylabel('Leg length rate rate (m/s^-^2)')

figure(4)
set(gcf,'color','w')
E = m*g*y + 0.5*m*(vx.^2 + vy.^2);
plot(tOutput,E);
xlabel('Time (s)')
ylabel('System energy (J)')

%% animate
figure(5)
set(gcf, 'color','w'); %,'Position', get(0, 'Screensize'));

% Plot the mass
hold on
leg_stance   = plot([xf(1),x(1)],[yf(1),y(1)],'k-','linewidth',2);
mass_point = plot(x(1),y(1),'ro','markerfacecolor','r','markersize',20);
ground_pre  = plot([-1,0.5*(xf(1)+xf(end))],[yf(1),yf(end)],'color',[0,0.8,0],'LineStyle','-','linewidth',3);
ground_post = plot([0.5*(xf(1)+xf(end)),2],[yf(1),yf(end)],'color',[0,0.8,0],'LineStyle','--','linewidth',3);

% Set axis properties
set(gca,'visible','off')
hold off
xlim([-1,2])
axis equal

% Set other useful animation properties
currFoot = xf(1);
avgSpeed = max(x)/length(tOutput);

% Run the animation
for i = 1:2:length(tOutput)
    if xf(i) ~= currFoot
        set(leg_stance,'color','b')
    end
    
    % Change the mass locations
    set(mass_point,'ydata',y(i));
    set(mass_point,'xdata',x(i));
 
    % Change the leg end-points
    set(leg_stance,'ydata',[yf(i), y(i)])
    set(leg_stance,'xdata',[xf(i), x(i)])
                
    pause(0.1);
end
