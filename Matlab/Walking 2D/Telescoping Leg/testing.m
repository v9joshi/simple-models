% Root file
close all;

% Set the parameters
g = 10;           % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;           % Mass of the human, concentrated at a single point
Fmax = 200*m*g;
stepLength = 0.3;        % The target step length - Unused
Speed = 1.0;

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
nSteps  = 2;
nStates = 7;
nInputs = 7;
nSeg    = 20;
nVars = nInputs*nSeg + 4;

% Pack these params
params.nSteps = nSteps;
params.nStates = nStates;
params.nSeg = nSeg;
params.nVars = nVars;
params.nInputs = nInputs;

%% Set the initial guess
input0 = [state0(1:nStates - 2), delF, delT];
var0 = repmat(input0, nSeg, 1);
var0 = reshape(var0, [], 1);
impPO = 1;
impHS = 1;
var0 = [var0; footX; footY; impHS; impPO];
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
xfmin   = 0;            xfmax   = 2*nSteps;
yfmin   = 0;            yfmax   = 1e-6;
delFmin = -Fmax*1000;   delFmax = Fmax*1000;
delTmin = 0.001;        delTmax = 0.1;
impMin  = 0;            impMax  = 1e-6;

LBstate = [xmin,ymin,vxmin,vymin,Fmin,xfmin,yfmin];
UBstate = [xmax,ymax,vxmax,vymax,Fmax,xfmax,yfmax];

LBinput = [LBstate(1:nStates-2), delFmin, delTmin];
UBinput = [UBstate(1:nStates-2), delFmax, delTmax];

LBvar = repmat(LBinput, nSeg,1);
LBvar = reshape(LBvar, [], 1);
LBvar = [LBvar; LBstate(end-1); LBstate(end); impMin; impMin]; 
LB = repmat(LBvar, nSteps,1);

UBvar = repmat(UBinput, nSeg,1);
UBvar = reshape(UBvar, [], 1);
UBvar = [UBvar; UBstate(end-1); UBstate(end); impMax; impMax]; 
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
% sampleVal = load('DecentStartingSol.mat');
% var0 = sampleVal.xOut;

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

% Loop through the steps
for stepNum = 1:nSteps
   currStepVar = stepVars(:, stepNum);

   % Read the impulses and foot positions
   currXf = currStepVar(end-3);
   currYf = currStepVar(end-2);

   currHS = currStepVar(end-1);
   currPO = currStepVar(end);

   % Read the seg vars for each segment
   segVars = reshape(currStepVar(1:end-4)', nSeg, nInputs);  

   for segNum = 1:nSeg
      currSegVar =  segVars(segNum,:);

      % Read delF and delT
      currDelF   =  currSegVar(end-1);
      currDelT   =  currSegVar(end);

      % Read the states
      currSegState = [currSegVar(1:end-2), currXf, currYf];
      inputStateStore = [inputStateStore; currSegState];
   end
end

%% Run the result
[tStore, outputStateStore] = nSteps_TelescopingLeg(xOut, params);

figure(1)
subplot(5,1,1);
plot([0,tStore(1:end-1)], inputStateStore(1:end,1),'r');
hold on
plot(tStore,outputStateStore(:,1),'b');
xlabel('Time (s)')
ylabel('X position (m)');

subplot(5,1,2);
plot([0,tStore(1:end-1)], inputStateStore(1:end,2),'r');
hold on
plot(tStore,outputStateStore(:,2),'b');
xlabel('Time (s)')
ylabel('Y position (m)');

subplot(5,1,3);
plot([0,tStore(1:end-1)], inputStateStore(1:end,3),'r');
hold on
plot(tStore,outputStateStore(:,3),'b');
xlabel('Time (s)')
ylabel('X velocity (m)');

subplot(5,1,4);
plot([0,tStore(1:end-1)], inputStateStore(1:end,4),'r');
hold on
plot(tStore,outputStateStore(:,4),'b');
xlabel('Time (s)')
ylabel('Y velocity (m)');

subplot(5,1,5);
plot([0,tStore(1:end-1)], inputStateStore(1:end,5),'r');
hold on
plot(tStore,outputStateStore(:,5),'b');
xlabel('Time (s)')
ylabel('Force (N)');

%% Plot the entire trajectory
figure(2)
plot(outputStateStore(:,1), outputStateStore(:,2),'ko')
hold on
plot(outputStateStore(:,6), outputStateStore(:,7),'ro')
axis equal
