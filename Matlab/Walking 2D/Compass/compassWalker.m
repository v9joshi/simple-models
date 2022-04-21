% Compass walker
clearvars; close all;

% Set the parameters
g =  10;          % acceleration due to gravity
L0 = 1;           % Length of the leg
m  = 1;       % Mass of the foot, concentrated at a single point
M = 70;           % Mass of the HAT, concentrated at a single point
gamma = -0.03;    % Slope of the ground, in radians

% Pack parameters
params.g = g; params.L0 = L0; params.m = m; params.M = M; params.gamma = gamma;

%brachiate 0 -0.1516    2    4
initX = [-0.260417722506022  -0.317844122214875   0.852760447627659   0.155466284509077];

% Initial conditions
x00 = 0;                y00 = 0;              
x10 = initX(1);         vx10 = initX(3);
x20 = x10 + initX(2);   vx20 = initX(4);
      
% Enforce leg length constraints
% Stance leg
y10 =  y00 + sqrt(L0^2 - (x10 - x00)^2);             
vy10 = -(x10 - x00)*vx10/(y10 - y00);

% Swing leg
y20 = y10 - sqrt(L0^2 - (x20 - x10)^2);
vy20 = vy10 - (x20 - x10)*(vx20 - vx10)/(y20 - y10);


% Time settings
t0 = 0;         % Starting time.
tmax = 30;      % This must be larger than step time.

% Pack the states together
state0   = [x10; x20; y10; y20; vx10; vx20; vy10; vy20; x00; y00];
tSpan    = linspace(t0,tmax,tmax*1000);

% Set the ODE files
ODE_walk        = @(t,statevar) ODE_2DCompass(t,statevar,params);
Event_walk      = @(t,statevar) HSEvent_2DCompass(t,statevar,params);
contactFunction = @(t,statevar) Contact_2DCompass(t,statevar,params);

%% Simulate the motion
stateStore = [];
timeStore  = [];
collTimeStore = [];

% simulate movement
options = odeset('reltol',1e-12,'abstol',1e-12,'Events',Event_walk);

while t0 < tmax
    [tListOut,stateListOut, te,ye,ie] = ode15s(ODE_walk,tSpan,state0,options);

    stateStore = [stateStore; stateListOut(1:end-1,:)]; % leaving out the last point to avoid repetition
    timeStore = [timeStore; t0 + tListOut(1:end-1)];

    % Apply heel-strike and push-off impulses
    if ~isempty(te)
        disp('collision')
        collTimeStore = [collTimeStore, te(end) + t0];
        state0 = contactFunction(t0, stateListOut(end,:));
    else
        state0 = stateListOut(end,:);
    end
    
    % Accounting for integration errors
    x_diff(1)  = state0(9)  - x00 - state0(9);
    x_diff(2)  = state0(1)  - x10 - state0(9);
    x_diff(3)  = state0(2)  - x20 - state0(9);
    
    x_diff(4)  = state0(10) - y00 - state0(10);
    x_diff(5)  = state0(3)  - y10 - state0(10);
    x_diff(6)  = state0(4)  - y20 - state0(10);
    
    x_diff(7)  = state0(5)  - vx10;     x_diff(8)  = state0(6)  - vx20;
    x_diff(9)  = state0(7)  - vy10;     x_diff(10) = state0(8)  - vy20;
    
    tot_diff = x_diff;
    
    % Only apply this trick if it is clear that we're dealing with
    % integration errors.
    if sum(tot_diff.^2) < 1e-9
        state0 = [x10 + state0(9);
                  x20 + state0(9);
                  y10 + state0(10);
                  y20 + state0(10);
                  vx10;
                  vx20;
                  vy10;
                  vy20;
                  state0(9); state0(10)];
    end
    
    % Update the start time
    t0 = t0 + tListOut(end);
end

%% Unpack the state variables
x1  = stateStore(:,1); x2  = stateStore(:,2);
y1  = stateStore(:,3); y2  = stateStore(:,4);
vx1 = stateStore(:,5); vx2 = stateStore(:,6);
vy1 = stateStore(:,7); vy2 = stateStore(:,8);
xf  = stateStore(:,9); yf  = stateStore(:,10);

%% Check total energy and foot position
kineticEnergy = 0.5*M*(vx1.^2 + vy1.^2) + 0.5*m*(vx2.^2 + vy2.^2);
totalEnergy   = kineticEnergy + M*g*y1 + m*g*y2 + m*g*yf;

figure(1)
set(gcf, 'color','w')
hold on
plot(timeStore, kineticEnergy);
plot(timeStore, totalEnergy);
plot(collTimeStore,kineticEnergy(1)*ones(size(collTimeStore,1), size(collTimeStore,2)),'r*')
legend('kinetic','total')
hold off

footErrorY = y2 - yf - (x2 - xf).*tan(gamma);
footErrorX = x2 - xf - (y2 - yf)./tan(gamma);

figure(2)
set(gcf, 'color','w')
hold on
plot(timeStore, footErrorY);
plot(timeStore, footErrorX);
plot(collTimeStore, zeros(size(collTimeStore,1), size(collTimeStore,2)),'r*')
hold off
legend('Y error','X error')

%% Animate the compass walker
figure(4)
set(gcf, 'color','w')
link1 = plot([xf(1), x1(1)],[yf(1), y1(1)],'b-','marker','o','markerfacecolor','b');
hold on
link2 = plot([x2(1), x1(1)],[y2(1), y1(1)],'r-','marker','o','markerfacecolor','r');
ground = plot([xf(1) - 3*cos(gamma), xf(1) + 25*cos(gamma)],[yf(1) - 3*sin(gamma), yf(1) + 25*sin(gamma)],'color',[0,0.2,0],'LineStyle','-','linewidth',3);
hold off
xlim([-3,3])
ylim([-3,3])
axis equal
set(gca, 'visible','off')
animAx = gca;

% Do we want to write to a gif? If yes, specify file name.
% gifFileName = "compass_InfSteps.gif";

% What foot are we starting on?
currFoot = [xf(1), yf(1)];

for i = 1:100:length(timeStore)
    
    % Check if foot changed
    if ~all([xf(i), yf(i)] == currFoot)
        currFoot = [xf(i), yf(i)];
        swapper = link1;
        link1 = link2;
        link2 = swapper;
    end        
        
    set(link1, 'xdata',[xf(i), x1(i)],'ydata',[yf(i), y1(i)])
    set(link2, 'xdata',[x2(i), x1(i)],'ydata',[y2(i), y1(i)])
    animAx.XLim = [x1(i) - 5,x1(i) + 5];
    pause(0.01)
    
    if exist('gifFileName','var')
        F = getframe(gcf);
        im = frame2im(F);
        [imind, cm] = rgb2ind(im,256);

        % Write the frame to the gif file
        if i == 1
            imwrite(imind, cm, gifFileName,'gif','DelayTime',0,'Loopcount',inf);
        else
            imwrite(imind, cm, gifFileName,'gif','DelayTime',0,'WriteMode','append');
        end 
    end
end