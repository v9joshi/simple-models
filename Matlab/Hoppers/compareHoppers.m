% Set some constants
y0 = 1.5; v0 = 0; % Initial state
L0 = 1;  % Length of the leg
m  = 70; % body mass
g  = 10; % gravitational acc

% Set the control parameters for the hoppers
k = 2881; % Spring constant for spring hopper
F0_const = 1400; % Force value for constant force hopper
F0_sinus = 700*pi; % Max force for sinusoidal hopper
T0 = 2/(pi*sqrt(10)); % Time constant for sinusoidal hopper

%% Load data from all the hoppers
% springHopper     = load('hopper_spring_force.mat');
% constantHopper   = load('hopper_constant_force.mat');
% sinusoidalHopper = load('hopper_sinusoidal_force.mat');

springHopper     = Root_Hopping_SpringForce(y0, v0, m, g, L0, k, 0);
constantHopper   = Root_Hopping_ConstantForce(y0, v0, m, g, L0, F0_const, 0);
sinusoidalHopper = Root_Hopping_SinusoidalForce(y0, v0, m, g, L0, F0_sinus, T0, 0);

%% sync the hoppers
minTime = max([springHopper.tlist(1), constantHopper.tlist(1), sinusoidalHopper.tlist(1)]);
maxTime = min([springHopper.tlist(end), constantHopper.tlist(end), sinusoidalHopper.tlist(end)]);
tlist = linspace(minTime,maxTime,2000);

% Do some interpolation
ySpring = interp1(springHopper.tlist, springHopper.ylist, tlist);
yConst  = interp1(constantHopper.tlist, constantHopper.ylist, tlist);
ySinus = interp1(sinusoidalHopper.tlist, sinusoidalHopper.ylist, tlist);

fSpring = interp1(springHopper.tlist, springHopper.Flist, tlist)*100/(m*g);
fConst  = interp1(constantHopper.tlist, constantHopper.Flist, tlist)*100/(m*g);
fSinus  = interp1(sinusoidalHopper.tlist, sinusoidalHopper.Flist, tlist)*100/(m*g);
 
%% animate
figure(4)
set(gcf, 'color','w','Position', get(0, 'Screensize'));

% Plot the hoppers
subplot(1,2,1)
mass_spring = plot(-1,ySpring(1),'ro','markerfacecolor','r','markersize',20);
hold on
mass_const = plot(0,yConst(1),'bo','markerfacecolor','b','markersize',20);
mass_sinus = plot(1,ySinus(1),'ko','markerfacecolor','k','markersize',20);

leg_spring = plot([-1,-1],[max(ySpring(1) - L0,0), ySpring(1)],'k--');
leg_const = plot([0,0],[max(yConst(1) - L0,0), yConst(1)],'k--');
leg_sinus = plot([1,1],[max(ySinus(1) - L0,0), ySinus(1)],'k--');

plot([-3,3],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
axis([-3 3 -1 max(ySpring)+1]);
set(gca,'visible','off')
hold off
legend([mass_spring, mass_const, mass_sinus],{'Spring', 'Constant', 'Sinusoidal'},'orientation','horizontal')

% Plot the CoM height
subplot(2,2,2)
% Plot the time series
plot(tlist, ySpring,'r')
hold on
plot(tlist, yConst,'b')
plot(tlist, ySinus,'k')
% Make a vertical line for the current time
vertline_spring = plot([tlist(1), tlist(1)],[min(ySpring), max(ySpring)],'k--');

% Mark a point at the current time in the time-series
point_spring = plot(tlist(1), ySpring(1),'ro','markerfacecolor','r');
point_const = plot(tlist(1), yConst(1),'bo','markerfacecolor','b');
point_sinus = plot(tlist(1), ySinus(1),'ko','markerfacecolor','k');
hold off
axis([min(tlist) max(tlist) 0 2]);
ylabel('mass height (m)')

% Plot the contact forces
subplot(2,2,4)
bar(-1,max(fSpring),'edgecolor','r','facecolor','w');
hold on
bar_spring = bar(-1,fSpring(1),'r');
bar(0,max(fConst),'edgecolor','b','facecolor','w');
bar_const = bar(0,fConst(1),'b');
bar(1,max(fSinus),'edgecolor','k','facecolor','w');
bar_sinus = bar(1,fSinus(1),'k');
hold off
ylim([0, max([max(fSpring), max(fConst),max(fSinus)])*1.2])
xlim([-3,3])
ylabel('Contact force (%Body weight)')
set(gca,'box','off')
xticks([-1, 0, 1])
xticklabels({'Spring', 'Constant', 'Sinusoidal'})

% Run the animation
for i = 2:2:length(tlist)
    % Change the mass locations
    set(mass_spring,'ydata',ySpring(i));
    set(mass_const,'ydata',yConst(i));
    set(mass_sinus,'ydata',ySinus(i));

    % Change the leg end-points
    set(leg_spring,'ydata',[max(ySpring(i) - L0,0), ySpring(i)])
    set(leg_const,'ydata',[max(yConst(i) - L0,0), yConst(i)])
    set(leg_sinus,'ydata',[max(ySinus(i) - L0,0), ySinus(i)])
    
    % Modify the bars
    set(bar_spring,'ydata',fSpring(i));
    set(bar_const,'ydata',fConst(i));
    set(bar_sinus,'ydata',fSinus(i));
    
    % Move the vertical scan line
    set(vertline_spring,'xdata',[tlist(i), tlist(i)])
   
    % Move the point
    set(point_spring,'xdata',tlist(i), 'ydata', ySpring(i))
    set(point_const,'xdata',tlist(i), 'ydata',yConst(i))
    set(point_sinus,'xdata',tlist(i),'ydata',ySinus(i))
        
    pause(0.01);
end
