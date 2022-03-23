% Set some constants
L0 = 1; % Length of the leg

%% Load data from all the hoppers
springHopper     = load('hopper_spring_force.mat');
constantHopper   = load('hopper_constant_force.mat');
sinusoidalHopper = load('hopper_sinusoidal_force.mat');

%% sync the hoppers
minTime = max([springHopper.tlist(1), constantHopper.tlist(1), sinusoidalHopper.tlist(1)]);
maxTime = min([springHopper.tlist(end), constantHopper.tlist(end), sinusoidalHopper.tlist(end)]);
tlist = linspace(minTime,maxTime,2000);

% Do some interpolation
ySpring = interp1(springHopper.tlist, springHopper.ylist, tlist);
yConst  = interp1(constantHopper.tlist, constantHopper.ylist, tlist);
ySinus = interp1(sinusoidalHopper.tlist, sinusoidalHopper.ylist, tlist);
 
%% animate
figure(1)
set(gcf, 'color','w')
subplot(1,2,1)
mass_spring = plot(-1,ySpring(1),'ro','markerfacecolor','r');
hold on
mass_const = plot(0,yConst(1),'bo','markerfacecolor','b');
mass_sinus = plot(1,ySinus(1),'ko','markerfacecolor','k');

leg_spring = plot([-1,-1],[max(ySpring(1) - L0,0), ySpring(1)],'k--');
leg_const = plot([0,0],[max(yConst(1) - L0,0), yConst(1)],'k--');
leg_sinus = plot([1,1],[max(ySinus(1) - L0,0), ySinus(1)],'k--');

plot([-3,3],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
axis([-3 3 -1 max(ylist)+1]);
set(gca,'visible','off')
hold off

subplot(3,2,2)
plot(tlist, ySpring,'r')
hold on
vertline_spring = plot([tlist(1), tlist(1)],[min(ySpring), max(ySpring)],'k--');
point_spring = plot(tlist(1), ySpring(1),'ro','markerfacecolor','r');
hold off
axis([min(tlist) max(tlist) min(ySpring) max(ySpring)]);
ylabel('mass height (m)')

subplot(3,2,4)
plot(tlist, yConst,'b')
hold on
vertline_const = plot([tlist(1), tlist(1)],[min(yConst), max(yConst)],'k--');
point_const = plot(tlist(1), yConst(1),'bo','markerfacecolor','b');
hold off
axis([min(tlist) max(tlist) min(yConst) max(yConst)]);
ylabel('mass height (m)')

subplot(3,2,6)
plot(tlist, ySinus,'k')
hold on
vertline_sinus = plot([tlist(1), tlist(1)],[min(ySinus), max(ySinus)],'k--');
point_sinus = plot(tlist(1), ySinus(1),'ko','markerfacecolor','k');
hold off
axis([min(tlist) max(tlist) min(ySinus) max(ySinus)]);
ylabel('mass height (m)')
xlabel('time (s)')

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
    
    % Move the vertical scan line
    set(vertline_spring,'xdata',[tlist(i), tlist(i)])
    set(vertline_const,'xdata',[tlist(i), tlist(i)])
    set(vertline_sinus,'xdata',[tlist(i), tlist(i)])
    
    % Move the point
    set(point_spring,'xdata',tlist(i), 'ydata', ySpring(i))
    set(point_const,'xdata',tlist(i), 'ydata',yConst(i))
    set(point_sinus,'xdata',tlist(i),'ydata',ySinus(i))
        
    pause(0.01);
end
