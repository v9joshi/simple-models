% Program to Simulate Spring-Mass hopper
clear all; close all; clc;

% Parameters
g = 10; k = 2800; L0 = 1; m = 70;

% Packing
params.g = g; params.k =k; params.L0 = L0; params.m = m;

% Initial conditions and time list
y0 = 1.5;
v0 = 0;

tStart = 0;
tspan = linspace(tStart,10,1000);

StateVar0 = [y0; v0];

StateStore = [];
tlist = [];

%% Plug in the ode files for flight/contact and event functions for phase switching
ODE_flight = @(t,statevar) ODE_Flight(t,statevar,params);
ODE_contact = @(t,statevar) ODE_Contact_SpringForce(t,statevar,params);
Event_1 = @(t,statevar) Event_FlightToContact(t,statevar,params);
Event_2 = @(t,statevar) Event_ContactToFlight(t,statevar,params);

%% Hop 12 times
for i = 1:12 
    % Simulate the flight phase
    options = odeset('reltol',1e-9,'abstol',1e-9,'Events',Event_1);
    [tlist1,StateVarlist1] = ode45(ODE_flight,tspan,StateVar0,options);
    
    StateStore = [StateStore; StateVarlist1(1:end-1,:)]; % leaving out the last point to avoid repetition
    tlist = [tlist; tStart + tlist1(1:end-1)];
    tStart = tStart + tlist1(end);
    
    % Simulate the contact phase
    options = odeset('reltol',1e-9,'abstol',1e-9,'Events',Event_2);
    StateVar0 = StateVarlist1(end,:)';
    [tlist2,StateVarlist2] = ode45(ODE_contact,tspan,StateVar0,options);
    
    StateStore = [StateStore; StateVarlist2(1:end-1,:)];
    tlist = [tlist; tStart + tlist2(1:end-1)];
    tStart = tStart + tlist2(end);
    
    StateVar0 = StateVarlist2(end,:);
end

%% Unpack the data and save it
ylist = StateStore(:,1);
vlist = StateStore(:,2);

save('hopper_spring_force.mat','tlist','vlist','ylist')
%% Plot the data
figure(2)

subplot(2,1,1)
plot(tlist,ylist,'b');

title('Y Co-ordinate vs Time');
xlabel('Time (s)');
ylabel('Y Co-ordinate (m)');

subplot(2,1,2)
plot(tlist,vlist,'b');

title('Y speed vs Time');
xlabel('Time (s)');
ylabel('Y speed (m s ^-^1)');

% Calculate and Plot Force
F = zeros(length(tlist),1);
for i = 1:length(tlist)
    if ylist(i) > L0
        F(i) = 0;
    else
        F(i) = m*g + k*(L0 - ylist(i));
    end
end

figure(3)
plot(tlist,F)
title('Ground Reaction Force vs Time')
xlabel('Time (s)');
ylabel('Force (N)');

%% Animate the hopper
figure(1)
set(gcf, 'color','w')
mass_handle = plot(0,ylist(1),'ro','markerfacecolor','r');
hold on
leg_handle = plot([0,0],[max(ylist(1) - L0,0), ylist(1)],'k--');
plot([-3,3],[0,0],'color',[0,0.5,0],'LineStyle','-','linewidth',3)
axis([-3 3 -1 max(ylist)+1]);
set(gca,'visible','off')
hold off

for i = 2:10:length(tlist)
    set(mass_handle,'ydata',ylist(i));
    set(leg_handle,'ydata',[max(ylist(i) - L0,0), ylist(i)])
    pause(0.01);
end
