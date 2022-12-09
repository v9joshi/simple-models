% Top level file for running the  muscle metabolics models
clearvars; close all; clc;
% Version 1.0 - Setting muscle properties manually
% Load a file filled with muscles and their properties
load('./MuscleData/muscleParameters.mat')
load('./MuscleData/muscleStates.mat')

% Add paths to cost models
addpath('./Bhargava')
addpath('./Umberger')

% Set some body properties
bodyMass   = 76.80;
distanceTravelled = 0.7261;

basalRate  = 1.2; basalExp  = 1;
basalPower = basalRate*(bodyMass^basalExp);

% Read muscle properties from data files
muscleNames = muscleParameters.Name;
numMuscles  = length(muscleNames);

% Make a vector of times
timeVals = muscleStates.hamstrings_l.time;

% Initialize an array to store metabolic power for the body
bodyMetabolicPower_bgv04 = zeros(length(timeVals),7);
bodyMetabolicPower_bgv22 = zeros(length(timeVals),7);
bodyMetabolicPower_umb10 = zeros(length(timeVals),6);
bodyMetabolicPower_umb22 = zeros(length(timeVals),6);

% Initialize an array to store metabolic power for each muscle
muscleMetabolicPower_bgv = zeros(6, numMuscles);
muscleMetabolicPower_bgvNew = zeros(6, numMuscles);
muscleMetabolicPower_umb = zeros(5, numMuscles);
muscleMetabolicPower_umbNew = zeros(5, numMuscles);

% Loop through the time-points
for currTimeIndex = 1:length(timeVals)   

    % Find the metabolic power for each muscle
    for currMuscle = 1:length(muscleNames)
        currMuscleName = string(muscleNames(currMuscle));
        muscleMetabolicPower_bgv(:, currMuscle)     = bgv04_SingleMuscle(muscleParameters(currMuscle,:), muscleStates.(currMuscleName)(currTimeIndex,:));
        muscleMetabolicPower_bgvNew(:, currMuscle)  = bgv22_SingleMuscle(muscleParameters(currMuscle,:), muscleStates.(currMuscleName)(currTimeIndex,:));
        muscleMetabolicPower_umb(:, currMuscle)     = umb10_SingleMuscle(muscleParameters(currMuscle,:), muscleStates.(currMuscleName)(currTimeIndex,:));
        muscleMetabolicPower_umbNew(:, currMuscle)  = umb22_SingleMuscle(muscleParameters(currMuscle,:), muscleStates.(currMuscleName)(currTimeIndex,:));
    end
    
    % Sum up all the muscles to get the body metabolic power
    bodyMetabolicPower_bgv04(currTimeIndex,1:6) = sum(muscleMetabolicPower_bgv,2);
    bodyMetabolicPower_bgv22(currTimeIndex,1:6) = sum(muscleMetabolicPower_bgvNew,2);
    bodyMetabolicPower_umb10(currTimeIndex,1:5) = sum(muscleMetabolicPower_umb,2);
    bodyMetabolicPower_umb22(currTimeIndex,1:5) = sum(muscleMetabolicPower_umbNew,2);
    
   % Add in the basal power
    bodyMetabolicPower_bgv04(currTimeIndex,7) = 0;
    bodyMetabolicPower_bgv04(currTimeIndex,[1,6,7]) = bodyMetabolicPower_bgv04(currTimeIndex,[1,6,7]) + basalPower;
    
    bodyMetabolicPower_bgv22(currTimeIndex,7) = 0;
    bodyMetabolicPower_bgv22(currTimeIndex,[1,6,7]) = bodyMetabolicPower_bgv22(currTimeIndex,[1,6,7]) + basalPower;
    
    bodyMetabolicPower_umb10(currTimeIndex,6) = 0;
    bodyMetabolicPower_umb10(currTimeIndex,[1,6]) = bodyMetabolicPower_umb10(currTimeIndex,[1,6]) + basalPower;
    
    bodyMetabolicPower_umb22(currTimeIndex,6) = 0;
    bodyMetabolicPower_umb22(currTimeIndex,[1,6]) = bodyMetabolicPower_umb22(currTimeIndex,[1,6]) + basalPower;
end

% Integrate the metabolic power to get work
% Term orders:
% BGV
% 1 - Total work. 
% 2 - Mechanical work.
% 3 - Activation work.
% 4 - Maintenance work.
% 5 - Shortening/lengthening work.
% 6 - Smoothed total work.
% 7 - Basal/Resting work
costBreakdown_bgv04 = trapz(timeVals, bodyMetabolicPower_bgv04); 
costBreakdown_bgv22 = trapz(timeVals, bodyMetabolicPower_bgv22); 

% UMB
% 1 - Total work. 
% 2 - Mechanical work.
% 3 - Activation + Maintenance work.
% 4 - Shortening/lengthening work.
% 5 - Smoothed total work.
% 6 - Basal/Resting work.
costBreakdown_umb10 = trapz(timeVals, bodyMetabolicPower_umb10);
costBreakdown_umb22 = trapz(timeVals, bodyMetabolicPower_umb22);

% Convert work to cost of transport
costBreakdown_bgv04 = costBreakdown_bgv04(:)/(distanceTravelled*bodyMass);
costBreakdown_bgv22 = costBreakdown_bgv22(:)/(distanceTravelled*bodyMass);
costBreakdown_umb10 = costBreakdown_umb10(:)/(distanceTravelled*bodyMass);
costBreakdown_umb22 = costBreakdown_umb22(:)/(distanceTravelled*bodyMass);

%% Version 2.0 - Using opensim models
% Initialize a vector to store the cost of transport
costBreakdown_bgv04_Osim  = zeros(1,7);

% Set the model name and file name
modelName = './OSimFiles/testModel.osim';
fileName  = './OSimFiles/predictionSolution.sto';
% 
% Import the libraries
import org.opensim.modeling.*;
% 
% Load up the model
model = Model(modelName);
% 
% Make a state object
osimState = model.initSystem();
% 
% % Load the solution file
gaitPredictionSolution = MocoTrajectory(fileName);
% 
% % Get the time lists and states lists
statesVals = gaitPredictionSolution.getStatesTrajectoryMat;
controlVals = gaitPredictionSolution.getControlsTrajectoryMat;
timeVals = gaitPredictionSolution.getTimeMat;

% Initialize a vector to store metabolic power
bodyMetabolicPower_bgv04_Osim = zeros(length(timeVals),7);

for currTimeIndex = 1:length(timeVals)
    [bodyMetabolicPower_bgv04_Osim(currTimeIndex, :), ~] = bgv04_opensim(timeVals(currTimeIndex), statesVals(currTimeIndex,:), controlVals(currTimeIndex,:), model, osimState);
end

% Integrate the metabolic power to get work
costBreakdown_bgv04_Osim = trapz(timeVals, bodyMetabolicPower_bgv04_Osim); % Total work

% Find distance travelled
distanceVal_Pelvis = statesVals(end,2) - statesVals(1,2);

% Find the system mass
modelMass = model.getTotalMass(osimState);

% Turn the metabolic cost into a CoT
costBreakdown_bgv04_Osim = costBreakdown_bgv04_Osim(:)/(distanceVal_Pelvis*modelMass);

%% Plot the results
figure(1)
set(gcf, 'color','w')
subplot(1,2,1)
plot(timeVals, bodyMetabolicPower_bgv04(:,[2:5,7]),'linewidth',1.5)
legend({'Wdot','Adot','Mdot','Sdot','Bdot'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,1000])
title('Individual components')

subplot(1,2,2)
plot(timeVals, bodyMetabolicPower_bgv04(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_bgv22(:,1),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb10(:,1),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb22(:,1),'linewidth',1.5)
legend({'Edot - bgv04','Edot - bgv22','Edot - umb10','Edot - umb22'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff

ylabel('Power (W)')
xlabel('Time (s)')
ylim([0,1000])
title('Total power')

%% Plot individual terms
figure(2)
set(gcf,'Color','w')
tile_handle = tiledlayout(1,2);
nexttile()
plot(timeVals, bodyMetabolicPower_bgv04(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_bgv04(:,2),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv04(:,3),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv04(:,4),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv04(:,5),'linewidth',1.5)
legend({'Total power - bgv04','Mechanical power - bgv04','Activation power - bgv04','Maintenance power - bgv04','Shortening power - bgv04'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,1500])
title('BGV 04')

nexttile
plot(timeVals, bodyMetabolicPower_bgv22(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_bgv22(:,2),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv22(:,3),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv22(:,4),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_bgv22(:,5),'linewidth',1.5)
legend({'Total power - bgv22','Mechanical power - bgv22','Activation power - bgv22','Maintenance power - bgv22','Shortening power - bgv22'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,1500])
title('BGV 22')

ylabel(tile_handle, 'Power (W)')
xlabel(tile_handle, 'Time (s)')

figure(3)
set(gcf,'Color','w')
tile_handle = tiledlayout(1,2);
nexttile()
plot(timeVals, bodyMetabolicPower_umb10(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_umb10(:,2),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb10(:,3),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb10(:,4),'linewidth',1.5)
legend({'Total power - umb10','Mechanical power - umb10','Activation +Maintenance` power - umb10','Shortening power - umb10'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,1500])
title('UMB 10')

nexttile()
plot(timeVals, bodyMetabolicPower_umb22(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_umb22(:,2),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb22(:,3),'linewidth',1.5)
plot(timeVals, bodyMetabolicPower_umb22(:,4),'linewidth',1.5)
legend({'Total power - umb22','Mechanical power - umb22','Activation +Maintenance` power - umb22','Shortening power - umb22'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,1500])
title('UMB 22')

ylabel(tile_handle, 'Power (W)')
xlabel(tile_handle, 'Time (s)')


% %%
% figure(2)
% set(gcf, 'color','w')
% subplot(1,2,1)
% plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,[2:5,7]),'linewidth',1.5)
% legend({'Wdot','Adot','Mdot','Sdot','Bdot'},'location','northeast')
% set(gca,'TickDir','out','box','off','linewidth',2)
% legend boxoff
% ylim([0,700])
% title('Individual components')
% 
% subplot(1,2,2)
% plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,1),'linewidth',1.5)
% hold on
% plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,6),'linewidth',1.5)
% legend({'Edot','Edot - smooth'},'location','northeast')
% set(gca,'TickDir','out','box','off','linewidth',2)
% legend boxoff
% ylim([0,700])
% title('Total power')