% Top level file for running the Bhargava 2004 muscle metabolics model
clearvars; close all; clc;
% Version 1.0 - Setting muscle properties manually
% Load a file filled with muscles and their properties
load('./MuscleData/muscleParameters.mat')
load('./MuscleData/muscleStates.mat')

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

% Initialize an array to store metabolic power for each muscle
muscleMetabolicPower = zeros(6, numMuscles);

% Loop through the time-points
for currTimeIndex = 1:length(timeVals)   

    % Find the metabolic power for each muscle
    for currMuscle = 1:length(muscleNames)
        currMuscleName = string(muscleNames(currMuscle));
        muscleMetabolicPower(:, currMuscle) = bgv04_SingleMuscle(muscleParameters(currMuscle,:), muscleStates.(currMuscleName)(currTimeIndex,:));
    end
    
    % Sum up all the muscles to get the body metabolic power
    bodyMetabolicPower_bgv04(currTimeIndex,1:6) = sum(muscleMetabolicPower,2);
    
   % Add in the basal power
    bodyMetabolicPower_bgv04(currTimeIndex,7) = 0;
    bodyMetabolicPower_bgv04(currTimeIndex,[1,6,7]) = bodyMetabolicPower_bgv04(currTimeIndex,[1,6,7]) + basalPower;    
end

% Integrate the metabolic power to get work
costBreakdown_bgv04(1) = trapz(timeVals, bodyMetabolicPower_bgv04(:,1)); % Total work
costBreakdown_bgv04(2) = trapz(timeVals, bodyMetabolicPower_bgv04(:,2)); % Mechanical work
costBreakdown_bgv04(3) = trapz(timeVals, bodyMetabolicPower_bgv04(:,3)); % Activation work
costBreakdown_bgv04(4) = trapz(timeVals, bodyMetabolicPower_bgv04(:,4)); % Maintenance work
costBreakdown_bgv04(5) = trapz(timeVals, bodyMetabolicPower_bgv04(:,5)); % Shortening work
costBreakdown_bgv04(6) = trapz(timeVals, bodyMetabolicPower_bgv04(:,6)); % Smoothed total work
costBreakdown_bgv04(7) = trapz(timeVals, bodyMetabolicPower_bgv04(:,7)); % Basal heat rate

% Convert work to cost of transport
costBreakdown_bgv04 = costBreakdown_bgv04(:)/(distanceTravelled*bodyMass);

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
costBreakdown_bgv04_Osim(1) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,1)); % Total work
costBreakdown_bgv04_Osim(2) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,2)); % Mechanical work
costBreakdown_bgv04_Osim(3) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,3)); % Activation work
costBreakdown_bgv04_Osim(4) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,4)); % Maintenance work
costBreakdown_bgv04_Osim(5) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,5)); % Shortening work
costBreakdown_bgv04_Osim(6) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,6)); % Smoothed total work
costBreakdown_bgv04_Osim(7) = trapz(timeVals, bodyMetabolicPower_bgv04_Osim(:,7)); % Basal heat rate

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
ylim([0,700])
title('Individual components')

subplot(1,2,2)
plot(timeVals, bodyMetabolicPower_bgv04(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_bgv04(:,6),'linewidth',1.5)
legend({'Edot','Edot - smooth'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,700])
title('Total power')

figure(2)
set(gcf, 'color','w')
subplot(1,2,1)
plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,[2:5,7]),'linewidth',1.5)
legend({'Wdot','Adot','Mdot','Sdot','Bdot'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,700])
title('Individual components')

subplot(1,2,2)
plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,1),'linewidth',1.5)
hold on
plot(timeVals, bodyMetabolicPower_bgv04_Osim(:,6),'linewidth',1.5)
legend({'Edot','Edot - smooth'},'location','northeast')
set(gca,'TickDir','out','box','off','linewidth',2)
legend boxoff
ylim([0,700])
title('Total power')


