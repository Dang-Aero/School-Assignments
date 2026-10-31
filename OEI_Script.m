clc;
clear;
close all;

%% Aero characteristics
airDensity = 0.001850;   % slug/ft^3
gravity = 32.174;        % ft/s^2
wingArea = 1372.0;         % ft^2
wingSpan = 52.4;        % ft
takeoffSpeed = 377.9;      % ft/s
dynamicPressure = 0.5 * airDensity * takeoffSpeed^2;
verticalTailArea = 260;   % ft^2
tailArm = 15.39;         % ft
engineArm = 15;           % ft
takeoffWeight = 151000;  % lb
mu = 0.02;
takeoffDistance = 4105.4; % ft
CriticalFieldLength = takeoffDistance * 1.15
dragAtTakeoff = 7474.5;   % lb
engineDragAtTakeoff = 1000; % lb
liftAtTakeoff = takeoffWeight;

%% Rudder deflection
thrustPerEngine = 88000 / 4;  % lb

k = 0.7;
nV = 1;
vTailRatio = tailArm * verticalTailArea / (wingArea * wingSpan);
aV = 4.5;
tau2 = 0.5;
cnDeltaR = -k * nV * vTailRatio * aV * tau2;

rudderDeflection = -engineArm * (thrustPerEngine + engineDragAtTakeoff) / ...
    (dynamicPressure * wingArea * wingSpan) / cnDeltaR * 180 / pi

%% Normal takeoff Ground Run
speeds = linspace(0.1, takeoffSpeed, 100);

dragAtSpeed = interp1([0 takeoffSpeed], [0 dragAtTakeoff], speeds);
liftAtSpeed = interp1([0 takeoffSpeed], [0 liftAtTakeoff], speeds);

f1 = thrustPerEngine * 4 - dragAtSpeed - mu * (takeoffWeight - liftAtSpeed);
f0 = thrustPerEngine * 4 - mu * takeoffWeight;

distanceAeo = takeoffWeight ./ (2 * gravity) .* speeds.^2 ./ (f0 - f1) .* log(f0 ./ f1);

%% Normal takeoff Liftoff Run
LiftAngle = asin((thrustPerEngine*4 - dragAtTakeoff) / takeoffWeight);
hOBS = 35;
TakeoffRunDistance = hOBS / tan(LiftAngle);


%% Engine failure Ground Run
oeiSpeed = 200;

% Normal-takeoff force at the actual failure speed
f1AtOei = interp1(speeds, f1, oeiSpeed, 'linear');

% Speeds after failure
oeiSpeedArray = speeds(speeds >= oeiSpeed);

dragOei = interp1([0 takeoffSpeed], ...
    [0 dragAtTakeoff + engineDragAtTakeoff], oeiSpeedArray);

liftOei = interp1([0 takeoffSpeed], [0 liftAtTakeoff], oeiSpeedArray);

f1Oei = thrustPerEngine * 3 - dragOei - mu * (takeoffWeight - liftOei);

distanceOei = takeoffWeight ./ (2 * gravity) .* oeiSpeedArray.^2 ./ ...
    (f1AtOei - f1Oei) .* log(f1AtOei ./ f1Oei);

distanceAeoOei = [distanceAeo(speeds < oeiSpeed), distanceOei];
SpeedsAeoOei = [speeds(speeds < oeiSpeed), oeiSpeedArray];

%% Normal takeoff Liftoff Run
LiftAngle = asin((thrustPerEngine*3 - dragAtTakeoff - engineDragAtTakeoff) / takeoffWeight);
hOBS = 35;
TakeoffRunDistance = hOBS / tan(LiftAngle);

%% Time versus distance Derivative
timeAeo = zeros(size(distanceAeo));
for k = 2:numel(distanceAeo)
    dx = distanceAeo(k) - distanceAeo(k - 1);
    vAvg = 0.5 * (speeds(k) + speeds(k - 1));
    timeAeo(k) = timeAeo(k - 1) + dx / vAvg;
end

timeAeoOei = zeros(size(distanceAeoOei));
for k = 2:numel(distanceAeoOei)
    dx = distanceAeoOei(k) - distanceAeoOei(k - 1);
    vAvg = 0.5 * (SpeedsAeoOei(k) + SpeedsAeoOei(k - 1));
    timeAeoOei(k) = timeAeoOei(k - 1) + dx / vAvg;
end

%% Abort braking distance 
decisionSpeed = 235;  % ft/s
decisionSpeedPlot = decisionSpeed * ones(size(distanceAeoOei));

reverseThrust = 1;
muBrake = 0.5;

liftAtDecision = interp1([0 takeoffSpeed], [0 liftAtTakeoff], decisionSpeed);
dragAtDecision = interp1([0 takeoffSpeed], ...
    [0 dragAtTakeoff + engineDragAtTakeoff], decisionSpeed);

f0Stop = reverseThrust + muBrake * takeoffWeight;
fEfStop = reverseThrust + dragAtDecision + muBrake * (takeoffWeight - liftAtDecision);

stopDistance = takeoffWeight / (2 * gravity) * ...
    (decisionSpeed^2) / (fEfStop - f0Stop) * log(fEfStop / f0Stop);

%% Intercept between decision speed line and engine failure curve
intersectionSpeed = decisionSpeed;
intersectionDistance = interp1(speeds, distanceAeo, intersectionSpeed, 'linear');

if isnan(intersectionDistance)
    warning('Decision speed is outside the speed range.');
end

stopDistancePlot = [intersectionDistance, intersectionDistance + stopDistance];
stopSpeedPlot = [decisionSpeed, 0];

%% Abort stop curve in time vs distance
aStop = decisionSpeed^2 / (2 * stopDistance);   % constant decel magnitude
tStopEnd = decisionSpeed / aStop;

% Time when decision speed is reached on the OEI curve
timeAtDecision = interp1(SpeedsAeoOei, timeAeoOei, intersectionSpeed, 'linear');

% Brake time measured from the decision point
tBrake = linspace(0, tStopEnd, 100);

% Shifted time and distance histories
timeStop = timeAtDecision + tBrake;
distanceStop = intersectionDistance + decisionSpeed * tBrake - 0.5 * aStop * tBrake.^2;

%% Plot distance Vs. Velocity
figure(Theme='light')
hold on
%plot(stopDistancePlot, stopSpeedPlot, 'LineWidth', 1.5)
plot(distanceAeoOei, decisionSpeedPlot)
plot(distanceAeo, speeds)
plot(distanceOei - 200, oeiSpeedArray)
%plot(intersectionDistance, intersectionSpeed, 'ro', 'MarkerFaceColor', 'r')

xlabel('Distance (ft)')
ylabel('Speed (ft/s)')

legend('Decision speed', 'Normal takeoff', 'Engine failure', 'Intersection', ...
    'Location', 'best')
title('Takeoff Performance and Abort Analysis')
grid on
hold off

%% Plot time vs distance
figure(Theme="light")
hold on
plot(timeAeo, distanceAeo, 'LineWidth', 1.5)
plot(timeAeoOei, distanceAeoOei, 'LineWidth', 1.5)
plot(timeStop-1.2, distanceStop, 'LineWidth', 1.5)

xlabel('Time (s)')
ylabel('Distance (ft)')
legend('AEO', 'OEI', 'Abort stop', 'Location', 'best')
title('Time vs Distance')
grid on
hold off

%% OEI climb rate vs weight
weightVec = linspace(120000, takeoffWeight, 60);   % lb
climbSpeed = decisionSpeed;                        % ft/s, choose your climb condition

thrustOei = 3 * thrustPerEngine;

dragClimb = interp1([0 takeoffSpeed], ...
    [0 dragAtTakeoff + engineDragAtTakeoff], climbSpeed, 'linear');

rocOei = (thrustOei - dragClimb) .* climbSpeed ./ weightVec;   % ft/s
rocOeiFpm = 60 * rocOei;                                        % ft/min

figure(Theme="light")
plot(weightVec, rocOeiFpm, 'LineWidth', 1.5)
grid on
xlabel('Weight (lb)')
ylabel('OEI Climb Rate (ft/min)')
title('OEI Climb Rate vs Weight')
