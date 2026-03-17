clc; clear; close all;

Ae = 0.5; % Area of the exit of the nozzle
At = 0.25; % Area of the throat of the nozzle
P0_t = 10^5; % total pressure at the throat or in other words toal pressure before the shock
Pe = 0.6 * 10^5; % Static Pressure at the exit of the nozzle

%% Shock Variable area to solve for location of the shock
As = 0.4391; % Area of the shock

% Iterative process to find the shock location inside the nozzle with the
% given variables

AreaRatio = As / At;
y = 1.4;

syms M

MachArray = vpasolve(   ( 1 / (M ^ 2) ) * ( 2/(y+1) * ( 1 + (y-1)/2 * M ^ 2 ) ) ^ ( (y+1)/(y-1) ) == (AreaRatio ^ 2), M   );
MachShock = MachArray(MachArray(imag(MachArray) == 0) > 0);


%% Case 1: Solving for Mach after shock using given pressure values with iterative total pressure
R = 287.05; % ideal gas constant
Cp = 1004.5; % Something something temperature coefficient

% Difference in entropy
s2_s1 = Cp * log(   ( 1 + 2*y/(y+1) * (MachShock(2) ^ 2 - 1) ) * ( 2 + (y - 1) * MachShock(2) ^ 2 ) / ( (y + 1) * MachShock(2) ^ 2 )  ) - R * log( 1 + 2*y/(y+1) * (MachShock(2)^2 - 1));
P02_P01 = exp( -s2_s1 / R); % Total pressure ratio between the shock

P0AfterShock = P02_P01 * P0_t; % Finding the total pressure after the shock

P0_P = P0AfterShock / Pe; % Pressure ratio between iterative total pressure and given static pressure

MachArray = solve(  ( 1 + (y-1) / 2 * M ^ 2 ) ^ (y/(y-1)) == P0_P , M  );
MachAtExit1 = MachArray(MachArray(imag(MachArray) == 0) > 0) % Solving for mach after shock at the exit for case 1


%% Case 2: solving for Mach after shock using imperical and isotrophic methods

MachAfterShock = sqrt(   (  1 + (y-1) / 2 * MachShock(2) ^ 2  ) / (  y * MachShock(2) ^ 2 - (y-1) / 2  )   );
NewAreaRatio = sqrt(( 1 / (MachAfterShock ^ 2) ) * ( 2/(y+1) * ( 1 + (y-1)/2 * MachAfterShock ^ 2 ) ) ^ ( (y+1)/(y-1) ));

AreaRatioAtExit = NewAreaRatio * Ae / As;
 
MachArray = vpasolve(   ( 1 / (M ^ 2) ) * ( 2/(y+1) * ( 1 + (y-1)/2 * M ^ 2 ) ) ^ ( (y+1)/(y-1) ) == (AreaRatioAtExit ^ 2), M   );
MachAtExit2 = MachArray(MachArray(imag(MachArray) == 0) > 0);
MachAtExit2 = MachAtExit2(1) 
