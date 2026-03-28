% Author: Dage Lee
% Organization: SDSU
% Class: AE 310 Aerospace Structures
% Homework: Module 11 Homework problem 5.9

clc; clear; close all;

% Given Variables
a = 80e-3;
b = 90e-3;
alpha = 20;
M0 = 3e3;

% Second Area Moment of a triangle
Iz = a * b^3 / 36;
Iy = b * a^3 / 48;

% Components of the moment 0
My = M0 * cosd(alpha);
Mz = M0 * sind(alpha);

% Angle of the neutral Axis
phi = atan(Iz / Iy * My / Mz) * 180 / pi()

% Distance of point D from centroid of the triangle
zd = 2*b / 3;
yd = 0;

% Distance of point A from centroid of the triangle
ya = a / 2;
za = -b / 3;

% Stress at points D and A on the edge of the triangle
sigma_D = maxstresscalc(My, Iy, Mz, Iz, zd, yd)
sigma_A = maxstresscalc(My, Iy, Mz, Iz, za, ya)


% Function to simplify calculations for max stress
function sigma_max = maxstresscalc(My, Iy, Mz, Iz, z, y)
sigma_max = (My / Iy * z - Mz / Iz * y) / 1e6;
end
