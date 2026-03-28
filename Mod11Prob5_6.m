% Author: Dage Lee
% Organization: SDSU
% Class: AE 310 Aerospace Structures
% Homework: Module 11 Homework problem 5.6

clc; clear; close all;

% Given Variables
sigma_yp = 290*10^6;
L = 1.5;
t = 20e-3;
c = 60e-3;
b = 80e-3;
n = 1.2;
sigma_safety = sigma_yp / 1.2;

z = (0.5*(b*t + c^2) + c * t) / (b + t); % Distance component of z of an arbitary point on the triangle

dz1 = z - t/2; % distance component z of centroid of shape 1 from the overall centroid
dz2 = c/2 - (z-t); % distance compoenent z of centroid of shape 2 from the overall centroid

dy1 = 0; % distance component y of centroid of shape 1 from the overall centroid
dy2 = 0; % distance component y of centroid of shape 2 from the overall centroid

A1 = b * t; % Area of shape 1
A2 = c * t; % Area of shape 2

Iy1 = t^3 * b / 12; % Second area moment component y for shape 1
Iy2 = c^3 * t / 12; % Second area moment component y for shape 2

Iz1 = b^3 * t / 12; % Second area moment component z for shape 1
Iz2 = t^3 * c / 12; % Second area moment component z for shape 2

Iy = (Iy1 + A1 * dz1^2) + (Iy2 + A2 * dz2^2); % Second area moment component z for overall shape
Iz = (Iz1 + A1 * dy1^2) + (Iz2 + A2 * dy2^2); % Second area moment component z for overall shape

d1 = sqrt(z^2 + (b/2)^2); % distance of overall centroid to a point on the edge of the object
d2 = sqrt((c-(z-t))^2 +(t/2)^2); % distance of overall centroid to a point on the edge of the object

My0 = 0; % Since Alhpa is equal to 0 for the first problem it can be simplified
P0 = -sigma_safety * Iz / L / (-b / 2); % simplified form for finding the P maximum

P15 = sigma_safety / (sind(15) * L * z / Iy - cosd(15) * L * (-b/2) / Iz); % finding p maximum based on the angle of alpha

P30 = sigma_safety / (sind(30) * L * z / Iy - cosd(30) * L * (-b/2) / Iz);