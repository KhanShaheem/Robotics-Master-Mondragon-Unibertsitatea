% Perception
% Project-1
% Laser Stripe Sensor Calibration and 3D Point Clound Generation

close all; clear all; clc;

%% Load Data

addpath('laser_images_360'); 
folder = fileparts(which(mfilename));                           
addpath(genpath(folder));

%Pw = [222, 200, 200, 150, 150, 100, 100, 50, 50;...
%      23,  23,  36,  36,  49,  49,  62,  62, 75];

im = imread('1.jpg');

% Image Rotations
imRot = imrotate(im, 90);
imgr = rgb2gray(imRot);

%% Line Fitting using Max (pixel accuracy)

[nRows, nColumns] = size(imgr);
figure; imshow(imRot);

count = 1;

for i = 1:nRows
    
    imgrX = imRot(i, :);
    T = graythresh(imgrX); % Noise removal
    if (T > 0.1)
        [~, xMax] = max(imgrX);
        x(count) = xMax;
        y(count) = i;
        count = count + 1;
        hold on; plot(xMax, i, 'g.');
    end
    
end 
    
%% Line Crossing

% Manual Limits
xLimits = [639, 568, 562, 544, 534, 515, 509, 488, 483,  465,  459, 329, 328]; % 329, 328];
yLimits = [389, 453, 536, 555, 737, 760, 930, 954, 1113, 1136, 1284, 1459, 1491]; % 1462, 1491];

figure; imshow(imRot); hold on;

for i = 1:length(xLimits)-1
    
    %stepInit = (x == xLimits(i)) & (y == yLimits(i));
    % Error is here
    %stepEnd = (x == xLimits(i + 1)) & (y == yLimits(i + 1));
    
    diff_x1 = abs(x - xLimits(i));
    diff_y1 = abs(y - yLimits(i));
    diff_x2 = abs(x - xLimits(i + 1));
    diff_y2 = abs(y - yLimits(i + 1));
    
    stepInit = (diff_x1 < 3) & (diff_y1 < 3);
    stepEnd = (diff_x2 < 3) & (diff_y2 < 3);
    
    idxInit = find(stepInit, 01, 'first');
    idxEnd = find(stepEnd, 01, 'first');
    
    xLimits_calc = x(idxInit : idxEnd);
    yLimits_calc = y(idxInit : idxEnd);
    
    % Fitting coefficients
    coeff(i, :) = polyfit(xLimits_calc, yLimits_calc, 1);
    
    % Line Equation
    
    if xLimits_calc(1) > xLimits_calc(end)
        xLine(i, :) = linspace(xLimits_calc(1) + 10, xLimits_calc(end) - 10, 1000);
    else 
        xLine(i, :) = linspace(xLimits_calc(1) - 10, xLimits_calc(end) + 10, 1000);
    end
    
    yLine(i, :) = (coeff(i, 1) * xLine(i, :)) + coeff(i, 2);
    
    plot(xLine(i, :), yLine(i, :), 'r.');
    
end

%% Intersection of lines

figure, imshow(imRot); hold on;

for i = 1:length(xLimits)
    [x_corner, y_corner] = polyxpoly(xLine(i, :), yLine(i, :), xLine(i+1, :) , yLine(i+1, :));
    plot(x_corner(i,:), y_corner(i,:), 'r.')
end

% %% Homography calculation
% Pw = [220, 200, 200, 150, 150, 100, 100, 50, 50, 0;...
%   -23, -23, -36, -36, -49, -49, -62, -62, -75, 0];
% Pi = [x_corner(1:10); y_corner(1:10)];
% 
% [h, h1, Pi, L] = calculateHomoProjecting(Pi, Pw);
