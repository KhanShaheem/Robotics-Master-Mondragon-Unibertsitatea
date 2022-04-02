clc, close all, clear all;

%% Image Processing
addpath('laser_images_360');

folder = fileparts(which(mfilename));                                       % Determine where your m-file's folder is.
addpath(genpath(folder));                                                   % Add that folder plus all subfolders to the path.
% Read image
im = imread('5.jpg');
imgr = rgb2gray(im);
% Rotate image
imgr_rot = imrotate(imgr, 90);
im_rot = imrotate(im, 90);
% imshow(im)
% figure
% plot(imgr(:, 1000));

%% Find maximums
[rows, cols] = size(imgr_rot);
figure
imshow(im_rot);
j = 1;
for i = 1:rows
    imgrX = imgr_rot(i, :);
    T = graythresh(imgrX);
    if (T > 0.1)
        [~, xMax] = max(imgrX);
        x_xMax(j) = xMax;
        y_xMax(j) = i; 
        hold on
        plot(xMax,i,'g.')
        j = j+1;
    end
end
%% CoG method
% for i = 1 : rows
%     y = imgr_rot(i, :);             % image row (x value)
%     [~, yMax(i)] = max(y);          % image row maximum (y value)
%     intervalCoG = [yMax - 20: yMax + 20];
%     x0CoG = sum(yMax(:,intervalCoG).*im_rot(intervalCoG,:)) / sum(yMax(:,intervalCoG)); 
% end
%% FIR


%% Line Fitting
% Define line segment limits
xLine_limits = [639, 568, 562, 544, 534, 515, 509, 488, 483,  465,  459, 329, 328]; % 329, 328];
yLine_limits = [389, 453, 536, 555, 737, 760, 930, 954, 1113, 1136, 1284, 1459, 1491]; % 1462, 1491];


% Build coordinates array (x1,x2) and (y1,y2)
figure, imshow(im_rot);
hold on;
for i = 1:length(xLine_limits)-1
%     conditionInit = (x_xMax == xLine_limits(i)) & (y_xMax == yLine_limits(i));
%     conditionEnd = (x_xMax == xLine_limits(i + 1)) & (y_xMax == yLine_limits(i + 1));
    diff_x1 = abs(x_xMax - xLine_limits(i));
    diff_y1 = abs(y_xMax - yLine_limits(i));
    diff_x2 = abs(x_xMax - xLine_limits(i + 1));
    diff_y2 = abs(y_xMax - yLine_limits(i + 1));
    conditionInit = (diff_x1 < 3) & (diff_y1 < 3);
    conditionEnd = (diff_x2 < 3) & (diff_y2 < 3);
    idxInit = find(conditionInit, 01, 'first');
    idxEnd = find(conditionEnd, 01, 'first');
    xMax_array = x_xMax(idxInit : idxEnd);
    y_xMax_array = y_xMax(idxInit : idxEnd);
    % Find fitting coefficients
    a(i, :) = polyfit(xMax_array, y_xMax_array, 1);
    % Fitting ecuation
    if xMax_array(1) > xMax_array(end)
        x_f(i, :) = linspace(xMax_array(1)+10, xMax_array(end)-10, 1000);
    else 
        x_f(i, :) = linspace(xMax_array(1)-10, xMax_array(end)+10, 1000);
    end
    y_f(i, :) = a(i, 1) * x_f(i, :) + a(i, 2);
    plot(x_f(i,:),y_f(i,:),'r.')
end

%% Intersection of lines
figure, imshow(im_rot);
hold on;
for i = 1:length(xLine_limits)
    [x_corner(i), y_corner(i)] = polyxpoly(x_f(i, :), y_f(i, :), x_f(i+1, :) , y_f(i+1, :));
    plot(x_corner(i,:),y_corner(i,:),'r.')
end

% %% Homography calculation
% Pw = [220, 200, 200, 150, 150, 100, 100, 50, 50, 0;...
%   -23, -23, -36, -36, -49, -49, -62, -62, -75, 0];
% Pi = [x_corner(1:10); y_corner(1:10)];
% 
% [h, h1, Pi, L] = calculateHomoProjecting(Pi, Pw);
% 
% %% Generate Point Cloud