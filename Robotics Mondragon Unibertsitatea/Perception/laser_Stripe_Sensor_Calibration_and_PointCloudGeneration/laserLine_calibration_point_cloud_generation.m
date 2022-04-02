% Perception
% Project-1
% Laser Stripe Sensor Calibration and 3D Point Clound Generation

% Setup path
close all; clearvars; clc;

%addpath('laser_images_360'); 
folder = fileparts(which(mfilename));                           
addpath(genpath(folder));


    
for rot = 1:360
%% Load Data
    im = imread(sprintf('%d.jpg',rot));
    %im = imread('1.jpg');
    imRot = imrotate(im, 90);
    imgr = rgb2gray(imRot);

    %% Line Fitting using Max (pixel accuracy)
    count = 1;
    [nRows, nColumns] = size(imgr);
    %figure; imshow(imRot);
   
    for i = 1:nRows
        imgrX = imgr(i, :);
        T = graythresh(imgrX); % Noise removal
        if (T > 0.1)
            [~, xMax] = max(imgrX);
            x(count) = xMax;
            y(count) = i;
            count = count + 1;
            %hold on; plot(xMax, i, 'g.');
        end
    end 

    %% Line Crossing

    % Manual Limits
    xLimits = [639, 568, 562, 544, 534, 515, 509, 488, 483,  465,  459, 329, 328]; % 329, 328];
    yLimits = [389, 453, 536, 555, 737, 760, 930, 954, 1113, 1136, 1284, 1459, 1491]; % 1462, 1491];

    %figure; imshow(imRot); hold on;

    for i = 1:length(xLimits)-1
        
        diff_x1 = abs(x - xLimits(i));
        diff_y1 = abs(y - yLimits(i));
        diff_x2 = abs(x - xLimits(i + 1));
        diff_y2 = abs(y - yLimits(i + 1));

        stepInit = (diff_x1 < 7) & (diff_y1 < 7);
        stepEnd = (diff_x2 < 7) & (diff_y2 < 7);

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

        %plot(xLine(i, :), yLine(i, :), 'r.');

    end

    %% Intersection of lines

    %figure, imshow(imRot); hold on;
    for i = 1:length(xLimits)-2
        [x_corner(i), y_corner(i)] = polyxpoly(xLine(i, :), yLine(i, :), xLine(i+1, :) , yLine(i+1, :));
        %plot(x_corner(i), y_corner(i), 'b.')
    end

    %% Homography calculation
    Pwth = [222, 200, 200, 150, 150, 100, 100, 50, 50, 0;
        -23, -23, -36, -36, -49, -49, -62, -62, -75, -75];
    Pi = [x_corner(1:10); y_corner(1:10)];
    [h, h1, L] = calculateHomo(Pi, Pwth);
    H1 = reshape(h1, 3, 3)';
    H1inv = inv(H1);

    % Project points from image plane to world
    Pi = [Pi; ones(1, size(Pi, 2))];                
    Pw = H1inv * Pi;
    Pw = Pw ./ Pw(3, :);

    %% Generate Point Cloud
    Npoints = 100;     
    CloudPtsTot = [];
    %figure, imshow(im_rot);
    hold on;    
    for i = 1 : length(x_corner)-1
        x_line(i, :) = linspace(x_corner(i), x_corner(i+1), Npoints);
        y_line(i, :) = linspace(y_corner(i), y_corner(i+1), Npoints);

        %plot(x_line(i, :), y_line(i, :),'b');
    end
    % Generate point cloud
    x_lineT = x_line';
    y_lineT = y_line'; 
    
    for i = 1:length(x_line) * 9  
        CloudPts(i, :) = [x_lineT(i), y_lineT(i), 1];
    end 
    CloudPtsT = CloudPts';
    CloudPts_mm = H1inv * CloudPtsT;
    CloudPts_mm = CloudPts_mm ./ CloudPts_mm(3, :);
    CloudPtsTot = [CloudPtsTot, CloudPts_mm ];
    % Write cloud point
    writeCloud('PointCloud.txt', CloudPtsTot);
    %ptCloud = pointCloud(CloudPtsTot, 'Location');
    %pcshow(ptCloud);

end



%% Functions
% Homography Function
function [h, h1, L] = calculateHomo(Pi, Pw)
    nPoints = size(Pi, 2);
    A = zeros(nPoints*2, 9);

    for i = 1:nPoints
      A(i*2 - 1, 1) = -Pw(1, i);
      A(i*2 - 1, 2) = -Pw(2, i);
      A(i*2 - 1, 3) = -1;
      A(i*2 - 1, 7) = Pw(1, i) * Pi(1, i);
      A(i*2 - 1, 8) = Pw(2,i) * Pi(1,i);
      A(i*2 - 1, 9) = Pi(1,i);

      A(i*2, 4) = -Pw(1, i);
      A(i*2, 5) = -Pw(2, i);
      A(i*2, 6) = -1;
      A(i*2, 7) = Pw(1, i) * Pi(2, i);
      A(i*2, 8) = Pw(2, i) * Pi(2, i);
      A(i*2, 9) = Pi(2, i);
    end

    % Solution with eigenvalues
    [V, ~] = eig(A'* A);
    L = eig(A' * A);
    h = V(:, 1);

    % Solution with singular values
    [~, ~, V1] = svd(A);
    h1 = V1(:, end);

    return;
end

% PointCloud Function
function cloud = writeCloud(fileName, cloudPts)
file = fopen(fileName, 'w');
 
for i = 1:size(cloudPts, 2)
  PtIJ = cloudPts(:, i);
  fprintf(file, '%6.6f %6.6f %6.6f\n', PtIJ(1), PtIJ(2), PtIJ(3)); 
end
fclose(file);
end

