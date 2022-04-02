clc, close all, clear all;

%% Image Processing
addpath('laser_images_360');

folder = fileparts(which(mfilename));                                       % Determine where your m-file's folder is.
addpath(genpath(folder));
% Read images
for k = 1:360
    im = imread(sprintf('%d.jpg',k));
    % im = imread('6.jpg');
    imgr = rgb2gray(im);
    % Rotate image
    imgr_rot = imrotate(imgr, 90);
    im_rot = imrotate(im, 90);
    % imshow(im)
    % figure
    % plot(imgr(:, 1000));

    % Calculate homography
    Pwth = [222, 200, 200, 150, 150, 100, 100, 50, 50, 0;
            -23, -23, -36, -36, -49, -49, -62, -62, -75, -75];

%% Find maximums
    [rows, cols] = size(imgr_rot);
    %figure
    %imshow(im_rot);
    j = 1;
    for i = 1:rows
        imgrX = imgr_rot(i, :);
        T = graythresh(imgrX);
        if (T > 0.1)
            [~, xMax] = max(imgrX);
            x_xMax(j) = xMax;
            y_xMax(j) = i; 
            hold on
            %plot(xMax,i,'g.')
            j = j+1;
        end
    end
%% CoG method
%     figure
%     imshow(im_rot);
%     clc
%     [rows, cols] = size(imgr_rot);
%     for i = 1 : rows
%         y = imgr_rot(:, i);            
%         x = imgr_rot(i, :);
%         [~, yMax] = max(y);          % image row maximum (y value)
%         CoG_yMax(i) = yMax;
%         intervalCoG = [yMax - 20: yMax + 20];
%         x0CoG = sum(y(intervalCoG).*imgr_rot(intervalCoG)) / sum(y(intervalCoG)); 
%     end
%% FIR
%     [rows, cols] = size(imgr_rot);
%     for i = 1:rows
%     y = imgr_rot(:, i);            
%     x = imgr_rot(i, :);
%     mask = [-1 -2 -3 0 3 2 1];
%     yConv = conv(y, mask, 'same');
%     figure
%     plot(x, yConv);
%     % We look at the zero crossing of this curve. 
%     % First fitting a line a * x + b * y + c = 0
%     intervalZCross = [xMax - 4: xMax + 4];
%     A = [x(intervalZCross)', yConv(intervalZCross)', ones(size(x(intervalZCross)'))];
%     [~, ~, V] = svd(A);
%     Recta = V(:, end);
% 
%     hold on
%     plot(x(intervalZCross), (- Recta(1) * x(intervalZCross) - Recta(3)) / Recta(2), 'r');
%     x0ZCross = - Recta(3) / Recta(1)
%     end
%% Line Fitting
    % Define line segment limits
    xLine_limits = [639, 568, 562, 544, 534, 515, 509, 488, 483,  465,  459, 329, 328]; % 329, 328];
    yLine_limits = [389, 453, 536, 555, 737, 760, 930, 954, 1113, 1136, 1284, 1459, 1491]; % 1462, 1491];


    % Build coordinates array (x1,x2) and (y1,y2)
    %figure, imshow(im_rot);
    %hold on;
    for i = 1:length(xLine_limits)-2
%         conditionInit = (x_xMax == xLine_limits(i)) & (y_xMax == yLine_limits(i));
%         conditionEnd = (x_xMax == xLine_limits(i + 1)) & (y_xMax == yLine_limits(i + 1));
        PixErr = 5;
        % For images 255, 302 and 306        
        if k == 255 || k == 302 || k == 306
            PixErr = 7;
        end
        diff_x1 = abs(x_xMax - xLine_limits(i));
        diff_y1 = abs(y_xMax - yLine_limits(i));
        diff_x2 = abs(x_xMax - xLine_limits(i + 1));
        diff_y2 = abs(y_xMax - yLine_limits(i + 1));
        conditionInit = (diff_x1 < PixErr) & (diff_y1 < PixErr);
        conditionEnd = (diff_x2 < PixErr) & (diff_y2 < PixErr);
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
        %plot(x_f(i,:),y_f(i,:),'r.')
    end

%% Intersection of lines
    %figure, imshow(im_rot);
    hold on;
    for i = 1:length(xLine_limits)-3
        [x_corner(i), y_corner(i)] = polyxpoly(x_f(i, :), y_f(i, :), x_f(i+1, :) , y_f(i+1, :));
        %plot(x_corner(i),y_corner(i),'b.')
    end
    %% Homography calculation
    Pi = [x_corner; y_corner];
    [h, h1, L] = calculateHomo(Pi, Pwth);
    H1 = reshape(h1, 3, 3)';
    H1inv = inv(H1);

    % Project points from image plane to world
    Pi = [Pi; ones(1, size(Pi, 2))];                
    Pw = H1inv * Pi;
    Pw = Pw ./ Pw(3, :);
%% Generate Point Cloud
    Npoints = 100;     
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
        % CloudPts(k,:) = [x_line, y_line];
        CloudPts(i, :) = [x_lineT(i), y_lineT(i), 1];
    end
    CloudPtsT = CloudPts';
    CloudPts_mm = H1inv * CloudPtsT;
    CloudPts_mm = CloudPts_mm ./ CloudPts_mm(3, :);
    CloudPts_Tot(k, :) = CloudPts_mm;
    % Write cloud point
    writeCloud('PointCloud.txt', CloudPts_Tot);
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


