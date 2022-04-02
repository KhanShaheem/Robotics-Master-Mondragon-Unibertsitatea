% Evaluate the errors of the homography taking into account the number of points in the homography, 
% errors in the image, and errors in the world points. 
% Make a table for the number of points in the homography = 4, and 8 points 
% (blue, and blue and red points in the Figure). 
% Also give random noise to the pixels with a normal distribution, zero centered, 
% and sigma = zero, 0.1, 0.05 and 0.01 pixels. 
% Also give random noise to the laser points with a normal distribution, zero centered, 
% and sigma = zero, 0.005, 0.01, 0.005 and 0.1 mm.

% Do the procedure 100 times. Calculate average error in h

% Calculating Homography
close all; clear all; clc;
f = 12.0 * 1.0e-3;
sx = 5.0 * 1.0e-6; % Pixel Size
sy = sx;
Cx = 1152;
Cy = Cx;
K = [f/sx,0,Cx; 0, f/sy, Cy;0, 0, 1];

alph = 0; 
beta = 90;
gamma = 90;
Rx = [1, 0, 0;0 , cosd(alph), -sind(alph); 0, sind(alph), cosd(alph)];
Ry = [cosd(beta), 0 sind(beta); 0, 1, 0; -sind(beta), 0, cosd(beta)];
Rz = [cosd(gamma), -sind(gamma), 0; sind(gamma), cosd(gamma), 0; 0, 0, 1];
R = Rx * Ry * Rz;

T = [500; 1000; 0];
H = K * [R(:, 1), R(:, 2), T];

%% Evaluating errors in the Homography for a given set of points

% Blue Points
Pw(:, 1) = [200; -36; 1];
Pw(:, 2) = [150; -49; 1];
Pw(:, 3) = [100; -62; 1];
Pw(:, 4) = [50; -75; 1];

% Red points
Pw(:, 5) = [200; -23; 1];
Pw(:, 6) = [150; -36; 1];
Pw(:, 7) = [100; -49; 1];
Pw(:, 8) = [50; -62; 1];

Pw_blue = Pw(1:3, 1:4);

Pi_blue = H * Pw_blue;

for i= 1:4
   Pi_blue(:, i) = Pi_blue(:, i) / Pi_blue(3, i);
end

h = calculateHomo(Pi_blue, Pw_blue);

% Compare with H
H1 = H';
h1 = H1(:);
h1 = h1 / norm(h1);

res_eigen = [h1, h] 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = calculateHomo(Pi, Pw)
    
    nPoints = size(Pi, 2);
    A = zeros(nPoints*2, 9);
    
    for i = 1:nPoints
        
        % To calculate the order of the matrix A
        % Multiply the number of points by 2!
        
        % Odd rows of the matrix A
        A(i*2-1, 1) = -Pw(1, i);
        A(i*2-1, 2) = -Pw(2, i);
        A(i*2-1, 3) = -1;
        A(i*2-1, 7) = Pw(1, i) * Pi(1, i);
        A(i*2-1, 8) = Pw(2, i) * Pi(1, i);
        A(i*2-1, 9) = Pi(1, i);
        
        % Even rows of the matrix A
        A(i*2, 4) = -Pw(1, i);
        A(i*2, 5) = -Pw(2, i);
        A(i*2, 6) = -1;
        A(i*2, 7) = Pw(1, i) * Pi(2, i);
        A(i*2, 8) = Pw(2, i) * Pi(2, i);
        A(i*2, 9) = Pi(2, i);
    end
    
    % Solution with the eigenvalues.
    % V is the eigenvector.
    % D is the eigenvalues.
    % if second argument is not used but the function require an arguement
    % use the symbol '~' for the other arguement.
    [V, D] = eig(A' * A);
    % Eigenvalues order start from smaller then go towards larger values!
    h = V(:, 1);
    
    % Solution with singular values
    [u, S, V1] = svd(A);
    % Singular values order in matlab start from largest to smaller!
    h1 = V1(:, end);
    return;
end