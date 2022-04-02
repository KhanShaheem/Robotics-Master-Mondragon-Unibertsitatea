%*************************************************************************%
%                                                                         %
%       Date:               24/11/2021                                    %
%       Author:             Team 3                                        %
%       Subject:            Artificial Vision                             %
%       Main description:	Mini-project                                  %
%       Version:            1.0                                           %
%                                                                         %
%*************************************************************************%
clc; 
clear all;
close all;

%% Component folders loading
folder = fileparts(which(mfilename));                                       % Determine where your m-file's folder is.
addpath(genpath(folder));                                                   % Add that folder plus all subfolders to the path.

%% Images loading
n = size(dir([strcat(folder,'.\img\triangle\') '/*.bmp']),1);               % Number of images of the triangle metal part.
s = fun_loadImg (n, '.\img\triangle\', '.bmp');

%% Rectification

load('rectification.mat')
s_rect = fun_undistWarp (n, s, cameraParams, HRect, 'full', 0);

%% Hough transform
% Lines detection
[s_edge_line, s_BW_open, lines, lines_inf, lines_fin] = fun_findLines (n, s_rect);

% Circles detection (unknown radius)
[s_edge, centres, radii] = fun_findCircles (n, s_rect);

%% Distances calculation
error_radii = 1.2;  % [pixels] systematic error compensation in radius measurement
[W1,W2,W3,L1,L2,D1] = fun_calcDistances (n, lines_fin, centres, radii, error_radii, pix2mm_rect);

%% Statistics parameters
[accuracy, repeatability] = fun_calcStatistics (W1,W2,W3,L1,L2,D1);

%% Results visualization
fun_plotResults (n, s_rect, s_BW_open, s_edge, lines, lines_inf, lines_fin, centres, radii, error_radii, accuracy, repeatability);
