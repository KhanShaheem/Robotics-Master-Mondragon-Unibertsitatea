function [W1,W2,W3,L1,L2,D1] = fun_calcDistances (n, lines_fin, centres, radii, error_radii, pix2mm)

% Preallocation
W1 = zeros(1,n);
W2 = zeros(1,n);
W3 = zeros(1,n);
L1 = zeros(1,n);
L2 = zeros(1,n);
L3 = zeros(1,n);
D1 = zeros(1,n);

for i = 1 : n
    % Define file name
    if i < 10
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    
    %% Line measurements
    % Calculate the corner coordinates difference in X and Y
    xdiff_12 = abs (lines_fin.(file_name).xline_12(2) - lines_fin.(file_name).xline_12(1));
    ydiff_12 = abs (lines_fin.(file_name).yline_12(2) - lines_fin.(file_name).yline_12(1));
    xdiff_13 = abs (lines_fin.(file_name).xline_13(2) - lines_fin.(file_name).xline_13(1));
    ydiff_13 = abs (lines_fin.(file_name).yline_13(2) - lines_fin.(file_name).yline_13(1));
    xdiff_23 = abs (lines_fin.(file_name).xline_23(2) - lines_fin.(file_name).xline_23(1));
    ydiff_23 = abs (lines_fin.(file_name).yline_23(2) - lines_fin.(file_name).yline_23(1));
    
    % Calculate the triangle sides distance in mm
    width_12 = sqrt(xdiff_12^2 + ydiff_12^2) * pix2mm;
    width_13 = sqrt(xdiff_13^2 + ydiff_13^2) * pix2mm;
    width_23 = sqrt(xdiff_23^2 + ydiff_23^2) * pix2mm;
    
    % Assign the triangle sides distance in mm
    width_array = [width_12, width_13, width_23];       % build array with the 3 distances 
    W3(i) = max(width_array);                           % highest distance correspond to the hypotenuse
    width_array (width_array==W3(i)) = [];              % delete the hypotenuse distance
    W1(i) = width_array(1);                             % ASSUMPTION. Not critical because the cathetus are equal
    W2(i) = width_array(2);                             % ASSUMPTION. Not critical because the cathetus are equal
    
    % Calculate the circle centres difference in XY
    xy_diff_12 = abs (centres.(file_name)(2,:) - centres.(file_name)(1,:));
    xy_diff_13 = abs (centres.(file_name)(3,:) - centres.(file_name)(1,:));
    xy_diff_23 = abs (centres.(file_name)(3,:) - centres.(file_name)(2,:));
    
    %Calculate the distance between circles in mm
    length_12 = sqrt(sum(xy_diff_12)^2) * pix2mm;
    length_13 = sqrt(sum(xy_diff_13)^2) * pix2mm;
    length_23 = sqrt(sum(xy_diff_23)^2) * pix2mm;
    
    % Assign the distance between circles in mm
    length_array = [length_12, length_13, length_23]; 
    L3(i) = max(length_array);
    length_array (length_array==L3(i)) = [];
    L2(i) = max(length_array);
    length_array (length_array==L2(i)) = [];
    L1(i) = length_array;
    
    %% Circle measurement
    c = 1;                                              % ASSUMPTION. Not critical because the circles are equal
    R1 = (radii.(file_name)(c)-error_radii) * pix2mm;
    D1(i) = 2*R1;
end