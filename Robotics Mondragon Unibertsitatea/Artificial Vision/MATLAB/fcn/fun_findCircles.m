function [s_edge, centres, radii] = fun_findCircles (n, s_rect)

% rmin = 10; rmax = 30;
rmin = 20; rmax = 40;
radiusRange = [rmin rmax];

for i = 1 : n
    % Define file name
    if (i < 10)
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    
    % Find circle centre and radius (unkown radius)
    s_edge.(file_name) = edge(s_rect.(file_name),'canny',0.6);
    [centres.(file_name),radii.(file_name)] = imfindcircles(s_edge.(file_name),radiusRange); 
end