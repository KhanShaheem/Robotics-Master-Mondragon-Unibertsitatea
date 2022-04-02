function [s_edge, s_BW_open, lines, lines_inf, lines_fin] = fun_findLines (n, s_rect)

% Final version:
% function [s_edge, inf_lines, intersec] = fun_findLines (n, s_rect)

Ntri = 3;       % Triangle sides

for i = 1 : n
    % Define file name
    if (i < 10)
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    
    % Find edges and lines
    % s_BW_open.(file_name) = bwpropfilt(s_BW.(file_name),'area',100);
    s_BW.(file_name) = (s_rect.(file_name) > 90) & (s_rect.(file_name) < 150);
    SE = strel('disk',40);
    s_BW_open.(file_name) = imclose(s_BW.(file_name),SE);
    s_edge.(file_name) = edge(s_BW_open.(file_name),'canny');
    [A,theta,rho] = hough(s_edge.(file_name));
    peaks = houghpeaks(A,50);
    lines.(file_name) = houghlines(s_edge.(file_name),theta,rho,peaks,'FillGap',100);
    
    % Make lines infinite
    if (length(lines.(file_name)) >= Ntri)
        for j = 1 : length(lines.(file_name))
            line_name = strcat('line0',num2str(j));
            [lines_inf.xline_inf.(file_name).(line_name), ...
             lines_inf.yline_inf.(file_name).(line_name), ...
             lines_inf.slope.(file_name).(line_name)] = ...
             fun_makeInfLine (s_edge, lines, file_name, j);
        end
    else
        fprintf('ERROR! Triangle sides are not properly detected (%d/%d).\n\n',i,n);
    end
    
    % Find intersection
    %accept_error = 5;
    %lines_fin.(file_name) = fun_makeLinesFin(lines_inf.xline_inf.(file_name), lines_inf.yline_inf.(file_name), accept_error);
    lines_fin.(file_name) = fun_makeLinesFin_v2(lines_inf.xline_inf.(file_name), lines_inf.yline_inf.(file_name));
end