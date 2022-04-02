function [xline_inf, yline_inf, slope] = fun_makeInfLine (s_edge, lines, file_name, j)

[~, columns] = size(s_edge.(file_name));

xy = [lines.(file_name)(j).point1; lines.(file_name)(j).point2];
% Get the equation of the line
x1 = xy(1,1);
y1 = xy(1,2);
x2 = xy(2,1);
y2 = xy(2,2);
slope = (y2-y1)/(x2-x1);
xLeft = 1; % x is on the left edge
yLeft = slope * (xLeft - x1) + y1;
xRight = columns; % x is on the right edge.
yRight = slope * (xRight - x1) + y1;

xline_inf = [xLeft, xRight];
yline_inf = [yLeft, yRight];

%-------------------------- NOT WORKING PATCH ----------------------------%
% % Avoid infinite slope when X values are equal
% if (lines.(file_name)(j).point1(1) == lines.(file_name)(j).point2(1))
%     yline_inf = [lines.(file_name)(j).point1(2) lines.(file_name)(j).point1(2)];
%     fprintf('Line %d of %s contains an infinite slope (equal X values)\n\n',j,file_name);
% end
% % Avoid 0 slope when Y values are equal
% if (lines.(file_name)(j).point1(2) == lines.(file_name)(j).point2(2))
%     xline_inf = [lines.(file_name)(j).point1(1) lines.(file_name)(j).point1(1)];
%     fprintf('Line %d of %s contains a 0 slope (equal Y values)\n\n',j,file_name);
% end
%-------------------------------------------------------------------------%
