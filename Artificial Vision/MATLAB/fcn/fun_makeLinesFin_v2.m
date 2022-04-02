function lines_fin = fun_makeLinesFin_v2 (xline_inf, yline_inf)

%% Intersection finding
% Intersection of line 1 and 2
[x_corner, y_corner] = polyxpoly([xline_inf.line01(1) xline_inf.line01(2)], [yline_inf.line01(1) yline_inf.line01(2)], ...
                                 [xline_inf.line02(1) xline_inf.line02(2)], [yline_inf.line02(1) yline_inf.line02(2)]);

% Intersection of line 2 and 3
[x_corner2, y_corner2] = polyxpoly([xline_inf.line02(1) xline_inf.line02(2)], [yline_inf.line02(1) yline_inf.line02(2)], ...
                                   [xline_inf.line03(1) xline_inf.line03(2)], [yline_inf.line03(1) yline_inf.line03(2)]);

% Intersection of line 1 and 3
[x_corner3, y_corner3] = polyxpoly([xline_inf.line01(1) xline_inf.line01(2)], [yline_inf.line01(1) yline_inf.line01(2)], ...
                                   [xline_inf.line03(1) xline_inf.line03(2)], [yline_inf.line03(1) yline_inf.line03(2)]);
                               
                               
%% Finite lines
lines_fin.xline_12 = [x_corner x_corner2];
lines_fin.yline_12 = [y_corner y_corner2];

lines_fin.xline_13 = [x_corner x_corner3];
lines_fin.yline_13 = [y_corner y_corner3];

lines_fin.xline_23 = [x_corner2 x_corner3];
lines_fin.yline_23 = [y_corner2 y_corner3];