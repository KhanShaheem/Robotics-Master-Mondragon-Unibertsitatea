function fun_plotResults (n, s_rect, s_BW_open, s_edge, lines, lines_inf, ...
                          lines_fin, centres, radii, error_radii, accuracy, repeatability) 

% Binarization
% figure; imshow(s_BW_open.im01);

for i = 1 : n
    % Define file name
    if (i < 10)
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    
    % Original image
    figure; %subplot(1,2,1);
    imshow(s_rect.(file_name)); hold on;
    
    % Hough circles
    viscircles(centres.(file_name),radii.(file_name)); hold on;
    viscircles(centres.(file_name),radii.(file_name)-error_radii,'LineStyle','--'); hold on;
    
    % Hough lines
    for j = 1 : length(lines.(file_name))
        x_array = [lines.(file_name)(j).point1(1), lines.(file_name)(j).point2(1)];
        y_array = [lines.(file_name)(j).point1(2), lines.(file_name)(j).point2(2)];
        plot(x_array,y_array,'*');
    end
    
    % Infinite lines
    for j = 1 : length(lines.(file_name))
        line_name = strcat('line0',num2str(j));
        plot(lines_inf.xline_inf.(file_name).(line_name),...
             lines_inf.yline_inf.(file_name).(line_name),'--');
    end
    
    % Finite lines
    plot(lines_fin.(file_name).xline_12, lines_fin.(file_name).yline_12,'LineWidth',2); hold on;
    plot(lines_fin.(file_name).xline_13, lines_fin.(file_name).yline_13,'LineWidth',2); hold on;
    plot(lines_fin.(file_name).xline_23, lines_fin.(file_name).yline_23,'LineWidth',2);
    
    % Figure details
    title_content = sprintf('Rectified image (%s/%s)',num2str(i),num2str(n));
    title(title_content);
    %legend({'Hough lines','Infinite lines','Finite lines'},'FontSize',16);
    
    % Edge detection
%     subplot(1,2,2);
%     imshow(s_edge.(file_name));
%     title_content = sprintf('Automatic detection of all edges (%s/%s)',num2str(i),num2str(n));
%     title(title_content);
end

% Accuracy and repeatability
fprintf('Accuracy [%%] =\n\n'); disp(accuracy);
fprintf('Repeatability [%%] =\n\n'); disp(repeatability);
