function s_rect = fun_undistWarp (n, s, cameraParams, HRect, sizeOutImg, showImgComp)

for i = 1 : n
    % Define file name
    if (i < 10)
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    
    % Undistort image
    s_undist.(file_name) = undistortImage(s.(file_name), cameraParams);
    
    % Warp image
    switch (sizeOutImg)
        case 'full'
            [s_rect.(file_name), RB] = imwarp(s_undist.(file_name), projective2d((HRect')),'FillValues', 0);
    
        case 'same'
            fixedView = imref2d(size(s_undist.(file_name)));
            [s_rect.(file_name), RB] = imwarp(s_undist.(file_name), projective2d((HRect')),'FillValues', 0,'OutputView',fixedView);
    end
    
    % Show image comparison
    if (showImgComp)
        figure; imshowpair(s.(file_name),s_rect.(file_name),'montage');
        title_content = sprintf('Input image vs. Rectified image (%s/%s)',num2str(i),num2str(n));
        title(title_content);
    end
end