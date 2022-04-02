function s = fun_loadImg (Nimg, path, file_type)

for i = 1 : Nimg
	if (i < 10)
        file_name = strcat('im0',num2str(i));
    else
        file_name = strcat('im',num2str(i));
    end
    s.(file_name) = imread(strcat(path,file_name,file_type));
end