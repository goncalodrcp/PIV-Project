d1=dir('Datasets/labpiv/*.mat');
d2=dir('Datasets/labpiv/*.png');
for i=1:length(d1),
    imglistdepth{i} = ['Datasets/labpiv/' d1(i).name];
    imglistrgb{i} = ['Datasets/labpiv/' d2(i).name];
end

clear d1 d2 i
 
