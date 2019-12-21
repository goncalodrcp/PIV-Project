d1=dir('Datasets/imgs-2/*.mat');
d2=dir('Datasets/imgs-2/*.jpg');
for i=1:length(d1),
    imglistdepth{i} = ['Datasets/imgs-2/' d1(i).name];
    imglistrgb{i} = ['Datasets/imgs-2/' d2(i).name];
end

clear d1 d2 i
 
