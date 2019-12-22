d1=dir('Datasets/board1/*.mat');
d2=dir('Datasets/board1/*.jpg');
for i=1:length(d1),
    imglistdepth{i} = ['Datasets/board1/' d1(i).name];
    imglistrgb{i} = ['Datasets/board1/' d2(i).name];
end

clear d1 d2 i
 
