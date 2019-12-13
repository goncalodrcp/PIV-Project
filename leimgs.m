d1=dir('board1/*.mat');
d2=dir('board1/*.jpg');
for i=1:length(d1),
    data(i).depth = ['board1/' d1(i).name];
    data(i).rgb = ['board1/' d2(i).name];
end
