d1=dir('imgs/*.mat');
d2=dir('imgs/*.jpg');
for i=1:length(d1),
    data(i).depth = ['Datasets/imgs/' d1(i).name];
    data(i).rgb = ['Datasets/imgs/' d2(i).name];
    %im1=imread(['depth/' d1(i).name]);
    %im2=imread(['rgb/' d2(i).name]);
    %depth_array=double(im1)/5;
    %save(['board1/' sprintf('img_%10.10d.mat',i)],'depth_array');
    %imwrite(im2,['f/' sprintf('img_%10.10d.png',i)]);
end
 
clear d1 d2 i
 