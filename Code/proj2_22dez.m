clear

run('/Users/goncalopereira/Documents/MATLAB/vlfeat-0.9.21/toolbox/vl_setup')

cam_params = load('cam_params.mat');

imglistrgb = {'Datasets/room1/rgb_0000.jpg','Datasets/room1/rgb_0001.jpg','Datasets/room1/rgb_0002.jpg','Datasets/room1/rgb_0003.jpg','Datasets/room1/rgb_0004.jpg','Datasets/room1/rgb_0005.jpg','Datasets/room1/rgb_0006.jpg'};
imglistdepth = {'Datasets/room1/depth_0000.mat','Datasets/room1/depth_0001.mat','Datasets/room1/depth_0002.mat','Datasets/room1/depth_0003.mat','Datasets/room1/depth_0004.mat','Datasets/room1/depth_0005.mat','Datasets/room1/depth_0006.mat'};
%imglistrgb = {'Datasets/labpiv/rgb_image_1.png','Datasets/labpiv/rgb_image_2.png','Datasets/labpiv/rgb_image_3.png','Datasets/labpiv/rgb_image_4.png','Datasets/labpiv/rgb_image_5.png','Datasets/labpiv/rgb_image_6.png', ...
    %           'Datasets/labpiv/rgb_image_7.png','Datasets/labpiv/rgb_image_8.png','Datasets/labpiv/rgb_image_9.png','Datasets/labpiv/rgb_image_10.png','Datasets/labpiv/rgb_image_11.png','Datasets/labpiv/rgb_image_12.png', ...
     %          'Datasets/labpiv/rgb_image_12.png','Datasets/labpiv/rgb_image_13.png','Datasets/labpiv/rgb_image_14.png','Datasets/labpiv/rgb_image_15.png','Datasets/labpiv/rgb_image_16.png','Datasets/labpiv/rgb_image_17.png', ...
     %         'Datasets/labpiv/rgb_image_18.png','Datasets/labpiv/rgb_image_19.png','Datasets/labpiv/rgb_image_20.png','Datasets/labpiv/rgb_image_21.png','Datasets/labpiv/rgb_image_22.png','Datasets/labpiv/rgb_image_23.png'};

%imglistdepth = {'Datasets/labpiv/depth_1.mat','Datasets/labpiv/depth_2.mat','Datasets/labpiv/depth_3.mat','Datasets/labpiv/depth_4.mat','Datasets/labpiv/depth_5.mat','Datasets/labpiv/depth_6.mat', ...
         %      'Datasets/labpiv/depth_7.mat','Datasets/labpiv/depth_8.mat','Datasets/labpiv/depth_9.mat','Datasets/labpiv/depth_10.mat','Datasets/labpiv/depth_11.mat','Datasets/labpiv/depth_12.mat', ...
           %    'Datasets/labpiv/depth_12.mat','Datasets/labpiv/depth_13.mat','Datasets/labpiv/depth_14.mat','Datasets/labpiv/depth_15.mat','Datasets/labpiv/depth_16.mat','Datasets/labpiv/depth_17.mat', ...
          %     'Datasets/labpiv/depth_18.mat','Datasets/labpiv/depth_19.mat','Datasets/labpiv/depth_20.mat','Datasets/labpiv/depth_21.mat','Datasets/labpiv/depth_22.mat','Datasets/labpiv/depth_23.mat'};

%imglistrgb = {'Datasets/board1/rgb_0000.jpg','Datasets/board1/rgb_0001.jpg','Datasets/board1/rgb_0002.jpg','Datasets/board1/rgb_0003.jpg','Datasets/board1/rgb_0004.jpg','Datasets/board1/rgb_0005.jpg','Datasets/board1/rgb_0006.jpg','Datasets/board1/rgb_0007.jpg','Datasets/board1/rgb_0008.jpg','Datasets/board1/rgb_0009.jpg','Datasets/board1/rgb_0010.jpg','Datasets/board1/rgb_0011.jpg','Datasets/board1/rgb_0012.jpg','Datasets/board1/rgb_0013.jpg','Datasets/board1/rgb_0014.jpg'};
%imglistdepth = {'Datasets/board1/depth_0000.mat','Datasets/board1/depth_0001.mat','Datasets/board1/depth_0002.mat','Datasets/board1/depth_0003.mat','Datasets/board1/depth_0004.mat','Datasets/board1/depth_0005.mat','Datasets/board1/depth_0006.mat','Datasets/board1/depth_0007.mat','Datasets/board1/depth_0008.mat','Datasets/board1/depth_0009.mat','Datasets/board1/depth_0010.mat','Datasets/board1/depth_0011.mat','Datasets/board1/depth_0012.mat','Datasets/board1/depth_0013.mat','Datasets/board1/depth_0014.mat'};

 %imglistrgb = {'Datasets/table/rgb_0000.jpg','Datasets/table/rgb_0001.jpg','Datasets/table/rgb_0002.jpg','Datasets/table/rgb_0003.jpg','Datasets/table/rgb_0004.jpg','Datasets/table/rgb_0005.jpg','Datasets/table/rgb_0006.jpg','Datasets/table/rgb_0007.jpg'};
 %imglistdepth = {'Datasets/table/depth_0000.mat','Datasets/table/depth_0001.mat','Datasets/table/depth_0002.mat','Datasets/table/depth_0003.mat','Datasets/table/depth_0004.mat','Datasets/table/depth_0005.mat','Datasets/table/depth_0006.mat','Datasets/table/depth_0007.mat'};

num_images=length(imglistrgb);

for i=1:num_images
    images_vec{i} = imread(imglistrgb{i});
    depth_vec{i} = load(imglistdepth{i});
end
%%
affines{1} = affine3d(eye(4));

for i=1:num_images-1
    % Initialize flag that describes bad depths to 0
    flag_im1 = 0;
    flag_im2 = 0;
    
    % depths vectorized in a column array
    depth_im1 = depth_vec{i}.depth_array(:);
    depth_im2 = depth_vec{i+1}.depth_array(:);
    
    % rgb images
    rgb_im1 = images_vec{i};
    rgb_im2 = images_vec{i+1};
    
    % Intrinsic camera parameters
    depthK = cam_params.Kdepth;
    RGBK = cam_params.Krgb;
    R_d_to_rgb = cam_params.R;
    T_d_to_rgb = cam_params.T;
    
    if  sum(sum(isnan(depth_im1))) >= 1
        flag_im1=1;
    end
    if  sum(sum(isnan(depth_im2))) >= 1
        flag_im2=1;
    end
    % clean up nan in depths
    depth_im1(isnan(depth_im1))=0;
    depth_im2(isnan(depth_im2))=0;
    
    % if depth is too large, its probably wrong
    depth_im1(find(depth_im1>3*mean(depth_im1(:))))=0;
    depth_im2(find(depth_im2>3*mean(depth_im2(:))))=0;
    
    % get xyz from the depths and the images
    if flag_im1==0
        xyz_1 = get_xyzasus(depth_im1,[480 640],1:640*480,depthK,1,0);
    else 
        xyz_1 = get_xyzasus(depth_im1,[480 640],1:640*480,depthK,1,0)*1000;
    end
    if flag_im2==0
        xyz_2 = get_xyzasus(depth_im2,[480 640],1:640*480,depthK,1,0);
    else
        xyz_2 = get_xyzasus(depth_im2,[480 640],1:640*480,depthK,1,0)*1000;
    end
    
    % create virtual rgb-depth images
    rgbd_im1 = get_rgbd(xyz_1,rgb_im1,R_d_to_rgb,T_d_to_rgb,RGBK);
    rgbd_im2 = get_rgbd(xyz_2,rgb_im2,R_d_to_rgb,T_d_to_rgb,RGBK);
    
    % use the new rgbd images to detect sift features
    [F_im1 d_im1]= vl_sift(single(rgb2gray(rgbd_im1)), 'PeakThresh', 0, 'edgethresh', 10);
    [F_im2 d_im2]= vl_sift(single(rgb2gray(rgbd_im2)), 'PeakThresh', 0, 'edgethresh', 10);
    
    % Match sift features
    match_threshold = 1.5;
    
    [matches, weights] = vl_ubcmatch(d_im1, d_im2, match_threshold);
    
    while size(matches,2) < 4
        match_threshold = match_threshold - 0.1;
        [matches, weights] = vl_ubcmatch(d_im1, d_im2, match_threshold);
    end
    
    p1 = F_im1(1:2, matches(1,:));
    p2 = F_im2(1:2, matches(2,:));
    
    p1=round(p1)';
    p2=round(p2)';
    
    matches_1 =(p1(:,1)-1)*480+p1(:,2);
    matches_2 =(p2(:,1)-1)*480+p2(:,2);   
    %%
    % get the xyz at the points corresponding to features
    xyz_1_features = xyz_1(matches_1, :);
    xyz_2_features = xyz_2(matches_2, :);  
    
    % Filter features with 0s 
    xyz = horzcat(xyz_1_features,xyz_2_features);
    
    a = xyz(:,3)~=0;
    b = xyz(:,6)~=0;
    inds = a.*b;
    
    xyz = xyz(inds(:)==1,:);
    
    p1 = p1(inds(:)==1,:);
    p2 = p2(inds(:)==1,:);
    
    xyz_1_features = xyz(:,1:3);
    xyz_2_features = xyz(:,4:6);
    
    % get the indices of the inliers in the set of features with ransac
    e = 0.025;
    [inliers, error, no_inliers] = ransac_function(xyz_1_features, xyz_2_features, e);
    
    while no_inliers == 1
        e = e+0.01;
        [inliers, error, no_inliers] = ransac_function(xyz_1_features, xyz_2_features, e);
    end
    
    % Compute transformation using only the matched features that are 
    % considered inliers
    inlier_xyz_1 = xyz_1_features(inliers,:);
    inlier_xyz_2 = xyz_2_features(inliers,:);
    
    % Compute centroids with inliers
    centroid1 = mean(inlier_xyz_1',2);
    centroid2 = mean(inlier_xyz_2',2);
    
    % Covariance with inliers
    H = (inlier_xyz_1'-centroid1)*(inlier_xyz_2'-centroid2)';
    
    % SVD
    [U,S,V] = svd(H);
    
    R = V*U';
    
    if det(R)<0
        [U,S,V] = svd(R);
        V(:,end) = -V(:,end);
        R = V * transpose(U);
    end
    
    T = centroid2 - R*centroid1;
        
    % cell array with rgbd images
    rgbd_images{i} = rgbd_im1;
    rgbd_images{i+1} = rgbd_im2;
     
    % cell array with affine transforms
    affines{i+1} = invert(affine3d(([R T; 0 0 0 1]')));
    
    pc_inlier_2 = pointCloud(inlier_xyz_2);
    
    pc_inlier1_hat = pctransform(pc_inlier_2, affines{i+1});
    
    xyz_1_features_hat = pc_inlier1_hat.Location;
    
    
    % Calculate the difference between estimated and real points
    dist_svd = inlier_xyz_1-xyz_1_features_hat;
    
    % Calculate the error
    dist_svd = dist_svd.^2;
    error_svd = sqrt(sum(dist_svd,2));

    % Compare icp to svd
    [Points_Moved,M]=ICP_finite(inlier_xyz_1, xyz_1_features_hat);
    pc_icp = pointCloud(xyz_1_features_hat);
    xyz_1_features_hat_pc = pctransform(pc_icp, affine3d(M'));
    xyz_1_features_hat = xyz_1_features_hat_pc.Location;
    dist_icp = inlier_xyz_1-xyz_1_features_hat;
    error_icp = sqrt(sum(dist_icp.^2,2));
    if sum(error_icp)<sum(error_svd)
        affines{i+1} = affine3d(M'*affines{i+1}.T);
        disp('lmaooo');
    end
    

    % cell array with point clouds
    pc_vec{i} = pointCloud(xyz_1, 'Color', reshape(rgbd_im1,480*640,3));
    pc_vec{i+1} = pointCloud(xyz_2, 'Color', reshape(rgbd_im2,480*640,3));
    
    clearvars matches;
    
end
%%
for i=num_images:-1:1
    for j=i-1:-1:1
        affines{i} = affine3d(affines{i}.T*affines{j}.T);
    end
end

%% Detect objects
k=1;
for i=1:num_images

   [labeledImage] = detect_objects(images_vec{i});
   
   num_components = max(labeledImage(:));
   for j=1:num_components
       
       xyz_idx = find(labeledImage==j);
              
       xyz = pc_vec{i}.Location(xyz_idx,:);
       
       vectorized_img = reshape(images_vec{i},480*640,3);
%         
%        xyz_cloud = zeros(307200,3);
%        xyz_cloud(xyz_idx,:)=xyz;
       %valid_inds = find(xyz(:,1)~=0);
       
       %xyz = xyz(valid_inds,:);
       vectorized_img = vectorized_img(xyz_idx,:);
       if size(xyz,1)>50
           xyz_cloud = pointCloud(xyz, 'Color', vectorized_img);
           objects{k}.framenum = i;
           object_in_frame1 = pctransform(xyz_cloud, affines{i});
           
           pcshow(object_in_frame1);
           hold on;

           objects{k}.xyz = object_in_frame1.Location;
           k = k+1;
       end
   end
end

% %
% full_cloud = pc_vec{1};
% pc_vec_transformed{1} = pc_vec{1};
% for i=2:num_images
%     pc_1_hat = pctransform(pc_vec{i}, affines{i});
%     pc_vec_transformed{i} = pc_1_hat;
%     full_cloud = pcmerge(full_cloud, pc_1_hat, 0.005);
% end
% 
% pcshow(full_cloud);
