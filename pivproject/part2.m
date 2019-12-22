function [transforms, objects] = part2(imglistdepth, imglistrgb, cam_params)

num_images=length(imglistrgb);

for i=1:num_images
    images_vec{i} = imread(imglistrgb{i});
    depth_vec{i} = load(imglistdepth{i});
end

% Intrinsic camera parameters
depthK = cam_params.Kdepth;
RGBK = cam_params.Krgb;
R_d_to_rgb = cam_params.R;
T_d_to_rgb = cam_params.T;

% transform 1 to 1 is the identity
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
    
    % Check if there are NaN
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
    
    % get xyz from the depths and the images, if there are NaN the depths
    % are also in the wrong measurement unit -> multiply by 1000
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
    match_threshold = 2.5;
    
    [matches, weights] = vl_ubcmatch(d_im1, d_im2, match_threshold);
    
    % If there are less than 4 matches keep trying with smaller threshold
    while size(matches,2) < 4
        match_threshold = match_threshold - 0.1;
        [matches, weights] = vl_ubcmatch(d_im1, d_im2, match_threshold);
    end
    
    % Matched points
    p1 = F_im1(1:2, matches(1,:));
    p2 = F_im2(1:2, matches(2,:));
    
    p1=round(p1)';
    p2=round(p2)';
    
    % Transform to xyz indices
    matches_1 =(p1(:,1)-1)*480+p1(:,2);
    matches_2 =(p2(:,1)-1)*480+p2(:,2);   

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
    end
    

    % cell array with point clouds
    pc_vec{i} = pointCloud(xyz_1, 'Color', reshape(rgbd_im1,480*640,3));
    pc_vec{i+1} = pointCloud(xyz_2, 'Color', reshape(rgbd_im2,480*640,3));
    
    clearvars matches;
    
end
% Compute all transforms on the frame of the 1st pcloud
for i=num_images:-1:1
    for j=i-1:-1:1
        affines{i} = affine3d(affines{i}.T*affines{j}.T);
    end
end

% Store the transformations in the correct format to be returned
for i=1:num_images
    affine_aux = affines{i}.T';
    transforms{i}.R = affine_aux(1:3,1:3);
    transforms{i}.T = affine_aux(1:3,4);
end

% Detect objects
k=1;
for i=1:num_images

   labeledImage = detect_objects(images_vec{i});
   num_components = max(labeledImage(:));
   for j=1:num_components
       
       xyz_idx = find(labeledImage==j);
              
       xyz = pc_vec{i}.Location(xyz_idx,:);
       
       valid_inds = find(xyz(:,1)~=0);
       
       xyz = xyz(valid_inds,:);
       
       % Only accept objects with more than 40 connected points
       if size(xyz,1)>40
           xyz = pointCloud(xyz);
           objects{k}.framenum = i;
           
           object_in_frame1 = pctransform(xyz, affines{i});

           objects{k}.xyz = object_in_frame1.Location;
           
           k = k+1;
       end
   end
end

end