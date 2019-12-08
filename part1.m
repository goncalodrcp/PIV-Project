function [H, world_pts_x, world_pts_y]= part1(image_list, match_list,K ,pointsw)
%H - a cellarray (number of images x number of images) with all transformations between images. Each element H{i,j} containts the homography between image i and image j.
%world_pts_x - a double array (number of images x 4) with the X coordinates of the 4 image corners
%world_pts_y - a double array (number of images x 4) with the Y coordinates of the 4 imagecorners
%image_list - a cellarray where each element containts a string with the image file name.
%match_list - a (number of images -1 X 2) cellarray  with the coordinates of of matching points. match_list{i,1} is a 4x2 array with the xy coordinates of the 4 points of image i and match_list{i, 2} the cooresponding xy coordinates of the same point in imagem i+1.
%K - the 3x3 matrix with the camera intrinsic parameters
%pointsw - a 4x2 array with the XY coordinates in the world reference frame of the 4 points of image 1.

%% Create montage

%create image set
imds = imageDatastore(image_list);

numImages = numel(image_list); %number of images to stitch


%% Loop to iterate over all images

H=cell(numImages,numImages); %Create cell array for the transformations

H{1,1} = eye(3);

I_curr = imread(image_list{1});

I_curr = im2single(I_curr);

grayImage = rgb2gray(I_curr);

[f_curr, d_curr] = vl_sift(single(grayImage));

for i=2:numImages   
   
    % Store the features from image i-1
    f_prev = f_curr;
    d_prev = d_curr;
    
    I_next = imread(image_list{i});
    I_next = im2single(I_next);

    grayImage_next = rgb2gray(I_next);
    
    % Gets the features for image i using sift
    [f_curr, d_curr] = vl_sift(single(grayImage_next));

    % gets just the points that correspond to good features
    [matches, scores] = vl_ubcmatch(d_prev, d_curr);
    matchedPoints_prev = f_prev(1:2,matches(1,:)); matchedPoints_prev(3,:) = 1 ;
    matchedPoints_curr = f_curr(1:2,matches(2,:)); matchedPoints_curr(3,:) = 1 ;

    num_points = size(matchedPoints_prev,2);
    %Ransac
    % Variables: -homos will contain n homographies. Each homography is computed 
    %             using p lines randomly chosen from matrix A.
    %            -numinliers stores the number of inliers for each
    %            homography in homos
    num_iter = 1000;
    homos=cell(1, num_iter);
    errorthresh = 36;
    p=4;
    % Ransac loop for projectives
    clear H_i score ok ;
    for r=1:num_iter
        % Generate random indexes
        subset = vl_colsubset(1:num_points, p) ;
        A = [] ;
        % Compute matrix A from the subset of points
        for s = subset
            A = cat(1, A, kron(matchedPoints_prev(:,s)', vl_hat(matchedPoints_curr(:,s)))) ;
        end
        % Singular value decomposition
        [U,S,V] = svd(A);
        h = reshape(V(:,9),3,3);

        homos{r} = h;

        % Use the model to compute the estimated points of image I(i)
        x_hat = h*matchedPoints_prev;
        
        % Calculate the difference between estimated and real points
        du = x_hat(1,:)./x_hat(3,:) - matchedPoints_curr(1,:)./matchedPoints_curr(3,:);
        dv = x_hat(2,:)./x_hat(3,:) - matchedPoints_curr(2,:)./matchedPoints_curr(3,:);
        
        % Calculate the error 
        error_ransac = (du.*du + dv.*dv);       

        % Attribute a score to each homography
        ok{r} = error_ransac < errorthresh;
        score(r) = sum(ok{r});
    end
 
    % Choose the best homography
    [score, best] = max(score) ;
    h = homos{best} ;

    % Use the model to compute the estimated points of image I(i)
    x_hat = h*matchedPoints_prev;
    
    % Calculate the difference between estimated and real points
    du = x_hat(1,:)./x_hat(3,:) - matchedPoints_curr(1,:)./matchedPoints_curr(3,:);
    dv = x_hat(2,:)./x_hat(3,:) - matchedPoints_curr(2,:)./matchedPoints_curr(3,:);
    
    % Calculate the error
    error_ransac = (du.*du + dv.*dv);
    inlier_inds=find(error_ransac<errorthresh);
    
    A = [] ;
    % Compute matrix A from the subset of points
    for s = inlier_inds
        A = cat(1, A, kron(matchedPoints_prev(:,s)', vl_hat(matchedPoints_curr(:,s)))) ;
    end
    % Singular value decomposition
    [U,S,V] = svd(A);
    h = reshape(V(:,9),3,3);

    H{i,i} = eye(3);
    H{i-1,i} = h;

end

% Calculate the upper triangle of the cell array of homographies by using
% composition property -> H13 = H12*H23
for i=1:numImages
    for j=i+1:numImages
        H{i,j} = H{i,j-1}*H{j-1,j};
    end
end

% Compute the lower triangle of H by inverting the upper triangle e.g.
% H31=inv(H13)
for j=1:numImages
    for i=j+1:numImages
        H{i,j} = inv(H{j,i});
    end
end

% Normalize all homographies such that h(3,3)=1
for i=1:numImages
    for j=1:1:numImages
        H{i,j} = H{i,j}/H{i,j}(end);
    end
end

if nargin>2
    % Calcular world coords
    u_v_1_set = reshape([1 1 1  1 size(grayImage,1) 1  size(grayImage,2) size(grayImage,1) 1  size(grayImage,2) 1 1  ], [3 4]);
    x_y_1_set = [pointsw(:,1)' ; pointsw(:,2)'; ones(1,4)];
    x = pointsw(:,1);
    y = pointsw(:,2);
    
    u_v_1_set = u_v_1_set';
    w1 = u_v_1_set(:,1);
    w2 = u_v_1_set(:,2);
    
    World_mat(1:2:8,:) = [x y ones(4,1) zeros(4,3) -w1.*x -w1.*y -w1];
    World_mat(2:2:8,:) = [zeros(4,3) x y ones(4,1) -w2.*x -w2.*y -w2];
    
%     World_mat=[];
%     for s = 1:4
%         World_mat = cat(1, World_mat, kron(u_v_1_set(:,s)', vl_hat(x_y_1_set(:,s)))) ;
%     end
%     
%     [U,S,V] = svd(World_mat);
%     world_rot = reshape(V(:,9),3,3);

    world_rot = World_mat(:,1:8)\(-World_mat(:,9));
    world_rot = [reshape([world_rot;1],3,3)]';
    
    world_coords=inv(world_rot)*u_v_1_set';

    world_pts_x(1,:)=world_coords(1,:)./world_coords(3,:);
    world_pts_y(1,:)=world_coords(2,:)./world_coords(3,:);
    
    for i=2:numImages
        corners_inIm1 = H{i,1}*u_v_1_set';
        world_coords=inv(world_rot)*corners_inIm1;
        world_pts_x(i,:)=world_coords(1,:)./world_coords(3,:);
        world_pts_y(i,:)=world_coords(2,:)./world_coords(3,:);
    end

end
end

