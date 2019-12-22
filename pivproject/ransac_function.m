function [inliers, error, no_inliers] =  ransac_function(xyz_1, xyz_2, e)
num_iter = 100;
errorthresh = e;
p=4;
tforms=cell(1, num_iter);
no_inliers = 0;
for r=1:num_iter
    % Generate random indexes
    subset = vl_colsubset(1:length(xyz_1), p) ;
    
    % random xyz features
    xyz_1_subset = xyz_1(subset,:)';
    xyz_2_subset = xyz_2(subset,:)';

    % Compute the rotation and translation
    centroid1 = mean(xyz_1_subset,2);
    centroid2 = mean(xyz_2_subset,2);

    % Covariance matrix 
    H = (xyz_1_subset-centroid1)*(xyz_2_subset-centroid2)';
    
    % SVD
    [U,S,V] = svd(H);
    
    R = V*U';
    
    if det(R)<0
        [U,S,V] = svd(R);
        V(:,end) = -V(:,end);
        R = V * transpose(U);
    end
    
    T = centroid2 - R*centroid1;

    affines_r{r} = invert(affine3d([R T; 0 0 0 1]'));
    % Use the model to compute the estimated points of from xyz1 to xyz2
    pc_xyz_2 = pointCloud(xyz_2);
    pc_xyz_1_hat = pctransform(pc_xyz_2, affines_r{r});
    xyz_1_hat = pc_xyz_1_hat.Location;
    
    % Calculate the difference between estimated and real points
    dist = xyz_1-xyz_1_hat;
    
    % Calculate the error
    dist = dist.^2;
    error = sqrt(sum(dist,2));
    
    % Attribute a score to each homography
    ok{r} = error < errorthresh;
    score(r) = sum(ok{r});
end

% Choose the best homography
[score, best] = max(score) ;
affine_best = affines_r{best} ;

% Use the model to compute the estimated points of from xyz1 to xyz2
pc_xyz_2 = pointCloud(xyz_2);
pc_xyz_1_hat = pctransform(pc_xyz_2, affine_best);
xyz_1_hat = pc_xyz_1_hat.Location;

% Calculate the difference between estimated and real points
dist = xyz_1-xyz_1_hat;

% Calculate the error
dist = dist.^2;
error = sqrt(sum(dist,2));

inliers = find(error<errorthresh);

if length(inliers)<4
    no_inliers=1;
end

end
