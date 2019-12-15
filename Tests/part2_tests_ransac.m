load calib_asus.mat

d1=dir('Datasets/imgs/*.mat');
d2=dir('Datasets/imgs/*.jpg');
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

image_names=data;
K_depth=Depth_cam.K;
 K_rgb = RGB_cam.K;
 Rdtrgb = R_d_to_rgb;
 Tdtrgb = T_d_to_rgb;

    %change this to the right directory
    %% Variables initialization

    for i = 1 : length(image_names)
        im = imread(image_names(i).rgb);
        %im is a 480x640x3 uint8 - unsigned 8 bit integer.
        %480 is the number of lines, 640 is the number of columns, 3 used to specify the color.
        im_gray = rgb2gray(im);
        %The rgb2gray function converts RGB images to grayscale by eliminating the hue and saturation 
        %information while retaining the luminance.
        depth = load(image_names(i).depth);
        depth = depth.depth_array;
        %stores the depth_array [depth for each (x,y) pair].
        im_size = size(im);

        image_struct(i) = struct('original', im, 'gray', im_gray, 'depth_array', depth, 'size', [im_size(1) im_size(2)]);
        %structure containing info about each image.
        xyz = get_xyz_asus(depth(:), image_struct(i).size, 1:length(depth), K_depth, 1, 0);
        rgbd = get_rgbd(xyz, im, Rdtrgb, Tdtrgb, K_rgb);
        cl = reshape(rgbd,480*640,3);
        cloud_struct(i) = struct('point_cloud', pointCloud(xyz, 'Color', reshape(im, im_size(1)*im_size(2), 3)), 'xyz', xyz, 'rgb', rgbd, 'cl', cl);
        %structure containing info about each Point Cloud and 3D point.
    end

    clearvars im_name depth_name im im_gray depth im_size xyz;


    %Até aqui temos 3 vetores de estruturas, 1 com imagens e outro com point clouds.
    %A estrutura de cada imagem tem a imagem rgb, gray scale, tamanho da imagem e
    %o depth array da imagem.
    %A estrutura de cada point cloud tem a point cloud e a matriz dos x, y e z.
    %Temos também uma estrutura correspondente à camara, com os parâmetros todos.

    %% Get image features

    for i = 1 : length(image_names)
        [F d]= vl_sift(single(image_struct(i).gray));
        %computes the SIFT frames (keypoints) and descriptors of the image I. 
        %I is a gray-scale image in single precision. 
        %-Each column of F is a feature frame and has the format [X;Y;S;TH], 
        %where X,Y is the (fractional) center of the frame, 
        %S is the scale and TH is the orientation (in radians).
        %-Each column of D is the descriptor of the corresponding frame in F. 
        %A descriptor is a 128-dimensional vector of class UINT8.
        %The descriptor associates to the regions a signature which identifies 
        %their appearance compactly and robustly.
        %The SIFT feature descriptor is invariant to uniform scaling, 
        %orientation, and partially invariant to affine distortion and 
        %illumination changes.
        im_features(i) = struct('features', F, 'descriptor', d);
        %structure containing the features and the correspondant descriptor for
        %each image.
    end

    %% Matching features

    for i = 1 : length(image_names) - 1
        [matches, weight] = vl_ubcmatch(im_features(i).descriptor, im_features(i+1).descriptor);
        for j = 1 : 2
            for w = 1 : length(matches(1,:))    
                match(i,j,w) = matches(j,w);
                weights(i,w) = weight(w);
            end
        end
        clearvars matches;
    end

    %% Stitching
    n = 5; %number of best features we want.

    values = zeros(n, 1); %variable to store the minimum weights found
    best_features = zeros(length(image_names)-1, n); %position of each best pair.
    %declaration of an auxiliar variable for the weights values so that we
    %can edit the weight values without loosing the original vector with
    %weights
    aux = zeros(1, length(weights(1,:)));

    % For each image we will run ransac
    for i = 1 : length(image_names)-1
        aux = weights(i,:);
        
        % Ransac with 100 iterations
        for r=1:100
            
            % Goes through the vl_ubcmatches and picks the best ones
            for j = 1 : n
                %finds the minimum weight
                [values(i, j), best_features(i, j)] = min(aux(aux>0));
                %removes the best feature found, by giving it infinity value, 
                %so that it does not repeat.
                aux(best_features(i, j)) = inf;
            end


        %Selects the u and v coordinates for each of the best features calculated
        %previously in the rgb image
            for j = 1 : n
                u1(i,j) = im_features(i).features(1,match(i,1,best_features(i,j)));
                v1(i,j) = im_features(i).features(2,match(i,1,best_features(i,j)));
                u2(i,j) = im_features(i+1).features(1,match(i,2,best_features(i,j)));
                v2(i,j) = im_features(i+1).features(2,match(i,2,best_features(i,j)));
            end


        indices = zeros(3, 2, n);
            for j = 1 : n
               u1_aux(j) = uint64(u1(i,j));
               u2_aux(j) = uint64(u2(i,j));
               v1_aux(j) = uint64(v1(i,j));
               v2_aux(j) = uint64(v2(i,j));
            end
            %Convert subscripts to linear indices
            %These linear indices point the best matches found in the xyz
            %matrix previously found describing the 3D points
            indices(i,1,:) = sub2ind([image_struct(1).size(1) image_struct(1).size(2)], v1_aux, u1_aux);
            indices(i,2,:) = sub2ind([image_struct(1).size(1) image_struct(1).size(2)], v2_aux, u2_aux);

            %Computation of the centroids for each set of n(number of best
            %matches) best matches in the 3D referencial by doing the mean of
            %the 3D points corresponding to the features that feature in the
            %best matches.
            centroid(:,1) = mean(cloud_struct(i).xyz(indices(i,1,:),:))';
            centroid(:,2) = mean(cloud_struct(i+1).xyz(indices(i,2,:),:))';
            %Computation of the point clouds of the two images involved in the
            %transformation at hand and construction of the Rotation Matrix and
            %Translation vector between them, using singular value
            %decomposition of the product of the two point clouds.
            point_cloud1 = cloud_struct(i).xyz(indices(i,1,:),:)' - repmat(centroid(:,1),1,n);
            point_cloud2 = cloud_struct(i+1).xyz(indices(i,2,:),:)' - repmat(centroid(:,2),1,n);

            [a,b,c] = svd(point_cloud1*point_cloud2');

            R(i,:,:) = (c*a');
            R_aux(:,:)=R(i,:,:);
            if det(R_aux)<0
                [a,b,c] = svd(R_aux);
                c(:,end)=-c(:,end);
                R(i,:,:) = c*a';
            end
            T(i,:) = centroid(:,2) - (c*a')*centroid(:,1);
            
            
            %R_aux contains the rotation from i to i+1
            %T is the translation from i to i+1
            %xyz_aux is the pointcloud i
            %xyz_hat is the pointcloud i transformed to i+1
            R_aux(:,:)=R(i,:,:);
            Rotations{r}=R_aux;
            Translations{r}=T(i,:);
            xyz_aux = cloud_struct(i).xyz';
            xyz_hat = R_aux*xyz_aux + repmat(T(i,:)',1,length(cloud_struct(i).xyz));
            
            %error
            error = xyz_hat - cloud_struct(i+1).xyz';
            error = sum(error.^2, 1);
            
            ok{r} = error < 0.02;
            score(r) = sum(ok{r});
        end
        
        [score, best] = max(score);
        
        R_best=Rotations{best};
        T_best=Translations{best};
        
        xyz_hat = R_best*xyz_aux + repmat(T_best',1,length(cloud_struct(i).xyz));
            
        %error
        error = xyz_hat - cloud_struct(i+1).xyz';
        error = sum(error.^2, 1);
        
        %Indices of the inliers are those that have error smaller than 20cm
        inlier_inds=find(error<0.02);
        
        centroid(:,1) = mean(cloud_struct(i).xyz(inlier_inds,:))';
        centroid(:,2) = mean(cloud_struct(i+1).xyz(inlier_inds,:))';
        %Computation of the point clouds of the two images involved in the
        %transformation at hand and construction of the Rotation Matrix and
        %Translation vector between them, using singular value
        %decomposition of the product of the two point clouds.
        point_cloud1 = cloud_struct(i).xyz(inlier_inds,:)' - repmat(centroid(:,1),1,length(inlier_inds));
        point_cloud2 = cloud_struct(i+1).xyz(inlier_inds,:)' - repmat(centroid(:,2),1,length(inlier_inds));
        
        [a,b,c] = svd(point_cloud1*point_cloud2');
        
        R(i,:,:) = (c*a');
        R_aux(:,:)=R(i,:,:);
        if det(R_aux)<0
            [a,b,c] = svd(R_aux);
            c(:,end)=-c(:,end);
            R(i,:,:) = c*a';
        end
        T(i,:) = centroid(:,2) - (c*a')*centroid(:,1);
    end

    clearvars point_cloud1 point_cloud2 a b c R_aux;
    %%
    %The chosen referencial for the world frame is the first image
    xyz_aux = cloud_struct(1).xyz';
    xyz_struct(1) = struct('xyz', xyz_aux);
    %The rotation matrix for the referencial frame is the identity matrix
    %and the translation vector is a collumn of zeros since there is no
    %translation and rotation between one frame and itself
    transforms(1) = struct('R', [1 0 0; 0 1 0; 0 0 1], 'T', [0 0 0]');
    
    %Computation of each xyz matrix, by applying a rotation and translation to the previous set of xyz 
    for i = 1 : length(image_names) - 1
        xyz_aux = xyz_struct(i).xyz;
        R_aux(:,:) = R(i,:,:);
        xyz_struct(i+1) = struct('xyz',R_aux*xyz_aux + repmat(T(i,:)',1,length(cloud_struct(i).xyz)));
        transforms(i+1) = struct('R',R_aux,'T',T(i,:)');
    end
    %%
    %Computation of pcloud, a matrix containg, for each line, the
    %information about the x, y and z coordinates and the rgb colors for
    %every given point in the image
    for i = 1 : length(cloud_struct(1,:))
        xyz = cloud_struct(i).point_cloud.Location;
        color = cloud_struct(i).point_cloud.Color;
        %pcloud(:,:,i) = [xyz color];
        pc=pointCloud(xyz, 'Color', cloud_struct(i).point_cloud.Color);
    end
    pcshow(pc);