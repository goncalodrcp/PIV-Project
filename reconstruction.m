%% Reconstruction
% 
% Usage [pcloud, transforms] = reconstruct(image_names, K_depth, K_rgb, Rdtrgb,Tdtrgb)
%
% Arguments:
%           image_names           - array of structures with each structure
%                                   having two fields, the path to the .mat
%                                   file with the depth data of test image
%                                   i and the path to the png image
%           K_depth               - a 3x3 matrix with the intrinsic parameters
%           K_rgb                 - a 3x3 matrix with the intrinsic parameters
%           Rdtrgb and Tdrgb      - allow transforming 3D coordinates 
%                                   represented in the depth camera  
%                                   reference frame to the RGB camera frame
%
% Returns:
%           pcloud                -  One  N x 6 matrix with the 3D points and RGB data of each point,  
%                                    represented in the world reference frame (you choose the reference frame! 
%                                    For example, could be  the depth camera coordinate system of the first image).
%           transforms            - an array of structures with the same size of image_name 
%                                   where each element contains the transformation between 
%                                   the depth camera reference frame and the world reference frame 
%                                   (as selected above) for image k.

function [pcloud, transforms] = reconstruction( image_names, K_depth, K_rgb, Rdtrgb,Tdtrgb )
    
    %change this to the right directory
    run('vlfeat-0.9.20\toolbox\vl_setup');
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
        xyz = get_xyzasus(depth(:), image_struct(i).size, 1:length(depth), K_depth, 1, 0);
        rgbd = get_rgbd(xyz, im, Rdtrgb, Tdtrgb, K_rgb);
        cl = reshape(rgbd,480*640,3);
        cloud_struct(i) = struct('point_cloud', pointCloud(xyz, 'Color', reshape(im, im_size(1)*im_size(2), 3)), 'xyz', xyz, 'rgb', rgbd, 'cl', cl);
        %structure containing info about each Point Cloud and 3D point.
    end

    clearvars im_name depth_name im im_gray depth im_size xyz;


    %At� aqui temos 3 vetores de estruturas, 1 com imagens e outro com point clouds.
    %A estrutura de cada imagem tem a imagem rgb, gray scale, tamanho da imagem e
    %o depth array da imagem.
    %A estrutura de cada point cloud tem a point cloud e a matriz dos x, y e z.
    %Temos tamb�m uma estrutura correspondente � camara, com os par�metros todos.

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

    for i = 1 : length(image_names)-1
        aux = weights(i,:);
        for j = 1 : n
            %finds the minimum weight
            [values(i, j), best_features(i, j)] = min(aux(aux>0));
            %removes the best feature found, by giving it infinity value, 
            %so that it does not repeat.
            aux(best_features(i, j)) = inf;
        end
    end

    clearvars aux;
    
    %gets the u and v coordinates for each of the best features calculated
    %previously in the rgb image
    for i = 1 : length(image_names)-1
        for j = 1 : n
            u1(i,j) = im_features(i).features(1,match(i,1,best_features(i,j)));
            v1(i,j) = im_features(i).features(2,match(i,1,best_features(i,j)));
            u2(i,j) = im_features(i+1).features(1,match(i,2,best_features(i,j)));
            v2(i,j) = im_features(i+1).features(2,match(i,2,best_features(i,j)));
        end
    end
   
    
    indices = zeros(3, 2, n);
    for i = 1 : length(image_names)-1
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
    end

    clearvars u1_aux u2_aux v1_aux v2_aux;

    for i = 1 : length(image_names)-1
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
        T(i,:) = centroid(:,2) - (c*a')*centroid(:,1);
    end

    clearvars point_cloud1 point_cloud2 a b c;
    
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
    
    %Computation of pcloud, a matrix containg, for each line, the
    %information about the x, y and z coordinates and the rgb colors for
    %every given point in the image
    for i = 1 : length(cloud_struct(1,:))
        xyz = cloud_struct(i).point_cloud.Location;
        color = cloud_struct(i).point_cloud.Color;
        pcloud = [xyz color];
    end

end