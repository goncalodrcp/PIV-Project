function [L] = detect_objects(I)
    I = rgb2gray(I);

    [~,threshold] = edge(I,'sobel');

    fudgeFactor = 0.6;
    BWs = edge(I,'sobel',threshold * fudgeFactor);
 
    se90 = strel('line',3,90);
    se0 = strel('line',3,0);

    BWsdil = imdilate(BWs,[se90 se0]);

    BWdfill = imfill(BWsdil,'holes');


    seD = strel('diamond',2);
    BWfinal = imerode(BWdfill,seD);
    BWfinal = imerode(BWfinal,seD);
    BWfinal = imerode(BWfinal,seD);
  
    BWnobord = imclearborder(BWfinal,4);
   
    L = bwlabel(BWnobord,8);
    L = L.*bwareaopen(L, 70);
 
end

