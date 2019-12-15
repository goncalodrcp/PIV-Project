function [boundaries] = detect_objects(im_gray)
% Threshold the image to get a binary image (only 0's and 1's) of class "logical."
% Method #1: using im2bw()
%   normalizedThresholdValue = 0.4; % In range 0 to 1.
%   thresholdValue = normalizedThresholdValue * max(max(originalImage)); % Gray Levels.
%   binaryImage = im2bw(originalImage, normalizedThresholdValue);       % One way to threshold to binary
% Method #2: using a logical operation.
thresholdValue = 100;
binaryImage = im_gray > thresholdValue; % Bright objects will be chosen if you use >.

binaryImage = imfill(binaryImage, 'holes');

% bwlabel checks connected components
labeledImage = bwlabel(binaryImage, 8);  

boundaries = bwboundaries(binaryImage);

end