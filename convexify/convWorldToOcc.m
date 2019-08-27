function [x_p,y_p] = convWorldToOcc(x,y,occSize,occBox)
%
% convert real world coordinates to pixel values
% does not truncate at image boundaries
%
%  'occ' is a 2D occupancy grid
%  'occBox' contains the limits of the grid in real world coordinates
%     format is [Xmin Xmax Ymin Ymax]
%

% scale
x_p = ceil(occSize(2)*(x - occBox(1))/(occBox(2)-occBox(1)));
y_p = ceil(occSize(1)*(y - occBox(3))/(occBox(4)-occBox(3)));