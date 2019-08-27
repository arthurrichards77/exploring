function [x,y] = convOccToWorld(x_p,y_p,occSize,occBox)
%
% convert real world coordinates to pixel values
% does not truncate at image boundaries
%
%  'occ' is a 2D occupancy grid
%  'occBox' contains the limits of the grid in real world coordinates
%     format is [Xmin Xmax Ymin Ymax]
%

% scale
x = occBox(1) + (occBox(2)-occBox(1))*x_p/occSize(2);
y = occBox(3) + (occBox(4)-occBox(3))*y_p/occSize(1);