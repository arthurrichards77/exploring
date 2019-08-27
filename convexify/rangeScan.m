function [rScan,fScan,xScan,yScan] = rangeScan(cx,cy,occ,occBox,thetaRng,rmax)

% and the size of the occupancy image
occSize = size(occ);

% get start pixel
[cx_p,cy_p] = convWorldToOcc(cx,cy,occSize,occBox);

for ii=1:numel(thetaRng),
    
    % angle
    theta = thetaRng(ii);
    
    % end point
    ex = cx+rmax*cos(theta);
    ey = cy+rmax*sin(theta);
    
    % and end pixel
    [ex_p,ey_p] = convWorldToOcc(ex,ey,occSize,occBox);
    
    % draw the ray
%     hold on
%     plot(cx_p, cy_p, 'm.', ...
%         [cx_p ex_p], [cy_p ey_p], 'c')
%     
    % default return value
    f = false;
    
    % default initial length is maximum
    r = rmax;
    
    % loop through pixels along longest change
    if abs(ex_p-cx_p)>abs(ey_p-cy_p),
        
        % trace along pixels in X
        for xx = cx_p:sign(ex_p-cx_p):ex_p,
            yy = ceil(cy_p + (ey_p-cy_p)*(xx-cx_p)/(ex_p-cx_p));
            % check point is inside the box
            if ((yy>0)&&(xx>0)&&(yy<=occSize(1))&&(xx<=occSize(2))),
                % inside the occ grid - check the occupancy grid
                if occ(yy,xx),
                    r = sqrt((xx-cx_p)*(xx-cx_p) + (yy-cy_p)*(yy-cy_p));
                    f = true;
                    break
                    %else
                    %plot(xx,yy,'g.')
                end
            else
                % outside the occ grid - terminate search and mark as
                % range, but without reflection
                r = sqrt((xx-cx_p)*(xx-cx_p) + (yy-cy_p)*(yy-cy_p));
                f = false;
                break
            end
        end
        
    else,
        
        % trace along pixels in X
        for yy = cy_p:sign(ey_p-cy_p):ey_p,
            xx = ceil(cx_p + (ex_p-cx_p)*(yy-cy_p)/(ey_p-cy_p));
            % check point is inside the box
            if ((yy>0)&&(xx>0)&&(yy<=occSize(1))&&(xx<=occSize(2))),
                % inside the occ grid - check the occupancy grid
                if occ(yy,xx),
                    r = sqrt((xx-cx_p)*(xx-cx_p) + (yy-cy_p)*(yy-cy_p));
                    f = true;
                    break
                    %else
                    %plot(xx,yy,'g.')
                end
            else
                % outside the occ grid - terminate search and mark as
                % range, but without reflection
                r = sqrt((xx-cx_p)*(xx-cx_p) + (yy-cy_p)*(yy-cy_p));
                f = false;
                break
            end
        end
        
    end
    
    % plot the reading
%     plot(xx, yy, 'r.', ...
%         [cx_p xx], [cy_p yy], 'g')
%     
    % store it
    fScan(ii) = f;
    [xScan(ii),yScan(ii)] = convOccToWorld(xx,yy,occSize,occBox);    
    rScan(ii) = norm([xScan(ii)-cx; yScan(ii)-cy]);
    
end


