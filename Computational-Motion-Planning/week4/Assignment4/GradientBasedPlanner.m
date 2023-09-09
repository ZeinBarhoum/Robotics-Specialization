function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient(-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************

[max_y, max_x] = size(gx);
route = start_coords;
for i = 1:max_its-1
    current = route(i,:);
    gx_current = gx(round(current(2)), round(current(1)));
    gy_current = gy(round(current(2)), round(current(1)));
    g = [gx_current, gy_current]/norm([gx_current, gy_current]);
    next = current + g;

%     next = round(next);
%     if(next(1) < 1) 
%         next(1) = 1;
%     end
%     if(next(2) < 1) 
%         next(2) = 1;
%     end
%     if(next(1) > max_x) 
%         next(1) = max_x;
%     end
%     if(next(2) > max_y) 
%         next(2) = max_y;
%     end
    route = [route;next];
    if(sqrt((end_coords(1) - next(1))^2 + (end_coords(2) - next(2))^2) < 2) 
        break
    end
end
route = double(route);

% *******************************************************************
end
