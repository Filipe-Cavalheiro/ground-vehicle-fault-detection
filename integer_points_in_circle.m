function points = integer_points_in_circle(c, rx, ry)
    % c: center of the circle [cx, cy]
    % r: radius of the circle
    % points: N-by-2 array of integer points [x, y] inside or intersected by the circle

    cx = c(1);
    cy = c(2);

    % Bounding box for the search
    xmin = floor(cx - rx - 1);
    xmax = ceil(cx + rx + 1);
    ymin = floor(cy - ry - 1);
    ymax = ceil(cy + ry + 1);

    points = [];

    for x = xmin:xmax
        for y = ymin:ymax
            % Check if the unit square from (x, y) to (x+1, y+1) intersects the circle
            % Find the closest point on the square to the circle center
            closest_x = max(x, min(cx, x + 1));
            closest_y = max(y, min(cy, y + 1));
            
            % Compute normalized squared distance
            dx = (closest_x - cx) / rx;
            dy = (closest_y - cy) / ry;
            dist_norm_sq = dx^2 + dy^2;

            if dist_norm_sq <= 1
                points = [points; x, y];
            end
        end
    end
end
