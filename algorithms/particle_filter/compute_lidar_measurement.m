function [measurement] = compute_lidar_measurement(map, pose, lidar_config)

% priprava
n=length(lidar_config);
measurement=zeros(1, n);

ray_origin=[pose(1), pose(2)];
ptheta=pose(3);

for i=1:n
    % absolutni smer paprsku
    direction=ptheta+lidar_config(i);
    intersections=ray_cast(ray_origin, map.walls, direction);
    if isempty(intersections)
        % paprsek nedosel na zed
        measurement(i)=inf;
    else
        dx=intersections(:,1)-ray_origin(1);
        dy=intersections(:,2)-ray_origin(2);
        dists=sqrt(dx.^2+dy.^2);
        measurement(i)=min(dists);
        
    end
end
end