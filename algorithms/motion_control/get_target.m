function [target] = get_target(estimated_pose, path)
LOOKAHEAD_DIST=1.5;
target=path(end, :);
px=estimated_pose(1);
py=estimated_pose(2);

for i=1:size(path,1)-1
    ax=path(i,1);%zacatek usecky
    ay=path(i,2);
    bx=path(i+1,1);%konec usecky
    by=path(i+1,2);
    dx=bx-ax;%smerovy vektor usecky
    dy=by-ay;
    fx=ax-px;%vektor od robota k zacatku usecky
    fy=ay-py;
    a= dx^2 + dy^2;%koeficienty
    b=2*(fx*dx + fy*dy);
    c=fx^2 + fy^2 - LOOKAHEAD_DIST^2;
    disc = b^2 - 4*a*c;
    if disc < 0, continue; end
    t = (-b + sqrt(disc)) / (2*a);
    if t >= 0 && t <= 1
        target = [ax + t*dx, ay + t*dy];
        return;
    end
end
end