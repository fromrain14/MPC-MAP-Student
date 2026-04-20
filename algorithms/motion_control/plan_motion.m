function [public_vars] = plan_motion(read_only_vars, public_vars)

GOAL_THRESH=0.4;
V_BASE=read_only_vars.agent_drive.max_vel*1;

%init
if read_only_vars.counter==1
    public_vars.pp_path_id=4;
    public_vars.pp_wp_idx=1;
    public_vars.pp_done=false;
    public_vars.pp_path=generate_path(public_vars.pp_path_id);
end
public_vars.path=public_vars.pp_path;

if public_vars.pp_done
    public_vars.motion_vector=[0.0, 0.0];
    return;
end
% Pockat na dokonceni GNSS inicializace
if ~public_vars.gnss_init_done
    public_vars.motion_vector=[0,0];
    return;
end
pose=public_vars.mu;
%fprintf('mocap_pose=%.3f %.3f %.3f\n', pose(1), pose(2), pose(3));
px=pose(1);
py=pose(2);
ptheta=pose(3);

path=public_vars.pp_path;
wp_idx=public_vars.pp_wp_idx;

%update target waypoint, euklid distance
while wp_idx<=size(path, 1)
    dx=path(wp_idx,1)-px;
    dy=path(wp_idx,2)-py;
    if sqrt(dx^2 + dy^2)<GOAL_THRESH
        wp_idx=wp_idx+1;
    else
        break;
    end
end
%reached finish
if wp_idx>size(path, 1)
    public_vars.pp_done=true;
    public_vars.pp_wp_idx=wp_idx;
    public_vars.motion_vector=[0.0, 0.0];
    fprintf('Path %d completed at t=%d\n', public_vars.pp_path_id, read_only_vars.counter);
    return;
end
public_vars.pp_wp_idx = wp_idx;

%lookahead distance
remaining=path(wp_idx:end, :);
goal=get_target(pose, remaining);

%transofrmation to robot frame
dx=goal(1)-px;
dy=goal(2)-py;
xG_r=cos(ptheta)*dx+sin(ptheta)*dy;
yG_r=-sin(ptheta)*dx+cos(ptheta)*dy;

%pure pursuit
l = sqrt(xG_r^2 + yG_r^2);
if l < 1e-6
    public_vars.motion_vector = [0.0, 0.0];
    return;
end
if abs(yG_r) < 1e-6
    public_vars.motion_vector = [V_BASE, V_BASE];
    return;
end
%turning radius
R=(l^2)/(2*yG_r);
% differential drive
d=read_only_vars.agent_drive.interwheel_dist;
ratio=1.0/R; % omega/v = 1/R
vR=V_BASE*(1+ratio*d/2);
vL=V_BASE*(1-ratio*d/2);

%normalizce
max_val=max(abs([vR, vL, 1]));
vR=vR/max_val*V_BASE;
vL=vL/max_val*V_BASE;

public_vars.motion_vector =[vR,vL];

end


function path = generate_path(path_id)

switch path_id
    %primka
    case 1
    x=ones(20,1)* 5;
    y=linspace(2,8,20)';
    path=[x,y];

    case 2
%kruhovy oblouk
    cx=11;
    cy=8;
    r=6;
    angles=linspace(pi,0,30)';
    x=cx+r*cos(angles);
    y=cy+r*sin(angles);
    path=[x,y];

    case 3
%sinusoida
    x= linspace(5,18,40)';
    y= 11+3*sin((x-5)/13*2*pi);
    path=[x, y];

case 4
    % kalman
 path=[
    2,  2;
    2,  3;
    2,  4;
    2,  5;
    2,  6;
    2,  7;
    2,  8;
    3,  8;
    4,  8;
    5,  8;
    8,  8;
    11, 8;
    14, 8;
    16, 8;
    16, 7;
    16, 5;
    16, 3;
    16, 2;
];

end

end