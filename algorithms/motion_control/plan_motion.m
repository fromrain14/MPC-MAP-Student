function [public_vars] = plan_motion(read_only_vars, public_vars)
c = read_only_vars.counter;
if c <= 120         
    public_vars.motion_vector = [0.5, 0.5];
elseif c <= 136      
    public_vars.motion_vector = [0.08, -0.1];
elseif c <= 216      
    public_vars.motion_vector = [0.5, 0.5];
elseif c <= 232      
    public_vars.motion_vector = [0.1, -0.1];
elseif c <= 375      
    public_vars.motion_vector = [0.5, 0.5];
else
    public_vars.motion_vector = [0.0, 0.0];
end
target = get_target(public_vars.estimated_pose, public_vars.path);
end
