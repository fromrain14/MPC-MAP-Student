function [public_vars] = init_particle_filter(read_only_vars, public_vars)

N = read_only_vars.max_particles;
lim = read_only_vars.map.limits; % [xmin, ymin, xmax, ymax]

% Pokud je definována pozice pro lokální inicializaci (přechod Outdoor -> Indoor)
if isfield(public_vars, 'init_around_pose') && ~isempty(public_vars.init_around_pose)
    pose = public_vars.init_around_pose;
    
   
    std_xy = 0.1;    
    std_theta = 0.2; 
    
    xs = pose(1) + randn(N, 1) * std_xy;
    ys = pose(2) + randn(N, 1) * std_xy;
    thetas = pose(3) + randn(N, 1) * std_theta;
    
   
    xs = max(lim(1), min(lim(3), xs));
    ys = max(lim(2), min(lim(4), ys));
    

    public_vars.init_around_pose = []; 
else
    % Původní globální inicializace
    xs = lim(1) + (lim(3) - lim(1)) * rand(N,1);
    ys = lim(2) + (lim(4) - lim(2)) * rand(N,1);
    thetas = -pi + 2*pi*rand(N,1);
end

public_vars.particles = [xs, ys, thetas];

end