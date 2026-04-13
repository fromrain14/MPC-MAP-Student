function [public_vars] = init_particle_filter(read_only_vars, public_vars)

N=read_only_vars.max_particles;
lim=read_only_vars.map.limits; % [xmin, ymin, xmax, ymax]

% nahodne polohy rovnomerne v mezich mapy
xs=lim(1)+(lim(3)-lim(1))*rand(N,1);
ys=lim(2)+(lim(4)-lim(2))*rand(N,1);
thetas=-pi+2*pi*rand(N,1);

public_vars.particles=[xs, ys, thetas];
end