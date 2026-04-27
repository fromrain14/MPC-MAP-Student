function [path] = plan_path(read_only_vars, public_vars)

% Planovani se spusti jen pokud cesta neexistuje a GNSS je pripraveno
planning_required = isempty(public_vars.path) && public_vars.gnss_init_done;

% Priprava mapy a rozliseni
if planning_required
    map = read_only_vars.discrete_map.map';
    res = read_only_vars.map.discretization_step;

    % Definice bezpecne vzdalenosti
    clearance_m = 0.8;
    r_cells = ceil(clearance_m/res);

    % Kernel pro konvoluci
    [kx,ky] = meshgrid(-r_cells:r_cells,-r_cells:r_cells);
    kernel = double(sqrt(kx.^2+ky.^2)<=r_cells);

    % Vypocet vzdalenosti od prekrazek
    obs_map = double(map~=0);
    dist_map = conv2(obs_map,kernel,'same');

    % Vytvoreni cost mapy
    n_x = read_only_vars.discrete_map.dims(1);
    n_y = read_only_vars.discrete_map.dims(2);
    cost_map = zeros(n_x,n_y);
    idx = dist_map>0 & map==0;
    cost_map(idx) = dist_map(idx)*3.0;
    public_vars.cost_map = cost_map;

    % Vypocet nejkratsi cesty
    path = astar(read_only_vars, public_vars);
    % Vyhlazeni ostre cesty
    path = smooth_path(path);
else
    path = public_vars.path;
end

end