function [path] = astar(read_only_vars, public_vars)
% Inicializace a prevod souradnic na indexy gridu
path = [];
map = read_only_vars.discrete_map.map;
res = read_only_vars.map.discretization_step;
ox = read_only_vars.discrete_map.limits(1);
oy = read_only_vars.discrete_map.limits(2);
n_x = read_only_vars.discrete_map.dims(1);
n_y = read_only_vars.discrete_map.dims(2);
map = map';
sx = public_vars.estimated_pose(1);
sy = public_vars.estimated_pose(2);
sx_idx = round((sx-ox)/res)+1;
sy_idx = round((sy-oy)/res)+1;
sx_idx = max(1,min(n_x,sx_idx));
sy_idx = max(1,min(n_y,sy_idx));
gx_idx = read_only_vars.discrete_map.goal(1);
gy_idx = read_only_vars.discrete_map.goal(2);

% Kontrola zda start nebo cil nelezi v prekazce
if map(sx_idx,sy_idx)~=0 || map(gx_idx,gy_idx)~=0
    return;
end

% Priprava cost mapy pro clearance
if isfield(public_vars,'cost_map')
    cost_map = public_vars.cost_map;
else
    cost_map = zeros(n_x,n_y);
end

% Definice 8-okoli a cen pohybu
moves = [-1,-1;-1,0;-1,1;0,-1;0,1;1,-1;1,0;1,1];
move_costs = [sqrt(2);1;sqrt(2);1;1;sqrt(2);1;sqrt(2)];

% Inicializace struktur pro A*
g = inf(n_x,n_y);
g(sx_idx,sy_idx) = 0;
parent = zeros(n_x,n_y,2);
heur = @(xi,yi) sqrt((xi-gx_idx).^2+(yi-gy_idx).^2)*res;
q_f = heur(sx_idx,sy_idx);
q_x = sx_idx;
q_y = sy_idx;
state = zeros(n_x,n_y);
state(sx_idx,sy_idx) = 1;
found = false;

% Hlavni smycka prohledavani
while ~isempty(q_x)
    [~,idx] = min(q_f);
    cx = q_x(idx);
    cy = q_y(idx);
    q_f(idx) = [];
    q_x(idx) = [];
    q_y(idx) = [];

    if state(cx,cy)==2
        continue;
    end
    state(cx,cy) = 2;
    % Kontrola
    if cx==gx_idx && cy==gy_idx
        found = true;
        break;
    end

    % Expanze sousednich uzlu
    for i=1:8
        nx = cx+moves(i,1);
        ny = cy+moves(i,2);
        if nx<1||nx>n_x||ny<1||ny>n_y
            continue;
        end
        if map(nx,ny)~=0
            continue;
        end
        % Osetreni
        dx = moves(i,1);
        dy = moves(i,2);
        if abs(dx)==1 && abs(dy)==1
            if map(cx+dx,cy)~=0 || map(cx,cy+dy)~=0
                continue;
            end
        end

        % Vypocet nove ceny g(x) a aktualizace fronty
        new_g = g(cx,cy)+move_costs(i)*res+cost_map(nx,ny);
        if state(nx,ny)==0
            g(nx,ny) = new_g;
            parent(nx,ny,1) = cx;
            parent(nx,ny,2) = cy;
            state(nx,ny) = 1;
            q_f(end+1) = new_g+heur(nx,ny);
            q_x(end+1) = nx;
            q_y(end+1) = ny;
        elseif state(nx,ny)==1
            if new_g<g(nx,ny)
                g(nx,ny) = new_g;
                parent(nx,ny,1) = cx;
                parent(nx,ny,2) = cy;
                q_f(end+1) = new_g+heur(nx,ny);
                q_x(end+1) = nx;
                q_y(end+1) = ny;
            end
        end
    end
end

% Rekonstrukce nalezene cesty pomoci rodicu
if ~found
    return;
end
path_x = gx_idx;
path_y = gy_idx;
cx = gx_idx;
cy = gy_idx;
while ~(cx==sx_idx && cy==sy_idx)
    px = parent(cx,cy,1);
    py = parent(cx,cy,2);
    cx = px;
    cy = py;
    path_x(end+1) = cx;
    path_y(end+1) = cy;
end

% Prevod vysledne cesty z indexu zpet na metry
path_x = flip(path_x);
path_y = flip(path_y);
path = [(path_x'-1)*res+ox, (path_y'-1)*res+oy];
end