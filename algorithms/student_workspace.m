function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
public_vars.is_gnss_available = ~any(isnan(read_only_vars.gnss_position));

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    
    % Pokud robot začíná v indooru 
    if ~public_vars.is_gnss_available
        public_vars.pf_enabled = true;
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.pf_warmup = 50;
    else
        
        public_vars.pf_enabled = false;
        public_vars.particles = [];
    end
    
    public_vars.was_gnss_available = public_vars.is_gnss_available;
end


if public_vars.was_gnss_available && ~public_vars.is_gnss_available
    % Přechod Outdoor -> Indoor: Zapnout PF a inicializovat kolem poslední známé pozice
    public_vars.pf_enabled = true;
    
 
    public_vars.init_around_pose = public_vars.estimated_pose; 
    

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.pf_warmup = 30;
elseif ~public_vars.was_gnss_available && public_vars.is_gnss_available
    % Přechod Indoor -> Outdoor: Vypnout PF
    public_vars.pf_enabled = false;
    public_vars.particles = []; 
end

% Aktualizace historie stavu GNSS pro další krok
public_vars.was_gnss_available = public_vars.is_gnss_available;

% Task 1: Sber GNSS dat
if ~public_vars.gnss_init_done
    if ~any(isnan(read_only_vars.gnss_position))
        public_vars.gnss_init_data=[public_vars.gnss_init_data; read_only_vars.gnss_position(:)'];
    end
    if size(public_vars.gnss_init_data,1)>=20
        gnss_mean=mean(public_vars.gnss_init_data);
        gnss_cov=cov(public_vars.gnss_init_data) *10;
        public_vars.mu=[gnss_mean(1); gnss_mean(2); 0];
        public_vars.sigma=diag([gnss_cov(1,1), gnss_cov(2,2), 1e6]);
        public_vars.kf.Q=gnss_cov;
        public_vars.gnss_init_done=true;
    end
    % pokud neni GNSS dostupne, preskoc inicializaci
    if all(isnan(read_only_vars.gnss_position))
        public_vars.gnss_init_done=true;
    end
end

% 9. Update particle filter
if public_vars.pf_enabled
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)


if ~isempty(public_vars.particles)
    std_x = std(public_vars.particles(:, 1));
    std_y = std(public_vars.particles(:, 2));
    
    
    pf_converged = (std_x < 0.5) && (std_y < 0.5);
else
    pf_converged = false;
end

% Detekce špatně konvergovaného PF
if isfield(public_vars, 'pf_warmup') && public_vars.pf_warmup > 0
    public_vars.pf_warmup = public_vars.pf_warmup - 1;
elseif pf_converged && ~public_vars.is_gnss_available
    predicted_lidar = compute_lidar_measurement( ...
        read_only_vars.map, ...
        public_vars.estimated_pose, ...
        read_only_vars.lidar_config);
    
    real_lidar = read_only_vars.lidar_distances;
    valid = ~isinf(predicted_lidar) & ~isinf(real_lidar) ...
          & ~isnan(predicted_lidar) & ~isnan(real_lidar);
    
    if sum(valid) > 0
        lidar_error = mean(abs(predicted_lidar(valid) - real_lidar(valid)));
    else
        lidar_error = inf;
    end
    
    if lidar_error > 1.0
        pf_converged = false;
        public_vars.path = [];
        public_vars.pp_path = [];
        public_vars.pp_wp_idx = 1;
        public_vars.pp_done = false;

        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.pf_warmup = 50;
    end
end

%Detekce falešného cíle 
if isfield(public_vars, 'pp_done') && public_vars.pp_done    
    pf_converged = false;
    
    % Smazání paměti staré trasy
    public_vars.path = [];
    public_vars.pp_path = [];
    public_vars.pp_wp_idx = 1;
    public_vars.pp_done = false;

    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.pf_warmup = 50;
end

% 12. & 13. Path planning a Motion control
if public_vars.is_gnss_available || pf_converged
    public_vars.path = plan_path(read_only_vars, public_vars);
    public_vars = plan_motion(read_only_vars, public_vars);

    lidar = read_only_vars.lidar_distances;
    lidar(isnan(lidar)) = 10;
    
    front       = lidar(1);
    left_front  = lidar(2);
    right_front = lidar(8);
    
    EMERGENCY_DIST = 0.25; % [m] - nouzová vzdálenost
    SLOW_DIST      = 0.5;  % [m] - začátek zpomalování
    
    min_front = min([front, left_front, right_front]);
    
    if min_front < EMERGENCY_DIST
        % Kritická vzdálenost: okamžité otočení
        if left_front <= right_front
            public_vars.motion_vector = [-0.4, 0.4]; % otočení doprava
        else
            public_vars.motion_vector = [0.4, -0.4]; % otočení doleva
        end
    elseif min_front < SLOW_DIST
        % Zpomalovací zóna
        scale = (min_front - EMERGENCY_DIST) / (SLOW_DIST - EMERGENCY_DIST);
        scale = max(0.2, scale); 
        public_vars.motion_vector = public_vars.motion_vector * scale;
    end

else
    public_vars.path = [];
    
    lidar = read_only_vars.lidar_distances;
    lidar(isnan(lidar)) = 10; 
    
    front       = lidar(1);
    left_front  = lidar(2);
    right_front = lidar(8);
    left_side   = lidar(3);
    right_side  = lidar(7);
    
    if min([front, left_front, right_front]) < 0.8
        if left_side < right_side || left_front < right_front
            public_vars.motion_vector = [-0.5, 0.5]; 
        else
            public_vars.motion_vector = [0.5, -0.5]; 
        end
    elseif left_side < 0.6
        public_vars.motion_vector = [0.4, 0.8]; 
    elseif right_side < 0.6
        public_vars.motion_vector = [0.8, 0.4]; 
    else

        public_vars.motion_vector = [0.8, 0.8]; 

    end
end
end