function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

mu=public_vars.mu;
sigma=public_vars.sigma;


% Prevod
vR=public_vars.motion_vector(1);
vL=public_vars.motion_vector(2);
d=read_only_vars.agent_drive.interwheel_dist;
v=(vR+vL)/2;
omega=(vR-vL)/d;
u=[v; omega];

% I. Predikce
[mu,sigma]=ekf_predict(mu,sigma,u,public_vars.kf,read_only_vars.sampling_period);
% II. Korekce
z=read_only_vars.gnss_position;
if ~isempty(z) && ~any(isnan(z))
    [mu,sigma]=kf_measure(mu,sigma,z(:),public_vars.kf);
end
end