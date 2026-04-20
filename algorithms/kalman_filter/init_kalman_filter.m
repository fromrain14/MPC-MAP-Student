function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
public_vars.kf.C=[1,0,0; 0,1,0];
public_vars.kf.R=diag([0.01,0.01,0.01]);
public_vars.kf.Q=diag([0.5^2,0.5^2]);  
public_vars.mu=[0;0;0];
public_vars.sigma=diag([1e6,1e6,1e6]);
public_vars.gnss_init_data=[];
public_vars.gnss_init_done=false;
end