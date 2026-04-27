function [estimated_pose] = estimate_pose(public_vars)
  
    if isfield(public_vars, 'is_gnss_available') && public_vars.is_gnss_available
        estimated_pose = public_vars.mu';
        
    
    elseif isfield(public_vars, 'particles') && ~isempty(public_vars.particles)
        est_x = mean(public_vars.particles(:, 1));
        est_y = mean(public_vars.particles(:, 2));
        
        
        sin_theta = mean(sin(public_vars.particles(:, 3)));
        cos_theta = mean(cos(public_vars.particles(:, 3)));
        est_theta = atan2(sin_theta, cos_theta);
        
        estimated_pose = [est_x, est_y, est_theta];
        
    
    else
        estimated_pose = public_vars.mu';
    end
end