function [weights] = weight_particles(particle_measurements, lidar_distances)

sigma=0.3;      
N=size(particle_measurements, 1);
weights=zeros(N, 1);

real_d=lidar_distances(:)';

for i=1:N
    pred_d=particle_measurements(i,:); 
    % ignoruji se paprsky co nedosahnou
    valid=~isinf(pred_d) & ~isinf(real_d) & ~isnan(pred_d) & ~isnan(real_d);
    if sum(valid)==0
        weights(i)=1e-10;
        continue;
    end
    diff=pred_d(valid)-real_d(valid);
    % logaritmicka suma
    log_w=sum(-0.5*(diff/sigma).^2);
    weights(i)=exp(log_w);
end

% normalizace vah
s=sum(weights);
if s==0
    weights=ones(N,1)/N;
else
    weights=weights/s;
end
end