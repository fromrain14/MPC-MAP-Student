function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)

d=read_only_vars.agent_drive.interwheel_dist; % roztec kol
dt=read_only_vars.sampling_period;            % casovy krok
vR=motion_vector(1);
vL=motion_vector(2);

% linearni a uhlova rychlost z rychlosti kol
v=(vR+vL)/2;
omega=(vR-vL)/d;

px=old_pose(1);
py=old_pose(2);
ptheta=old_pose(3);

% integrace polohy
new_x=px+v*cos(ptheta)*dt;
new_y=py+v*sin(ptheta)*dt;
new_theta=ptheta+omega*dt;

% Gaussovsky sum
sigma_xy=0.05;   % sum polohy
sigma_th=0.03;   % sum orientace
new_x=new_x+sigma_xy*randn();
new_y=new_y+sigma_xy*randn();
new_theta=new_theta+sigma_th*randn();

% normalizace uhlu do (-pi, pi)
new_theta=atan2(sin(new_theta), cos(new_theta));

new_pose=[new_x, new_y, new_theta];
end