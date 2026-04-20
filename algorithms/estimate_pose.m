function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Vrati EKF odhad polohy
estimated_pose=public_vars.mu';
end