function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)

v=u(1);
omega=u(2);
theta=mu(3);
dt=sampling_period;

% 1. Predikce stredni hodnoty
new_mu=mu;
new_mu(1)=mu(1)+cos(theta)*v*dt;
new_mu(2)=mu(2)+sin(theta)*v*dt;
new_mu(3)=mu(3)+omega*dt;

% Jakobianova matice G
G=[1, 0, -sin(theta)*v*dt;
   0, 1,  cos(theta)*v*dt;
   0, 0,  1];

% Aktualizace kovariance
new_sigma=G*sigma*G'+kf.R;
end