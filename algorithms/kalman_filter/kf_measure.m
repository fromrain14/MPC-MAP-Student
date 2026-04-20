function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)

% 3. Kalman gain
S=kf.C*sigma*kf.C'+kf.Q;
K=sigma*kf.C'*(S\eye(size(S)));

% 4. Aktualizace stredni hodnoty
new_mu=mu+K*(z-kf.C*mu);

% 5. Aktualizace kovariance
new_sigma=(eye(size(sigma))-K*kf.C)*sigma;
end