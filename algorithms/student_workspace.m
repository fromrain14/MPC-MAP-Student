function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
   public_vars = init_kalman_filter(read_only_vars, public_vars);

end


% Task 1: Sber GNSS dat
if ~public_vars.gnss_init_done
    if ~any(isnan(read_only_vars.gnss_position))
        public_vars.gnss_init_data=[public_vars.gnss_init_data; read_only_vars.gnss_position(:)'];
    end
    if size(public_vars.gnss_init_data,1)>=100
        gnss_mean=mean(public_vars.gnss_init_data);
        gnss_cov=cov(public_vars.gnss_init_data);
        % Task 1: vypis vysledku
        fprintf('GNSS mean: x=%.4f, y=%.4f\n', gnss_mean(1), gnss_mean(2));
        fprintf('GNSS kovariancni matice:\n'); disp(gnss_cov);
        % Task 4: nastav pocatecni belief z GNSS dat
        public_vars.mu=[gnss_mean(1); gnss_mean(2); 0];
        public_vars.sigma=diag([gnss_cov(1,1), gnss_cov(2,2), 1e6]);
        % Task 1: nastav Q matici
        public_vars.kf.Q=gnss_cov;
        public_vars.gnss_init_done=true;
    end
end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);
%fprintf('mu po PF: size=%dx%d\n', size(public_vars.mu,1), size(public_vars.mu,2));
% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);
% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

% % week2, task 2,3,4
% counter = read_only_vars.counter;
% 
% if ~public_vars.data_collected && counter <= 200
%     public_vars.lidar_data(counter, :) = read_only_vars.lidar_distances';
%     public_vars.gnss_data(counter, :)  = read_only_vars.gnss_position';
% end
% 
% if ~public_vars.data_collected && counter == 200
%     public_vars.data_collected = true;
% 
% lidar = public_vars.lidar_data;
% gnss  = public_vars.gnss_data;
%  % task2
% sigma_lidar = std(lidar);   
% sigma_gnss  = std(gnss);    
% public_vars.sigma_lidar = sigma_lidar;
% public_vars.sigma_gnss  = sigma_gnss;
% fprintf('TASK 2: Směrodatné odchylky\n');
%  for i = 1:8
%      if isnan(sigma_lidar(i))
%          fprintf('  LiDAR kanál %d: paprsek nedosáhl na zeď\n', i);
%      else
%          fprintf('  LiDAR kanál %d: sigma = %.5f m\n', i, sigma_lidar(i));
%      end
%  end
%  fprintf('  GNSS X: sigma = %.5f m\n', sigma_gnss(1));
%  fprintf('  GNSS Y: sigma = %.5f m\n', sigma_gnss(2));
% 
% 
%  figure('Name', 'Task 2 - LiDAR histogramy');
%  plot_idx = 1;
%   for i = 1:8
%       if ~isnan(sigma_lidar(i))
%           subplot(2, 4, plot_idx);
%           histogram(lidar(:,i) - mean(lidar(:,i)), 20);
%           title(sprintf('LiDAR kanál %d', i));
%           xlabel('Odchylka [m]');
%           ylabel('Počet');
%           grid on;
%           plot_idx = plot_idx + 1;
%       end
%   end
% sgtitle('Task 2 - Šum LiDAR'); 
% figure('Name', 'Task 2 - GNSS histogramy');
% subplot(1, 2, 1);
% histogram(gnss(:,1) - mean(gnss(:,1)), 20);
% title('GNSS osa X');
% xlabel('Odchylka [m]'); ylabel('Počet'); grid on;
% subplot(1, 2, 2);
% histogram(gnss(:,2) - mean(gnss(:,2)), 20);
% title('GNSS osa Y');
% xlabel('Odchylka [m]'); ylabel('Počet'); grid on;
% sgtitle('Task 2 - Šum GNSS');
% 
%        %task 3
% lidar_filled = lidar;
%   for i = 1:8
%       if isnan(sigma_lidar(i))
%           lidar_filled(:, i) = 0;
%       end
%   end
% C_lidar = cov(lidar_filled);   
% C_gnss  = cov(gnss);          
% fprintf('TASK 3\n');
% fprintf('LiDAR\n');
% disp(C_lidar);
% disp(diag(C_lidar)');
% fprintf('Ocekavane hodnoty (sigma^2):\n');
% disp(sigma_lidar.^2);   
% fprintf('GNSS\n');
% disp(C_gnss);
% fprintf('Ocekavane hodnoty (sigma^2):\n');
% disp(sigma_gnss.^2);
% 
%     % task4
% x = linspace(-2, 2, 500);
% pdf_lidar = norm_pdf(x, 0, sigma_lidar(1));
% pdf_gnss  = norm_pdf(x, 0, sigma_gnss(1));
% figure('Name', 'Task 4 - PDF normálního rozdělení');
% plot(x, pdf_lidar, 'b-', 'LineWidth', 2); hold on;
% plot(x, pdf_gnss,  'r-', 'LineWidth', 2);
% legend('LiDAR kanál 1', 'GNSS osa X');
% xlabel('Odchylka [m]');
% ylabel('Hustota pravděpodobnosti');
% title('Task 4 - PDF šumu senzorů');
% grid on;
% end

end