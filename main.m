%%% This is a sample template. You can change as you like
clear all
close all

p = parameters();
mean_rmse_speed = zeros(1, p.scenario.num_of_time_steps);
mean_rmse_position = zeros(1, p.scenario.num_of_time_steps);
num_valid_rmse = zeros(1, p.scenario.num_of_time_steps);

% Define the center and radius of the circle
center = p.sensor(1).position;
% Center coordinates (x, y)
radius =  p.sensor(1).coverage_range_UB - p.sensor(1).coverage_range_LB ;

% Set aspect ratio to be equal
axis equal;
mean_rmse_speed_v = [];
mean_rmse_position_v = [];

%%% perform any initialization

for r=1:p.scenario.monte_runs
    %%% perform any initialization this run
    Pk_hat = p.tracker.init_cov;

    truth(:,1) = p.target(1).start_state;

    xk_hat(:,1) = mvnrnd(truth(:,1), Pk_hat)';

    for k=2:p.scenario.num_of_time_steps 
        
        truth(:,k) = moveTarget(parameters , truth(:,k-1));
            
        measurements = generateMeasurements(parameters, truth(:,k));

        xk_1_hat= xk_hat(:,k-1);
        Pk_1_hat = Pk_hat;

        % .. = convertMeasurements(....);
        
        [asso_meas_id, asso_meas,asso_meas_car, R_xy] = dataAssociation(parameters, xk_1_hat, Pk_1_hat, measurements);
      
        [xk_hat(:,k), Pk_hat] = kalmanFilter(parameters, xk_1_hat, Pk_1_hat, asso_meas_car, R_xy);        

        % Calculate RMSE at each time step
        true_pos = truth([1,3],k);
        estimate_pose = xk_hat([1,3],k);
        true_speed = truth([2,4],k);
        estimate_speed = xk_hat([2,4], k);
        error_positions(r, k) = sqrt(sum((true_pos - estimate_pose).^2));
        error_speeds(r, k) = sqrt(sum((true_speed - estimate_speed).^2));
        if (error_positions(r, k) > p.perf_eval.gate_size)
            error_positions(r, k) = 0;
            error_speeds(r, k) = 0;

        else
            num_valid_rmse(k) = num_valid_rmse(k) + 1;
        end

    end
 
    plot(truth(1,:), truth(3,:) , 'r');
    hold on;
    plot(xk_hat(1,:), xk_hat(3,:), 'b');
    xlabel('X', 'FontSize', 14);
    ylabel('Y', 'FontSize', 14);

    % Calculate RMSE for this run
    rmse_speed = sqrt(sum((xk_hat([2,4],:) - truth([2,4],:)).^2));
    rmse_position = sqrt(sum((xk_hat([1,3],:) - truth([1,3],:)).^2));

    % Accumulate RMSE values for each time step
    mean_rmse_speed = mean_rmse_speed + rmse_speed.^2;
    mean_rmse_position = mean_rmse_position + rmse_position.^2;
end

mean_rmse_speed = sqrt(mean_rmse_speed/p.scenario.monte_runs);
mean_rmse_position = sqrt(mean_rmse_position/ p.scenario.monte_runs);

% Plot mean RMSE for speed
figure;

subplot(2,2,1);
plot(1:p.scenario.num_of_time_steps, mean_rmse_position, 'LineWidth', 2);
xlabel('Time Step (a)', 'FontSize', 14);
ylabel('Mean RMSE of Position', 'FontSize', 14);
%ylim([0 50])

subplot(2,2,2);
plot(1:p.scenario.num_of_time_steps, mean_rmse_speed, 'LineWidth', 2);
xlabel('Time Step (b)', 'FontSize', 14);
ylabel('Mean RMSE of Speed', 'FontSize', 14);

subplot(2,2,3);
plot(1:p.scenario.num_of_time_steps, sqrt(sum(error_positions.^2./num_valid_rmse, 1)), 'LineWidth', 2);
xlabel('Time Step (c)', 'FontSize', 14);
ylabel('Valid Mean RMSE of Position', 'FontSize', 14);
%ylim([0 50])

subplot(2,2,4);
plot(1:p.scenario.num_of_time_steps, sqrt(sum((error_speeds).^2./num_valid_rmse, 1)), 'LineWidth', 2);
xlabel('Time Step (d)', 'FontSize', 14);
ylabel('Valid Mean RMSE of Speed', 'FontSize', 14);



% figure()
% plot(num_valid_rmse);
