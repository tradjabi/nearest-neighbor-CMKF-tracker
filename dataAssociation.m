function [asso_meas_id, asso_meas,z_min,Rxy_min] = dataAssociation(parameter, xk_1_hat, Pk_1_hat, measurements)

    prev_mahalanobis = Inf;
    % Extract parameters
    gate_size = parameter.tracker.gate_size;
    F = parameter.F;
    G = parameter.G;
    H = [1 0 0 0;
        0 0 1 0];

    lambada_noise = parameter.target.process_noise;
    Q = G * lambada_noise^2 * G';

    error_range =  parameter.sensor(1).error_range;
    error_azimuth =  parameter.sensor(1).error_azimuth;

    % Number of measurements
    num_measurements = size(measurements, 2);

    % Initialize association arrays
    asso_meas_id = -1;
    asso_meas = [];

    R_polar = [error_range^2, 0   ;
               0, error_azimuth^2];

    xk_hat = F * xk_1_hat;      %equation 30
    Pk = F * Pk_1_hat * F' + Q; % equation 31

    Rxy_min = []; 
    z_min = [];

    % Loop through each measurement
    for i = 1:size(measurements, 2)  %tetha
        % Convert measurement covariance from polar to Cartesian
        

        z_cartesian = poltoCart(parameter, measurements(:,i));
        z_hat = H * xk_hat; 

        R_xy = calculateCovarianceRp(R_polar, measurements);
        S = H * Pk* H'+ R_xy; % equation 34

        mahalanobis = sqrt((z_cartesian - z_hat)' * inv(S) * (z_cartesian - z_hat));

        % Check if the measurement is within the gate
        if mahalanobis <= gate_size  && prev_mahalanobis > mahalanobis
            % Associate measurement with target
            asso_meas_id = i; % For simplicity, assume only one target
            asso_meas = measurements(:, i);
            prev_mahalanobis = mahalanobis;
            Rxy_min = R_xy;
            z_min = z_cartesian;

        else
            % Measurement not associated with any target
            %asso_meas_id(i) = 0;
        end
    end
end


function z_cartesian = poltoCart(parameter, measurements)
    % Extract parameters
    sensor_position = parameter.sensor(1).position;

    % Extract polar coordinates
    range = measurements(1);
    azimuth = measurements(2);

    % Convert polar coordinates to Cartesian coordinates
    x_sensor = sensor_position(1);
    y_sensor = sensor_position(2);
    x_measurement = x_sensor + range * cos(azimuth);
    y_measurement = y_sensor + range * sin(azimuth);

    z_cartesian = [x_measurement; y_measurement];
end

function R_xy = calculateCovarianceRp(R_polar, z)
    % Extract components of polar covariance matrix
    rm = z(1);
    thetam = z(2);
    sigmar2 = R_polar(1, 1);
    sigmatheta2 = R_polar(2, 2);
    
    % Compute intermediate variables
    lambda_theta1 = exp(-sigmatheta2 / 2);
    lambda_theta2 = exp(-2 * sigmatheta2);
    theta2 = thetam^2;
    cos2_theta_m = cos(2 * thetam);
    sin2_theta_m = sin(2 * thetam);
    cos_theta_m = cos(thetam);
    sin_theta_m = sin(thetam);
    
    % Compute covariance matrix components in Cartesian coordinates
    R11_p = (lambda_theta1^(-2)  - 2) * rm^2 * cos_theta_m^2 + 0.5 * (rm^2 + sigmar2) * (1 + lambda_theta2 * cos2_theta_m);
    R22_p = (lambda_theta1^(-2) - 2) * rm^2 * sin_theta_m^2 + 0.5 * (rm^2 + sigmar2) * (1 - lambda_theta2 * cos2_theta_m);
    R12_p = (lambda_theta1^(-2) - 2) * rm^2 * cos_theta_m * sin_theta_m + 0.5 * (rm^2 + sigmar2) * lambda_theta2 * sin2_theta_m;
    
    % Construct Cartesian covariance matrix
    R_xy = [R11_p, R12_p; R12_p, R22_p];
end



