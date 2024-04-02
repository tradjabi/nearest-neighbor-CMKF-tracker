function Z = generate_measurements(parameter, target_state) % add additional parameters, if necessary
    %% Extract essential parameters
    Pd = parameter.sensor(1).Pd;
    velocity = parameter.sensor(1).velocity;
    coverage_range_LB = parameter.sensor(1).coverage_range_LB;
    coverage_range_UB = parameter.sensor(1).coverage_range_UB;
    coverage_azimuth_LB = parameter.sensor(1).coverage_azimuth_LB;
    coverage_azimuth_UB = parameter.sensor(1).coverage_azimuth_UB;
    false_alarm_density = parameter.sensor(1).false_alarm_density;
    sensor_position = parameter.sensor(1).position;
    error_range = parameter.sensor(1).error_range;
    error_azimuth = parameter.sensor(1).error_azimuth;

    % Calculate the volume of coverage
    coverage_range = coverage_range_UB - coverage_range_LB;
    coverage_azimuth = coverage_azimuth_UB - coverage_azimuth_LB;
    coverage_volume = coverage_range * coverage_azimuth;

    % Generate measurements from targets with given Pd and measurement noise
    Z = [];

    % Compute range and azimuth from sensor to target
    % target_position: [x, y] and Unpack positions
    xs = sensor_position(1);
    ys = sensor_position(2);

    x = target_state(1); %%%%%%
    y = target_state(3);

    if (rand < Pd)
        % Generate measurement noise
        % Compute range and Compute azimuth
        range = sqrt((x - xs)^2 + (y - ys)^2) ;
        azimuth = atan2(y - ys, x - xs);

        range_measurement = range + error_range * randn;         %x
        azimuth_measurement = azimuth + error_azimuth * randn;   %y
        measurement = [range_measurement; azimuth_measurement];


        % Check if false alarm is within coverage range
        if (range_measurement >= coverage_range_LB && range_measurement <= coverage_range_UB && ...
            azimuth_measurement >= coverage_azimuth_LB && azimuth_measurement <= coverage_azimuth_UB)
            % false_alarm = [false_alarm_range; false_alarm_azimuth];
            Z = [Z measurement];
        end
        % Z = [Z measurement];

        
    end

    % Generate false alarms
    mean_false_alarms = false_alarm_density * coverage_volume;
    num_false_alarms = poissrnd(mean_false_alarms);

    for i = 1:num_false_alarms
        false_alarm_range = coverage_range_LB + coverage_range * rand;
        false_alarm_azimuth = coverage_azimuth_LB + coverage_azimuth * rand;

        % false_alarm = [false_alarm_range; false_alarm_azimuth];
        % Z = [Z false_alarm];
    end
end
