function p = parameters()
%%% You can change the names as you like

 p.scenario.monte_runs = 100;
 p.scenario.num_of_time_steps = 50;

 % add anything related to the scenario
 p.target(1).start_time = 0.80;
 p.target(1).start_state = [100 30 3000 20];
 p.target(1).process_noise = 0.5;

 p.target(1).velocity_x = 10;
 p.targert(1).velocity_x = 10;


 % add anything related to the target
 p.sensor(1).sampling_time = 2;

 p.sensor(1).position = [1000 500];
 p.sensor(1).velocity = [0 0];
 p.sensor(1).error_range = 10; %meter
 p.sensor(1).error_azimuth = 0.01; %rad
 p.sensor(1).Pd = 0.9;
 p.sensor(1).false_alarm_density = 1e-4;
 p.sensor(1).coverage_range_LB = 0; %meter
 p.sensor(1).coverage_range_UB = 10000; %meter
 p.sensor(1).coverage_azimuth_LB = -pi;
 p.sensor(1).coverage_azimuth_UB = pi;


 % add anything related to the the sensor
 p.tracker.gate_size = chi2inv(0.99,2);
 p.tracker.init_cov = diag([100^2 10^2 100^2 10^2]);

 % add anything related to the tracker

 p.perf_eval.gate_size = 150;
 % add anything related to performance evaluation
 T = p.sensor(1).sampling_time;
 %add additional parameters
 p.F =  [1, T ,0 ,0 ;
         0, 1 ,0 ,0 ;
         0, 0 ,1 ,T ;
         0, 0, 0, 1];

 p.G = [T^2/2, 0;
         T,     0;
         0, T^2/2;
         0,    T];

end