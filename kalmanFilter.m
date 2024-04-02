function [xk_hat, Pk_hat] = kalmanFilter(parameter, xk_1_hat, Pk_1_hat, z, R_xy) % add additional parameters, if necessary
    % Kalman Filter Implementation
    
    % Extract parameters
    F = parameter.F;
    G = parameter.G;
      H = [1 0 0 0;
        0 0 1 0];
    lambada_noise = parameter.target.process_noise;
    Q = G * lambada_noise^2 * G';

    % Prediction Step
    xk_hat = F * xk_1_hat; % Equation (30)
    Pk_hat = F * Pk_1_hat * F' + Q; % Equation (31)

    if ~isempty(z)
        % Update Step
        z_hat = H * xk_hat;         % Equation (32)
        nu = z - z_hat;             % Equation (33)
        S = H * Pk_hat * H' + R_xy; % Equation (34)
        W = Pk_hat * H' * inv(S);   % Equation (37)
    
        xk_hat = xk_hat + W * nu;   % Equation (35)
        Pk_hat = Pk_hat - W * S * W'; % Equation (36)
    else
    end

end

