function xk = moveTarget(parameter, xk_1) % add additional parameters, if necessary
    
    F = parameter.F;
    G = parameter.G;

    V = parameter.target.process_noise* randn(2,1);
    xk = F * xk_1 + G * V; 
end