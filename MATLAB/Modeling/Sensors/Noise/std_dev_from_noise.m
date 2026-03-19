function std_dev = std_dev_from_noise(noise_density, hz)
    % STD_DEV_FROM_NOISE Calculates standard deviation from noise density
    % 
    % INPUTS:
    %   noise_density : Noise density in units / [sqrt(Hz)]
    %   hz            : Polling frequency
    %
    % OUTPUTS:
    %   std_dev : Calculated standard deviation
    std_dev = noise_density * sqrt(hz);
end