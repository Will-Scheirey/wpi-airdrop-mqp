function std_dev = std_dev_from_noise(noise_density, hz)
    std_dev = noise_density * sqrt(hz);
end