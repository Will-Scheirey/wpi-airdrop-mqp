function example_HAHO()
    % Example HAHO mission
    
    fprintf('\n=== EXAMPLE 2: HAHO Mission ===\n');
    
    data_out = struct();
    
    % Mission
    data_out.mission.type = 'HAHO';
    
    % Parachute
    data_out.parachute = loadParachuteDatabase('MC-5', 350);
    
    % DZ
    data_out.dz.altimeter_setting = 29.92;
    data_out.dz.pi_elevation = 500;
    data_out.dz.terrain_elevation = 500;
    data_out.dz.centerline = 270;
    data_out.dz.offset = 0;
    
    % CARP
    data_out.carp.altitude = 25000;
    data_out.carp.airspeed = 140;
    data_out.carp.heading = 270;
    
    % Temperatures
    data_out.temps.drop = -30;
    data_out.temps.actuation = -30;  % Immediate deployment
    data_out.temps.surface = 20;
    
    % Winds (using wind profile)
    data_out.winds.profile = [
        25000, 280, 45, -30;
        20000, 275, 40, -25;
        15000, 270, 35, -15;
        10000, 265, 30, -5;
        5000,  260, 25, 5;
        0,     255, 20, 20
    ];
    
    % Weather
    data_out.weather.d_value = 500;
    
    % Safety
    data_out.safety.factor = 3000;  % Higher for HAHO
    data_out.safety.percentage = 0.90;
    
    % Compute HARP
    outputs = HARPComputer.compute(data_out);
    
    % Plot
    plotHARP(outputs, outputs.inputs);
end