function example_WithWindProfile()
    % Example using complete wind profile
    
    fprintf('\n=== EXAMPLE 3: HALO with Full Wind Profile ===\n');
    
    data_out = struct();
    
    % Mission
    data_out.mission.type = 'HALO';
    
    % Parachute
    data_out.parachute = loadParachuteDatabase('MC-4', 300);
    
    % DZ
    data_out.dz.altimeter_setting = 30.10;
    data_out.dz.pi_elevation = 1000;
    data_out.dz.terrain_elevation = 1000;
    data_out.dz.centerline = 90;
    
    % HALO
    data_out.halo.actuation_altitude = 3500;
    
    % CARP
    data_out.carp.altitude = 18000;
    data_out.carp.airspeed = 130;
    data_out.carp.heading = 90;
    
    % Complete wind profile
    data_out.winds.profile = [
        18000, 100, 55, -20;
        16000, 105, 50, -16;
        14000, 110, 45, -12;
        12000, 115, 40, -8;
        10000, 120, 35, -4;
        8000,  125, 30, 0;
        6000,  130, 25, 4;
        4000,  135, 22, 8;
        3000,  138, 20, 10;
        2000,  140, 18, 12;
        1000,  142, 15, 14;
        0,     145, 12, 16
    ];
    
    % Temperatures (extracted from wind profile)
    data_out.temps.drop = -20;
    data_out.temps.actuation = 8;
    data_out.temps.surface = 16;
    
    % Weather
    data_out.weather.d_value = 600;
    
    % Safety
    data_out.safety.factor = 2000;
    data_out.safety.percentage = 0.80;
    
    % Compute HARP
    outputs = HARPComputer.compute(data_out);
    
    % Plot
    plotHARP(outputs, outputs.inputs);
end