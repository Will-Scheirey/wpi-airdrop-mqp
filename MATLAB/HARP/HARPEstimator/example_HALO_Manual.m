function example_HALO_Manual()
    % Example from AFMAN 11-231 Figure 5.21
    
    fprintf('\n=== EXAMPLE 1: HALO ===\n');
    
    % Create data_out structure
    data_out = struct();
    
    % Mission
    data_out.mission.type = 'HALO';
    data_out.mission.method = 'crew';
    
    % Parachute
    data_out.parachute = loadParachuteDatabase('MC-4', 300);
    
    % DZ
    data_out.dz.altimeter_setting = 30.30;
    data_out.dz.pi_elevation = 250;
    data_out.dz.terrain_elevation = 250;
    data_out.dz.centerline = 180;
    data_out.dz.offset = 0;
    
    % HALO specific
    data_out.halo.actuation_altitude = 4000;
    
    % CARP
    data_out.carp.altitude = 20000;
    data_out.carp.airspeed = 130;
    data_out.carp.heading = 180;
    
    % Temperatures
    data_out.temps.drop = -24;
    data_out.temps.actuation = 7;
    data_out.temps.surface = 15;
    
    % Winds (pre-computed ballistic)
    data_out.winds.hvBallistic = [163, 44];
    data_out.winds.deployedBallistic = [165, 24];
    data_out.winds.dropAltitude = [150, 60];
    
    % Weather
    data_out.weather.d_value = 750;
    
    % Safety
    data_out.safety.factor = 2000;
    data_out.safety.percentage = 0.80;
    
    % Compute HARP
    outputs = HARPComputer.compute(data_out);
    
    % Plot
    plotHARP2(outputs, outputs.inputs);
end