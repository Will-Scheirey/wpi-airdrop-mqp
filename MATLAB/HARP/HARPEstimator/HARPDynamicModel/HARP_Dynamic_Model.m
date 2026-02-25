function results = HARP_Dynamic_Model(data_out)
    arguments
        data_out.drop_altitude (1,1) double = 400      % ft AGL
        data_out.terrain_elevation (1,1) double = 560  % ft
        data_out.indicated_airspeed (1,1) double = 130 % knots
        data_out.ballistic_wind (1,1) double = 10      % knots
        data_out.drop_altitude_wind (1,1) double = 15  % knots
        data_out.dz_altimeter (1,1) double = 29.85     % inHg 
        data_out.surface_temperature (1,1) double = 20 % Celsius
        data_out.true_altitude_temperature (1,1) double = 15 % Celsius
        data_out.DZ_course (1,1) double = 302           % degrees
        data_out.num_parachutes (1,1) double = 1       % Number of parachutes
        data_out.parachute_deploy_time (1,1) double = 0  % Parachute deployment time
        data_out.tspan = linspace(0, 90, 2000)
        data_out.visualize (1,1) logical = true
        data_out.w (1,1) double = 48 %dimensions of payload in inches
        data_out.l (1,1) double = 83
        data_out.h (1,1) double = 43
        data_out.m (1,1) double = 500 %mass of payload in lbs
        data_out.carp_data = []
    end
    %% STEP 2: Create Parachute System
    parachute_system1 = Create_Parachute(...
        data_out.num_parachutes, 5, 2);
    parachute_system1.t_cut = 38;

    parachute_system2 = Create_Parachute(...
    data_out.num_parachutes, 41, single_radius);

    %% STEP 3: Create Payload System
    payload = Create_Payload(data_out.w, data_out.l, data_out.h,data_out.m);
    
    %% STEP 4: Convert CARP Results to Propagator Initial Conditions
    x0 = HARP_To_Propagator(data_out);

    % x0(4:6) = x0(4:6) * 2;

    [~, the_weather] = load_weather(data_out.carp_data.time_UTC);
    the_weather.win_speed = ks2mps(the_weather.win_speed);
    the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
    the_weather.alt_agl(1) = 0;
    
    %% STEP 5: Run High-Fidelity Propagator 
    [t, y, model_obj] = propagate_model(...
        'x0', x0, ...
        'tspan', data_out.tspan, ...
        'parachute', parachute_system1, ...
        'parachute2', parachute_system2, ...
        'payload', payload, ...
        'weather', the_weather, ...
        'model', @Two_Stage_Model);

    %% STEP 5: Extract Final State 
    results = Results(t, y, model_obj, data_out.num_parachutes);
end


