function results = Carp_Estimator(NameValueArgs)
    arguments
        NameValueArgs.drop_altitude (1,1) double = 400      % ft AGL
        NameValueArgs.terrain_elevation (1,1) double = 560  % ft
        NameValueArgs.indicated_airspeed (1,1) double = 130 % knots
        NameValueArgs.ballistic_wind (1,1) double = 10      % knots
        NameValueArgs.drop_altitude_wind (1,1) double = 15  % knots
        NameValueArgs.dz_altimeter (1,1) double = 29.85     % inHg 
        NameValueArgs.surface_temperature (1,1) double = 20 % Celsius
        NameValueArgs.true_altitude_temperature (1,1) double = 15 % Celsius
        NameValueArgs.DZ_course (1,1) double = 302           % degrees
        NameValueArgs.num_parachutes (1,1) double = 1       % Number of parachutes
        NameValueArgs.parachute_deploy_time (1,1) double = 0  % Parachute deployment time
        NameValueArgs.tspan = linspace(0, 90, 2000)
        NameValueArgs.visualize (1,1) logical = true
        NameValueArgs.w (1,1) double = 48 %dimensions of payload in inches
        NameValueArgs.l (1,1) double = 83
        NameValueArgs.h (1,1) double = 43
        NameValueArgs.m (1,1) double = 1200 %mass of payload in lbs
        NameValueArgs.carp_data = []
    end

    %% STEP 1: Run CARP Calculator (Mission Planning Tool)
    carp_results = Carp_Calculator(NameValueArgs);

    single_radius = 9.7536;      % m (64 ft diameter / 2)
    
    %% STEP 2: Create Parachute System
    parachute_system1 = Create_Parachute(...
        NameValueArgs.num_parachutes, 1.3, 2);

    parachute_system2 = Create_Parachute(...
    NameValueArgs.num_parachutes, 30, single_radius);

    %%STEP 3: Create Payload System
    payload = Create_Payload(NameValueArgs.w, NameValueArgs.l, NameValueArgs.h,NameValueArgs.m);
    
    %% STEP 4: Convert CARP Results to Propagator Initial Conditions
    x0 = Carp_To_Propagator(carp_results, NameValueArgs.carp_data);

    [~, the_weather] = load_weather(NameValueArgs.carp_data.time_UTC);
    the_weather.win_speed = ks2mps(the_weather.win_speed);
    the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
    the_weather.alt_agl(1) = 0;
    
    %% STEP 5: Run High-Fidelity Propagator 
    [t, y, model_obj] = propagate_model(...
        'x0', x0, ...
        'tspan', NameValueArgs.tspan, ...
        'parachute', parachute_system1, ...
        'parachute2', parachute_system2, ...
        'payload', payload, 'weather', the_weather, 'model', @Two_Stage_Model);

    %% STEP 5: Extract Final State 
    results = Results(carp_results, t, y, model_obj, NameValueArgs.num_parachutes);

end


