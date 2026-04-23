% CARP_ESTIMATOR Run the full CARP estimation pipeline with high-fidelity propagation.
%   Accepts flight data via a name-value argument struct, builds a two-stage
%   parachute-payload system, converts CARP outputs to ENU initial conditions,
%   loads weather data, propagates the dynamics model, and returns trajectory
%   and landing results.
%
%   Stage 1 parachute: small drogue (5 m, 2 s deployment, cut at 38 s)
%   Stage 2 parachute: main canopy (41 m, using single_radius = 7.4676 m)
%
% INPUTS (name-value pairs via MATLAB arguments block):
%   drop_altitude          : Drop altitude AGL (ft), default 400
%   terrain_elevation      : Terrain elevation MSL (ft), default 560
%   indicated_airspeed     : Aircraft IAS (kts), default 130
%   ballistic_wind         : Ballistic wind speed (kts), default 10
%   drop_altitude_wind     : Wind speed at drop altitude (kts), default 15
%   dz_altimeter           : DZ altimeter setting (inHg), default 29.85
%   surface_temperature    : Surface temperature (°C), default 20
%   true_altitude_temperature : Temperature at altitude (°C), default 15
%   DZ_course              : DZ centerline course (°), default 302
%   num_parachutes         : Number of parachutes, default 1
%   parachute_deploy_time  : Parachute deployment time (s), default 0
%   tspan                  : Time vector for propagation, default linspace(0,90,2000)
%   visualize              : Plot results flag, default true
%   w                      : Payload width (inches), default 48
%   l                      : Payload length (inches), default 83
%   h                      : Payload height (inches), default 43
%   m                      : Payload mass (lbs), default 500
%   carp_data              : CARP data struct from get_flight_estimates (required)
%
% OUTPUTS:
%   results : Struct from Results(), containing propagator trajectory,
%             landing displacement, descent rate analysis, and drag diagnostics
%
% NOTES:
%   - Step 5 label appears twice in the original code (both the propagator
%     call and the Results extraction are labelled "STEP 5"). The second
%     should be "STEP 6".
%   - The commented-out line `x0(4:6) = x0(4:6) * 2` scales initial
%     horizontal velocity — left in place but disabled.
%   - single_radius = 7.4676 m corresponds to a 64 ft diameter canopy / 2.

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
        NameValueArgs.m (1,1) double = 500 %mass of payload in lbs
        NameValueArgs.carp_data = []
    end

    %% STEP 1: Run CARP Calculator (Mission Planning Tool)
    carp_results = Carp_Calculator2(NameValueArgs.carp_data);

    single_radius = 7.4676;      % m (64 ft diameter / 2)
    
    %% STEP 2: Create Parachute System
    parachute_system1 = Create_Parachute(...
        NameValueArgs.num_parachutes, 5, 2);
    parachute_system1.t_cut = 38;

    parachute_system2 = Create_Parachute(...
    NameValueArgs.num_parachutes, 41, single_radius);

    %%STEP 3: Create Payload System
    payload = Create_Payload(NameValueArgs.w, NameValueArgs.l, NameValueArgs.h,NameValueArgs.m);
    
    %% STEP 4: Convert CARP Results to Propagator Initial Conditions
    x0 = Carp_To_Propagator(carp_results, NameValueArgs.carp_data);

    % x0(4:6) = x0(4:6) * 2;

    [~, the_weather] = load_weather_data(NameValueArgs.carp_data.time_UTC);
    the_weather.win_speed = ks2mps(the_weather.win_speed);
    the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
    the_weather.alt_agl(1) = 0;
    
    %% STEP 5: Run High-Fidelity Propagator 
    [t, y, model_obj] = propagate_model(...
        'x0', x0, ...
        'tspan', NameValueArgs.tspan, ...
        'parachute', parachute_system1, ...
        'parachute2', parachute_system2, ...
        'payload', payload, ...
        'weather', the_weather, ...
        'model', @Two_Stage_Model);

    %% STEP 5: Extract Final State 
    results = Results(carp_results, t, y, model_obj, NameValueArgs.num_parachutes);
end


