% HARP_DYNAMIC_MODEL Run the high-fidelity two-stage parachute trajectory model.
%   Builds a parachute-payload system from HARP outputs, loads atmospheric
%   weather data, and propagates the 26-state ENU dynamics model from
%   release until ground impact. Returns trajectory and landing data.
%
%   This function serves as a bridge between the HARP estimator outputs and
%   the high-fidelity propagator. It uses a two-stage parachute model: a
%   small drogue (2.5 m, 0.1 s) followed by the main canopy (22.5 m, 35 s).
%
% INPUTS (name-value pairs via MATLAB arguments block):
%   drop_altitude          : Drop altitude AGL (ft), default 400
%   terrain_elevation      : Terrain elevation MSL (ft), default 560
%   indicated_airspeed     : Aircraft IAS (knots), default 130
%   ballistic_wind         : Ballistic wind speed (knots), default 10
%   drop_altitude_wind     : Wind speed at drop altitude (knots), default 15
%   dz_altimeter           : DZ altimeter setting (inHg), default 29.85
%   surface_temperature    : Surface temperature (°C), default 20
%   true_altitude_temperature : Temperature at altitude (°C), default 15
%   DZ_course              : Drop zone course (degrees), default 302
%   num_parachutes         : Number of parachutes, default 1
%   parachute_deploy_time  : Parachute deployment time (s), default 0
%   tspan                  : Time vector for propagation, default linspace(0,90,2000)
%   visualize              : Plot results flag, default true
%   w                      : Payload width (inches), default 48
%   l                      : Payload length (inches), default 83
%   h                      : Payload height (inches), default 43
%   m                      : Payload mass (lbs), default 500
%   inputs                 : HARP inputs struct (from convertDataOutToInputs)
%   data_out               : Raw flight data struct (from get_flight_estimates)
%   carp_data              : CARP sub-struct (from data_out.carp)
%
% OUTPUTS:
%   results : Struct containing propagator results (see Harp_Dynamic_Model_Results):
%               - t_plot      : Full time vector (s)
%               - y_sim       : Full state matrix (Nx26)
%               - time        : Time vector to ground impact (s)
%               - trajectory  : ENU position to ground impact (Mx3, m)
%               - landing_time           : Time of ground impact (s)
%               - east_displacement      : East landing offset (m)
%               - north_displacement     : North landing offset (m)
%               - total_horizontal_displacement : Horizontal range (m)

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
        data_out.inputs = []
        data_out.data_out = []
        data_out.carp_data = []
    end
    %% STEP 2: Create Parachute System
    parachute_system1 = Create_Parachute(2.5, 0.1);

    parachute_system2 = Create_Parachute(22.5, 35);

    %% STEP 3: Create Payload System
    payload = Create_Payload(data_out.w, data_out.l, data_out.h,data_out.m);
    
    %% STEP 4: Convert CARP Results to Propagator Initial Conditions
    x0 = HARP_To_Propagator(data_out.carp_data);

    [~, the_weather] = load_weather(data_out.carp_data.time_UTC);
    the_weather.win_speed = ks2mps(the_weather.win_speed);
    the_weather.alt_agl = ft2m(1000 * the_weather.alt_agl);
    the_weather.alt_agl(1) = 0;
    
    %% STEP 5: Run High-Fidelity Propagator 
    [t, y] = propagate_model(...
        'x0', x0, ...
        'tspan', data_out.tspan, ...
        'parachute', parachute_system1, ...
        'parachute2', parachute_system2, ...
        'payload', payload, ...
        'weather', the_weather, ...
        'model', @Two_Stage_Model);

 results = Harp_Dynamic_Model_Results(t, y);

end


