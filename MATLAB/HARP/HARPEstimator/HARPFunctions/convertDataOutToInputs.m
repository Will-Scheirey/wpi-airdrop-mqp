function inputs = convertDataOutToInputs(data_out)
            % Convert data_out structure to HARP inputs format
           
            data_out.mission.type = 'HALO';
            data_out.mission.method = 'crew';
            data_out.Parachute_weight = data_out.system_data.total_weight - data_out.system_data.payload_weight;
            data_out.parachute = loadParachuteDatabase('G-15', data_out.Parachute_weight); 
            data_out.dz.altimeter_setting = 29.92; %Not given in Data, this is the typical value
            data_out.dz.pi_elevation = data_out.system_data.dz_alt;
            data_out.halo.actuation_altitude = data_out.system_data.planned_activation;
            
            inputs = struct();
           
            
            %% MISSION CONFIGURATION
            if isfield(data_out, 'mission') && isfield(data_out.mission, 'type')
                inputs.mission.type = data_out.mission.type;
            else
                error('Mission type (HALO/HAHO) must be specified in data_out.mission.type');
            end
            
            if isfield(data_out, 'mission') && isfield(data_out.mission, 'method')
                inputs.mission.method = data_out.mission.method;
            else
                inputs.mission.method = 'crew';
            end
            
            %% PARACHUTE DATA
            
            inputs.parachute = data_out.parachute;
                
               
            
            %% ALTITUDES
            inputs.altitude = struct();
            
            % Drop altitude
            inputs.altitude.dropIndicatedTrue = data_out.carp.altitude;
            
            
            % DZ altimeter setting
            
            inputs.altitude.dzAltimeter = data_out.dz.altimeter_setting;
           
            
            % PI elevation
            
            inputs.altitude.piElevation = data_out.dz.pi_elevation;
            
            
            % Terrain elevation
            
            if isfield(data_out, 'dz') && isfield(data_out.dz, 'terrain_elevation')
                inputs.altitude.dzTerrain = data_out.dz.terrain_elevation;
            else
                inputs.altitude.dzTerrain = inputs.altitude.piElevation;
                warning('DZ terrain elevation not specified, using PI elevation');
            end
            
            % D-value
            if isfield(data_out, 'weather') && isfield(data_out.weather, 'd_value')
                inputs.altitude.dValue = data_out.weather.d_value;
            else
                inputs.altitude.dValue = 0;
                warning('D-value not specified, using 0');
            end
            
            % Actuation altitude (HALO only)
            
                inputs.altitude.actuationAGL = data_out.halo.actuation_altitude;
                
            
            %% TEMPERATURES
            inputs.temps = struct();
            
            % if isfield(data_out, 'temps')
                inputs.temps.drop = data_out.carp.drop_temp;
                inputs.temps.surface = data_out.weather.temperature(1); %data_out.temps.surface;
                if strcmp(inputs.mission.type, 'HALO')
                    inputs.temps.actuation = data_out.carp.activation_temp; %data_out.temps.actuation;
                else
                    error('Temperature data required in data_out.temps');
                end
            % end
            
            %% WINDS
            inputs.winds = struct();
            
            if isfield(data_out, 'winds')
                if isfield(data_out.winds, 'profile')
                    % Full wind profile provided
                    inputs.winds.profile = data_out.winds.profile;
                elseif isfield(data_out.winds, 'hvBallistic') && ...
                       isfield(data_out.winds, 'deployedBallistic') && ...
                       isfield(data_out.winds, 'dropAltitude')
                    % Pre-computed ballistic winds
                    inputs.winds.hvBallistic = data_out.winds.hvBallistic;
                    inputs.winds.deployedBallistic = data_out.winds.deployedBallistic;
                    inputs.winds.dropAltitude = data_out.winds.dropAltitude;
                else
                    error('Wind data incomplete - need profile or ballistic winds');
                end
            elseif isfield(data_out, 'weather') && isfield(data_out.weather, 'profile')
                inputs.winds.profile = data_out.weather.profile;
            else
                error('Wind data required in data_out.winds');
            end
            
            %% AIRCRAFT
            inputs.aircraft = struct();
            
            if isfield(data_out, 'carp')
                if isfield(data_out.carp, 'airspeed')
                    inputs.aircraft.airspeed = data_out.carp.airspeed;
                else
                    error('Aircraft airspeed required in data_out.carp.airspeed');
                end
                
                if isfield(data_out.carp, 'heading')
                    inputs.aircraft.magneticCourse = data_out.carp.heading;
                elseif isfield(data_out, 'dz') && isfield(data_out.dz, 'centerline')
                    inputs.aircraft.magneticCourse = data_out.dz.centerline;
                else
                    error('Aircraft heading or DZ centerline required');
                end
            else
                error('Aircraft data required in data_out.carp');
            end
            
            %% SAFETY FACTORS
            inputs.safety = struct();
            
            if isfield(data_out, 'safety')
                if isfield(data_out.safety, 'factor')
                    inputs.safety.factor = data_out.safety.factor;
                else
                    inputs.safety.factor = 2000;
                    warning('Safety factor not specified, using default 2000 ft');
                end
                
                if isfield(data_out.safety, 'percentage')
                    inputs.safety.percentage = data_out.safety.percentage;
                else
                    inputs.safety.percentage = 0.80;
                    warning('Safety percentage not specified, using default 80%%');
                end
            else
                inputs.safety.factor = 2000;
                inputs.safety.percentage = 0.80;
                warning('Safety parameters not specified, using defaults');
            end
            
            %% DZ INFORMATION
            inputs.dz = struct();
            
            if isfield(data_out, 'dz')
                if isfield(data_out.dz, 'centerline')
                    inputs.dz.centerline = data_out.dz.centerline;
                else
                    inputs.dz.centerline = inputs.aircraft.magneticCourse;
                end
                
                if isfield(data_out.dz, 'offset')
                    inputs.dz.offset = data_out.dz.offset;
                else
                    inputs.dz.offset = 0;
                end
            else
                inputs.dz.centerline = inputs.aircraft.magneticCourse;
                inputs.dz.offset = 0;
            end
        end