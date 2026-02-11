function inputs = convertDataOutToInputs(data_out)
            % Convert data_out structure to HARP inputs format
            
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
            if isfield(data_out, 'parachute')
                inputs.parachute = data_out.parachute;
                
                % Validate required fields
                required = {'type', 'weight', 'hvRoF', 'deployedRoF', 'vd', ...
                           'dd', 'dt', 'tfc', 'et', 'dq', 'forwardDrive'};
                for i = 1:length(required)
                    if ~isfield(inputs.parachute, required{i})
                        error('Missing parachute parameter: %s', required{i});
                    end
                end
            else
                error('Parachute data must be provided in data_out.parachute');
            end
            
            %% ALTITUDES
            inputs.altitude = struct();
            
            % Drop altitude
            if isfield(data_out, 'carp') && isfield(data_out.carp, 'altitude')
                inputs.altitude.dropIndicatedTrue = data_out.carp.altitude;
            else
                error('Drop altitude required in data_out.carp.altitude');
            end
            
            % DZ altimeter setting
            if isfield(data_out, 'dz') && isfield(data_out.dz, 'altimeter_setting')
                inputs.altitude.dzAltimeter = data_out.dz.altimeter_setting;
            else
                error('DZ altimeter setting required in data_out.dz.altimeter_setting');
            end
            
            % PI elevation
            if isfield(data_out, 'dz') && isfield(data_out.dz, 'pi_elevation')
                inputs.altitude.piElevation = data_out.dz.pi_elevation;
            elseif isfield(data_out, 'carp') && isfield(data_out.carp, 'land_location')
                % Try to extract from land_location if it contains elevation
                if isstruct(data_out.carp.land_location) && ...
                   isfield(data_out.carp.land_location, 'elevation')
                    inputs.altitude.piElevation = data_out.carp.land_location.elevation;
                else
                    error('PI elevation required');
                end
            else
                error('PI elevation required in data_out.dz.pi_elevation');
            end
            
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
            if strcmp(inputs.mission.type, 'HALO')
                if isfield(data_out, 'halo') && isfield(data_out.halo, 'actuation_altitude')
                    inputs.altitude.actuationAGL = data_out.halo.actuation_altitude;
                else
                    error('Actuation altitude required for HALO in data_out.halo.actuation_altitude');
                end
            end
            
            %% TEMPERATURES
            inputs.temps = struct();
            
            if isfield(data_out, 'temps')
                inputs.temps.drop = data_out.temps.drop;
                inputs.temps.surface = data_out.temps.surface;
                if strcmp(inputs.mission.type, 'HALO')
                    inputs.temps.actuation = data_out.temps.actuation;
                else
                    inputs.temps.actuation = data_out.temps.drop;
                end
            else
                error('Temperature data required in data_out.temps');
            end
            
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