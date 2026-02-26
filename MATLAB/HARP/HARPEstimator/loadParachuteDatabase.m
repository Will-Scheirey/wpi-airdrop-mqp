function parachute = loadParachuteDatabase(parachuteType, weight)
    % Load parachute ballistics from database
    % Data from AFMAN 11-231 online ballistics tables
    
    switch upper(parachuteType)
        case 'MC-4'
            parachute = struct();
            parachute.type = 'MC-4';
            parachute.weight = weight;
            parachute.hvRoF = 156.6;        % ft/sec
            parachute.deployedRoF = 19.2;   % ft/sec (300 lbs)
            parachute.vd = 2900;            % ft
            parachute.dd = 380;             % ft
            parachute.dt = 3.4;             % sec
            parachute.tfc = 11.2;           % sec
            parachute.et = 2.3;             % sec
            parachute.dq = 7.5;             % sec
            parachute.forwardDrive = 19.8;  % kts
            
        case 'MC-5'
            parachute = struct();
            parachute.type = 'MC-5';
            parachute.weight = weight;
            parachute.hvRoF = 156.6;
            parachute.deployedRoF = 16.5;
            parachute.vd = 2900;
            parachute.dd = 420;
            parachute.dt = 3.8;
            parachute.tfc = 11.2;
            parachute.et = 2.3;
            parachute.dq = 7.5;
            parachute.forwardDrive = 22.5;
            
        case 'T-11'
            parachute = struct();
            parachute.type = 'T-11';
            parachute.weight = weight;
            parachute.hvRoF = 0;  % Static line only
            parachute.deployedRoF = 19.0;
            parachute.vd = 150;
            parachute.dd = 0;
            parachute.dt = 0;
            parachute.tfc = 3.5;
            parachute.et = 1.0;
            parachute.dq = 2.0;
            parachute.forwardDrive = 0;  % Non-gliding

        case 'G-15'
            parachute = struct();
            parachute.type = 'G-15';
            parachute.weight = weight;
            parachute.hvRoF = 150;  % Static line only
            parachute.deployedRoF = 23.0;  % G-15 has slower descent than T-11
            parachute.vd = 150;
            parachute.dd = 0;
            parachute.dt = 0;
            parachute.tfc = 3.5;
            parachute.et = 1.0;
            parachute.dq = 2.0;
            parachute.forwardDrive = 0;  % Non-gliding
            
        otherwise
            error('Parachute type %s not in database. Add to loadParachuteDatabase function.', parachuteType);
    end
end