function parachute = Create_Parachute(num_parachutes, t_deploy, r)
    if nargin < 2
        t_deploy = 0;  % Default: immediate deployment
    end
    
    % Single G-12D parameters
    single_mass = 57;            % kg (125 lb per chute)
    riser_length = 18.28;        % m (60 ft published)
    riser_k = 26000;             % N/m per riser
    riser_c = 700;               % N·s/m system damping
    
    % REALISTIC PARACHUTE PARAMETERS
    % using vertical-only drag, these can be physically realistic
    canopy_efficiency = 0.90;    % 90% efficient (realistic for cargo chute)
    canopy_porosity = 0.15;      % 15% porosity (typical for vents/seams)
    
    use_drag = true;
    
    parachute = Parachute(r, ...
        single_mass, ...
        riser_length, ...
        riser_k, ...
        riser_c, ...
        canopy_efficiency, ...
        canopy_porosity, ...
        use_drag, ...
        true);  % variable_ma = true
    
    % Set deployment time
    parachute.t_deploy = t_deploy;
    parachute.is_deployed = (t_deploy == 0);  % Deploy immediately if t_deploy is 0
    
end
