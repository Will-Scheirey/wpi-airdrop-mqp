% CREATE_PARACHUTE Instantiate a Parachute object with standard G-12D parameters.
%   Builds a Parachute object using fixed physical constants for the G-12D
%   cargo parachute (mass, riser length, riser stiffness and damping,
%   canopy efficiency, and porosity), with a caller-specified radius and
%   deployment/cut timing.
%
% INPUTS:
%   r        : Canopy radius (m)
%   t_deploy : Deployment time (s); defaults to 0 (immediate deployment)
%   t_cut    : Cut/release time (s); defaults to t_deploy if not provided
%
% OUTPUTS:
%   parachute : Parachute object with the following properties set:
%                 - radius            : r (m)
%                 - canopy_mass       : 57 kg (G-12D single chute)
%                 - riser_length      : 18.28 m (60 ft)
%                 - riser_k           : 1000 N/m per riser
%                 - riser_c           : 10000 N·s/m system damping
%                 - canopy_efficiency : 0.90 (90%)
%                 - canopy_porosity   : 0.15 (15%)
%                 - use_drag          : true
%                 - variable_ma       : true
%                 - t_deploy          : deployment time (s)
%                 - t_cut             : cut time (s)
%                 - is_deployed       : true if t_deploy == 0, else false
%
% NOTES:
%   - riser_c was noted in the original as a candidate cause of long
%     simulation run times; value was tested and confirmed at 10000 N·s/m.
%   - is_deployed is set to true only when t_deploy == 0, meaning the
%     parachute is treated as already open at t=0 for immediate deployment.


function parachute = Create_Parachute(r, t_deploy, t_cut)
    if nargin < 2
        t_deploy = 0;  % Default: immediate deployment
    end
    if nargin < 3
        t_cut = t_deploy;
    end
    
    % Single G-12D parameters
    single_mass = 57;            % kg (125 lb per chute)
    riser_length = 18.28;        % m (60 ft published)
    riser_k = 1000;             % N/m per riser
    %riser_c = 10000;               % N·s/m system damping
    %test change to see if this is whats causing the program to have an
    %extrodinaryly long run time
    riser_c = 10000;

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
    parachute.t_cut = t_cut; % trying to ensure that the first chute opens at 0.1 seconds
    parachute.t_deploy = t_deploy;
    % parachute.t_cut = inf; % trying to ensure that the first chute opens at 0.1 seconds
    parachute.is_deployed = (t_deploy == 0);  % Deploy immediately if t_deploy is 0
    
end

