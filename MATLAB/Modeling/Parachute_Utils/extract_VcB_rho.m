function [Vc_B, rho] = extract_VcB_rho(tk, xk)
% Map your state to canopy BODY velocity and density.

% --- Example A: state = [r_E; v_E; eul321] ---
% idx.r=1:3; idx.v=4:6; idx.eul=7:9;
% r_E = xk(idx.r); v_E = xk(idx.v);
% psi=xk(idx.eul(1)); theta=xk(idx.eul(2)); phi=xk(idx.eul(3));
% C_B_E = eul321_to_dcm(psi,theta,phi);

% --- Example B: state = [r_E; v_E; q_EB] ---
% idx.r=1:3; idx.v=4:6; idx.q=7:10;
% r_E = xk(idx.r); v_E = xk(idx.v);
% C_B_E = quat_to_dcm(xk(idx.q));    % your existing util

% --- Wind in E (replace with your model) ---
% W_E = wind_in_E(tk, r_E);          % 3x1
% Vrel_E = v_E - W_E;

% --- Transform to BODY ---
% Vc_B = C_B_E * Vrel_E;

% --- Density from altitude ---
% h = r_E(3);                         % adjust sign for NED if needed
% rho = StandardAtmosphereModel.Density(h);

% ===== temporary safe fallback (replace with your mapping) =====
v_E = xk(4:6);
Vc_B = v_E;                           % assume BODY==E, no wind
h = xk(3);
rho = StandardAtmosphereModel.Density(h);
end