function debug_mag_convention_once(a_b_meas, m_b_meas, m_ref_i)
%DEBUG_MAG_CONVENTION_ONCE
% After applying (1) (transpose in quat_from_acc_mag),
% do this once at init:
%   1) compute q_meas from accel+mag (using the updated convention)
%   2) predict m_pred = ecef2body_rotm(q_meas)' * m_ref_i
%   3) compare direction/shape vs measured m_b (normalized)
%
% Inputs:
%   a_b_meas : 3x1 accel measurement in body (m/s^2)
%   m_b_meas : 3x1 mag measurement in body (any units)
%   m_ref_i  : 3x1 reference mag in inertial (unit or not; will normalize)
%
% Requires on path:
%   - ecef2body_rotm(q)  (returns C_bi, wxyz, body->inertial)
%   - your rotm_to_quat (or the one below)

    a_b_meas = a_b_meas(:);
    m_b_meas = m_b_meas(:);
    m_ref_i  = m_ref_i(:);

    % Normalize inputs for direction comparison
    a_b_u = a_b_meas / norm(a_b_meas);
    m_b_u = m_b_meas / norm(m_b_meas);
    m_ref_u = m_ref_i / norm(m_ref_i);

    % 1) compute q_meas (with the UPDATED convention)
    q_meas = quat_from_acc_mag_UPDATED(a_b_u, m_b_u);

    % 2) predict m_pred in body
    C_bi  = ecef2body_rotm(q_meas);   % body->inertial
    C_ib  = C_bi.';                  % inertial->body
    m_pred = C_ib * m_ref_u;
    m_pred_u = m_pred / norm(m_pred);

    % 3) compare vs measured
    cosang = dot(m_pred_u, m_b_u);
    cosang = max(-1, min(1, cosang));
    angdeg = acosd(cosang);

    fprintf("\n=== Magnetometer convention sanity check ===\n");
    fprintf("q_meas (wxyz) = [%.6f %.6f %.6f %.6f]\n", q_meas(1), q_meas(2), q_meas(3), q_meas(4));
    fprintf("Measured m_b (unit)  = [%.4f %.4f %.4f]\n", m_b_u(1), m_b_u(2), m_b_u(3));
    fprintf("Predicted m_pred (unit)= [%.4f %.4f %.4f]\n", m_pred_u(1), m_pred_u(2), m_pred_u(3));
    fprintf("Angle between (deg)   = %.3f\n", angdeg);

    if angdeg < 15
        fprintf("✅ Looks aligned (typically good).\n");
    elseif abs(angdeg-180) < 15
        fprintf("⚠️ Looks flipped ~180°. Likely sign/axis flip or m_ref_i direction issue.\n");
    else
        fprintf("❌ Not aligned. Convention still inconsistent (or m_ref_i is wrong for this segment).\n");
    end

    % Optional: quick 3D visualization
    figure; clf; grid on; axis equal; hold on;
    quiver3(0,0,0, m_b_u(1),    m_b_u(2),    m_b_u(3),    0, 'LineWidth', 2, 'DisplayName', 'm\_meas (body)');
    quiver3(0,0,0, m_pred_u(1), m_pred_u(2), m_pred_u(3), 0, 'LineWidth', 2, 'DisplayName', 'm\_pred (body)');
    xlabel('x_b'); ylabel('y_b'); zlabel('z_b');
    title(sprintf('Mag direction check: angle = %.2f deg', angdeg));
    legend('Location','best');
end

function q_meas = quat_from_acc_mag_UPDATED(a_b_u, m_b_u)
% Compute q from accel + mag, but with the "transpose change" applied.
% This returns q (wxyz) consistent with ecef2body_rotm(q) = C_bi (body->inertial).
%
% We build an ONB in body (e_b, n_b, u_b) like you did, then:
%   C_BE = [e_b n_b u_b]  (body axes expressed in Earth/inertial??)
% BUT the key is: if your previous rotm_to_quat expected the opposite direction,
% you now feed TRANSPOSE here:
%   q_meas = rotm_to_quat(C_BE');  % <---- the change (1)

    % Use accel to define down/up in body
    d_b = -a_b_u;              % down direction (body)
    u_b = -d_b;                % up direction

    % Make mag horizontal component define east
    m_b_u = m_b_u / norm(m_b_u);
    e_b = cross(m_b_u, d_b);
    e_b = e_b / norm(e_b);

    n_b = cross(u_b, e_b);
    n_b = n_b / norm(n_b);

    % This is your constructed DCM (as you had it)
    C_BE = [e_b, n_b, u_b];

    % *** The transpose change (1) ***
    q_meas = rotm_to_quat_local(C_BE.');
end

function q = rotm_to_quat_local(R)
% Paste your rotm_to_quat here if you want this standalone.
    tr = trace(R);
    if tr > 0
        S  = sqrt(tr + 1.0) * 2.0;
        q0 = 0.25 * S;
        q1 = (R(3,2) - R(2,3)) / S;
        q2 = (R(1,3) - R(3,1)) / S;
        q3 = (R(2,1) - R(1,2)) / S;
    else
        if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
            S  = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2.0;
            q0 = (R(3,2) - R(2,3)) / S;
            q1 = 0.25 * S;
            q2 = (R(1,2) + R(2,1)) / S;
            q3 = (R(1,3) + R(3,1)) / S;
        elseif R(2,2) > R(3,3)
            S  = sqrt(1.0 - R(1,1) + R(2,2) - R(3,3)) * 2.0;
            q0 = (R(1,3) - R(3,1)) / S;
            q1 = (R(1,2) + R(2,1)) / S;
            q2 = 0.25 * S;
            q3 = (R(2,3) + R(3,2)) / S;
        else
            S  = sqrt(1.0 - R(1,1) - R(2,2) + R(3,3)) * 2.0;
            q0 = (R(2,1) - R(1,2)) / S;
            q1 = (R(1,3) + R(3,1)) / S;
            q2 = (R(2,3) + R(3,2)) / S;
            q3 = 0.25 * S;
        end
    end
    q = [q0; q1; q2; q3];
    q = q / norm(q);
end