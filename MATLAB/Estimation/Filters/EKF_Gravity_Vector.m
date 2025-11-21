classdef EKF_Gravity_Vector < EKF_V_E
    properties
        g_norm = 9.80665;
        accel_gate  = 1.0;
    end

    methods
        function [q_meas, trust_accel] = quat_from_acc_mag(obj, a_b, m_b)
            a_norm = norm(a_b);

            trust_accel = abs(a_norm - obj.g_norm) < obj.accel_gate;

            if trust_accel
                d_b = -a_b / a_norm;
            else
                e = obj.get_e;
                g_b = ecef2body_rotm(e) * (obj.g_vec_e / obj.g_norm);

                d_b = g_b / norm(g_b);
            end
            
            m_b = m_b / norm(m_b);

            h_b = cross(m_b, d_b);
            h_b = h_b / norm(h_b);

            n_b = cross(d_b, h_b);

            C_BN = [n_b, h_b, d_b];

            q_meas = obj.rotm_to_quat(C_BN);
        end

            function q = rotm_to_quat(~, R)
            % Convert 3x3 rotation matrix to [q0;q1;q2;q3] (scalar-first)
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
    end
end