classdef EKF_No_Dynamics < Extended_Kalman_Filter
    %BASIC_PARACHUTE_KF Summary of this class goes here
    %   Detailed explanation goes here

    properties

    end

    methods
        function obj = EKF_No_Dynamics(R, Q, x0, H0, P0, dt)
            obj = obj@Extended_Kalman_Filter(R, Q, H0, x0, P0, dt);

            obj.H = obj.h_jacobian_states();
        end

        function step_filter(obj, y)
            step_filter@Extended_Kalman_Filter(obj, y);
            obj.x_curr(10:13) = obj.x_curr(10:13) / norm(obj.x_curr(10:13));

            % obj.P_curr(10:13, 10:13) = obj.P_curr(10:13, 10:13) / norm(obj.P_curr(10:13, 10:13));
        end

        function predict(obj)
            predict@Extended_Kalman_Filter(obj);

            obj.x_curr(10:13) = obj.x_curr(10:13) / norm(obj.x_curr(10:13));
        end

        function dxdt = f(obj)
            V_b = obj.x_curr(4:6);
            a_b = obj.x_curr(7:9);
            e   = obj.x_curr(10:13);
            w_b = obj.x_curr(14:16);
            alpha_b = obj.x_curr(17:19);

            C_EB = ecef2body_rotm(e)'; % Body to ECEF

            dP_dt = C_EB * V_b;
            dV_dt = a_b;
            da_dt = zeros(3, 1);

            de_dt = -1/2 * quat_kinematic_matrix(w_b) * e;
            dw_dt = alpha_b;
            dalpha_dt = zeros(3, 1);

            dxdt = [dP_dt; dV_dt; da_dt; de_dt; dw_dt; dalpha_dt];

            %{
            Alternatively, the transition matrix can be found by

                e   = obj.x_curr(10:13);
                w_b = obj.x_curr(14:16);
    
                e_dot = -1/2 * quat_kinematic_matrix(w_b);
    
                C_EB = ecef2body_rotm(e)'; % Body to ECEF

                A = [
                0, 0, 0, V(1,1), V(1,2), V(1,3), 0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, V(2,1), V(2,2), V(2,3), 0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, V(3,1), V(3,2), V(3,3), 0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      1, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 1, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 1, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, E(1,1), E(1,2), E(1,3), E(1,4), 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, E(2,1), E(2,2), E(2,3), E(2,4), 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, E(3,1), E(3,2), E(3,3), E(3,4), 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, E(4,1), E(4,2), E(4,3), E(4,4), 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                0, 0, 0, 0,      0,      0,      0, 0, 0, 0,      0,      0,      0,      0, 0, 0, 0, 0, 0;
                ];

            And dxdt = Ax
            %}
        end

        function dfdx = f_jacobian_states(obj)
            V_b = obj.x_curr(4:6);
            v0 = V_b(1); v1 = V_b(2); v2 = V_b(3);

            e = obj.x_curr(10:13);
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.x_curr(14:16);
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            dx0dx = [
                zeros(3,1);

                e0^2 + e1^2 - e2^2 - e3^2;
                2*(e1*e2 + e0*e3);
                2*(e1*e3 - e0*e2);

                zeros(3,1);

                 1/2*e0*v0 + 2*e3*v1 - 2*e2*v2;
                 1/2*e1*v0 + 2*e2*v1 + 2*e3*v2;
                -1/2*e2*v0 + 2*e1*v1 - 2*e0*v2;
                -1/2*e3*v0 + 2*e0*v1 + 2*e1*v2;

                zeros(6,1)
                ]';

            dx1dx = [
                zeros(3,1);

                2*(e1*e2 - e0*e3);
                e0^2 - e1^2 + e2^2 - e3^2;
                2*(e2*e3 + e0*e1);

                zeros(3,1);

                -2*e3*v0 + 1/2*e0*v1 + 2*e1*v2;
                -2*e2*v0 - 1/2*e1*v1 + 2*e0*v2;
                -2*e1*v0 + 1/2*e2*v1 + 2*e3*v2;
                -2*e0*v0 - 1/2*e3*v1 + 2*e2*v2;

                zeros(6,1)
                ]';

            dx2dx = [
                zeros(3,1);

                2*(e0*e2 + e1*e3);
                2*(e2*e3 - e0*e1);
                e0^2 - e1^2 + e2^2 + e3^2;

                zeros(3,1);

                2*e2*v0 - 2*e1*v1 + 1/2*e0*v2;
                2*e3*v0 - 2*e0*v1 - 1/2*e1*v2;
                2*e0*v0 + 2*e3*v1 + 1/2*e2*v2;
                2*e1*v0 - 2*e2*v1 + 1/2*e3*v2;

                zeros(6,1)
                ]';

            dx3dx = [
                zeros(6, 1);
                1;
                0;
                0;
                zeros(10, 1);
            ]';

            dx4dx = [
                zeros(6, 1);
                0;
                1;
                0;
                zeros(10, 1);
            ]';


            dx5dx = [
                zeros(6, 1);
                0;
                0;
                1;
                zeros(10, 1);
            ]';
            
            dx6dx = zeros(19, 1)';
            dx7dx = zeros(19, 1)';
            dx8dx = zeros(19, 1)';

            dx9dx = [
                zeros(9, 1);

                 0;
                -1/2*w0;
                -1/2*w1;
                -1/2*w2;

                -1/2*e1;
                -1/2*e2;
                -1/2*e3;

                zeros(3, 1);
            ]';

            dx10dx = [
                zeros(9, 1);

                 1/2*w0;
                 0;
                 1/2*w2;
                -1/2*w1;

                 1/2*e0;
                -1/2*e3;
                 1/2*e2;

                zeros(3, 1);
            ]';

            dx11dx = [
                zeros(9, 1);

                 1/2*w1;
                -1/2*w2;
                 0;
                 1/2*w0;

                 1/2*e3;
                 1/2*e0;
                -1/2*e1;

                zeros(3, 1);
                ]';

            dx12dx = [
                zeros(9, 1);

                 1/2*w2;
                 1/2*w1;
                -1/2*w0;
                 0;

                -1/2*e2;
                 1/2*e1;
                 1/2*e0;

                zeros(3, 1);
            ]';

            dx13dx = [
                zeros(16, 1);
                1;
                0;
                0;
            ]';

            dx14dx = [
                zeros(16, 1);
                0;
                1;
                0;
            ]';

            dx15dx = [
                zeros(16, 1);
                0;
                0;
                1;
            ]';

            dx16dx = zeros(19, 1)';
            dx17dx = zeros(19, 1)';
            dx18dx = zeros(19, 1)';

            dfdx = [
              dx0dx;
              dx1dx;
              dx2dx;
              dx3dx;
              dx4dx;
              dx5dx;
              dx6dx;
              dx7dx;
              dx8dx;
              dx9dx;
              dx10dx;
              dx11dx;
              dx12dx;
              dx13dx;
              dx14dx;
              dx15dx;
              dx16dx;
              dx17dx;
              dx18dx;
            ];
        end

        function dhdx = h_jacobian_states(~)
            dhdx = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                ];
        end

        function y = h(obj)
            P_E = obj.x_curr(1:3);
            p0 = P_E(1); p1 = P_E(2); p2 = P_E(3);

            a_B = obj.x_curr(7:9);
            a0 = a_B(1); a1 = a_B(2); a2 = a_B(3);

            e = obj.x_curr(10:13);
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.x_curr(14:16);
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            y = [
                p0;
                p1;
                p2;

                a0;
                a1;
                a2

                e0;
                e1;
                e2;
                e3;

                w0; 
                w1; 
                w2
            ];
        end
    end
end