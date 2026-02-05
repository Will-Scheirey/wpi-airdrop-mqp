classdef EKF_Basic_Kinematics < EKF_No_Dynamics
    %BASIC_PARACHUTE_KF Summary of this class goes here
    %   Detailed explanation goes here

    properties
    J
    g_vec_e = [0; 0; -9.81]
    end

    methods
        function obj = EKF_Basic_Kinematics(R, Q, x0, H0, P0, dt, J)

            obj = obj@EKF_No_Dynamics(R, Q, x0, H0, P0, dt);

            obj.H = obj.h_jacobian_states();

            obj.J = J;

            obj.dhdx_p = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            ];

            obj.dhdx_q = [
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
            ];

            obj.dhdx_w = [
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
            ];
        end

        function P_E_out = get_P_E(obj)
            P_E_out = obj.x_curr(obj.x_inds.P_E);
        end

        function V_B_out = get_V_B(obj)
            V_B_out = obj.x_curr(obj.x_inds.V_B);
        end

        function e_out = get_e(obj)
            e_out   = obj.x_curr(obj.x_inds.e);
        end

        function w_b_out = get_w_b(obj)
            w_b_out  = obj.x_curr(obj.x_inds.w_b);
        end

        function normalize_quat(obj)
            obj.x_curr(obj.x_inds.e) = obj.get_e() / norm(obj.get_e());
        end

        function [innovation, S] = step_filter(obj, y, u)
            [innovation, S] = step_filter@Extended_Kalman_Filter(obj, y, u);
            
            obj.normalize_quat();
        end

        function predict(obj, u)
            predict@Extended_Kalman_Filter(obj, u);

            obj.normalize_quat();
        end

        function dxdt = f(obj, u)
            V_b = obj.get_V_B();
            e   = obj.get_e();
            w_b = obj.get_w_b();

            a_b = u(1:3);

            C_BE = ecef2body_rotm(e);

            dP_dt = C_BE' * V_b;
            dV_dt = a_b + C_BE * obj.g_vec_e; % - cross(w_b, V_b);

            de_dt = -1/2 * quat_kinematic_matrix(w_b) * e;
            dw_dt = obj.J \ (-cross(w_b, obj.J*w_b));

            dxdt = [dP_dt; dV_dt; de_dt; dw_dt];
        end

        function dfdx = f_jacobian_states(obj, u)
            V_b = obj.get_V_B();
            v0 = V_b(1); v1 = V_b(2); v2 = V_b(3);

            % a_b = u(1:3);

            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.get_w_b();
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            g = -obj.g_vec_e(3);

            J11 = obj.J(1,1);
            J22 = obj.J(2,2);
            J33 = obj.J(3,3);

            dx0dx = [
                zeros(3,1);

                e0^2 + e1^2 - e2^2 - e3^2;
                2*(e1*e2 - e0*e3);
                2*(e1*e3 + e0*e2);

                2*e0*v0 - 2*e3*v1 + 2*e2*v2;
                2*e1*v0 + 2*e2*v1 + 2*e3*v2;
                -2*e2*v0 + 2*e1*v1 + 2*e0*v2;
                -2*e3*v0 - 2*e0*v1 + 2*e1*v2;

                zeros(3,1)
                ]';

            dx1dx = [
                zeros(3,1);

                2*(e1*e2 + e0*e3);
                e0^2 - e1^2 + e2^2 - e3^2;
                2*(e2*e3 - e0*e1);

                -2*e3*v0 + 2*e0*v1 - 2*e1*v2;
                -2*e2*v0 - 2*e1*v1 - 2*e0*v2;
                -2*e1*v0 + 2*e2*v1 + 2*e3*v2;
                -2*e0*v0 - 2*e3*v1 + 2*e2*v2;

                zeros(3,1)
                ]';

            dx2dx = [
                zeros(3,1);

                2*(-e0*e2 + e1*e3);
                2*(e2*e3 + e0*e1);
                e0^2 - e1^2 - e2^2 + e3^2;

                -2*e2*v0 + 2*e1*v1 + 2*e0*v2;
                2*e3*v0 + 2*e0*v1 - 2*e1*v2;
                -2*e0*v0 + 2*e3*v1 - 2*e2*v2;
                2*e1*v0 - 2*e2*v1 + 2*e3*v2;

                zeros(3,1)
                ]';

            dx3dx = [
                zeros(3, 1);
                0;
                -w2;
                w1;

                -2*e2*g;
                2*e3*g;
                -2*e0*g;
                2*e1*g;

                0;
                v2;
                -v1;
                ]';

            dx4dx = [
                zeros(3, 1);
                w2;
                0;
                -w0;

                2*e1*g;
                2*e0*g;
                2*e3*g;
                2*e2*g;

                -v2;
                0;
                v0;
                ]';


            dx5dx = [
                zeros(3, 1);
                -w1;
                w0;
                0

                2*e0*g;
                -2*e1*g;
                -2*e2*g;
                2*e3*g;

                v1;
                -v0;
                0;
                ]';

            dx6dx = [
                zeros(6, 1);

                0;
                -1/2*w0;
                -1/2*w1;
                -1/2*w2;

                -1/2*e1;
                -1/2*e2;
                -1/2*e3;
                ]';

            dx7dx = [
                zeros(6, 1);

                1/2*w0;
                0;
                1/2*w2;
                -1/2*w1;

                1/2*e0;
                -1/2*e3;
                1/2*e2;
                ]';

            dx8dx = [
                zeros(6, 1);

                1/2*w1;
                -1/2*w2;
                0;
                1/2*w0;

                1/2*e3;
                1/2*e0;
                -1/2*e1;
                ]';

            dx9dx = [
                zeros(6, 1);

                1/2*w2;
                1/2*w1;
                -1/2*w0;
                0;

                -1/2*e2;
                1/2*e1;
                1/2*e0;
                ]';

            dx10dx = [
                zeros(10, 1);
                0;
                w2 * (J22 - J33) / J11;
                w1 * (J22 - J33) / J11;
                ]';

            dx11dx = [
                zeros(10, 1);
                w2 * (J33 - J11) / J22;
                0;
                w0 * (J33 - J11) / J22;
                ]';

            dx12dx = [
                zeros(10, 1);
                w1 * (J11 - J22) / J33;
                w0 * (J11 - J22) / J33;
                0;
                ]';

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
            ];
        end
%{
        function dhdx = h_jacobian_states(~)
            dhdx = [
                

                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                ];
        end

        function y = h(obj)
            P_E = obj.get_P_E();
            p0 = P_E(1); p1 = P_E(2); p2 = P_E(3);

            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.get_w_b();
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            y = [
                p0;
                p1;
                p2;

                e0;
                e1;
                e2;
                e3;

                w0; 
                w1; 
                w2
            ];
        end
%}
    end
end