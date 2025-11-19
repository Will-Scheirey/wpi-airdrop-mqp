classdef EKF_V_E < EKF_Basic_Kinematics
    %BASIC_PARACHUTE_KF Summary of this class goes here
    %   Detailed explanation goes here

    properties

    end

    methods
        function obj = EKF_V_E(R, Q, x0, H0, P0, dt, J)

            obj = obj@EKF_Basic_Kinematics(R, Q, x0, H0, P0, dt, J);

            obj.x_inds.b_g = (14:16)';
            obj.x_inds.V_E = (4:6)';

            obj.dhdx_p = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            ];

            obj.dhdx_q = [
                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
            ];

            obj.dhdx_w = [
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1;
            ];
        end

        function P_E_out = get_P_E(obj)
            P_E_out = obj.x_curr(obj.x_inds.P_E);
        end

        function V_E_out = get_V_E(obj)
            V_E_out = obj.x_curr(obj.x_inds.V_E);
        end

        function e_out = get_e(obj)
            e_out   = obj.x_curr(obj.x_inds.e);
        end

        function w_b_out = get_w_b(obj)
            w_b_out  = obj.x_curr(obj.x_inds.w_b);
        end

        function b_g_out = get_b_g(obj)
            b_g_out = obj.x_curr(obj.x_inds.b_g);
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
            V_e = obj.get_V_E();
            e   = obj.get_e();
            
            w_b = obj.get_w_b();

            a_b = u(1:3);

            C_EB = ecef2body_rotm(e)';

            dP_dt = V_e;
            dV_dt = C_EB * a_b + obj.g_vec_e;

            de_dt = -1/2 * quat_kinematic_matrix(w_b) * e;
            dw_dt = obj.J \ (-cross(w_b, obj.J*w_b));

            db_g_dt = zeros(3,1);

            dxdt = [dP_dt; dV_dt; de_dt; dw_dt; db_g_dt];
        end

        function dfdx = f_jacobian_states(obj, u)
            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.get_w_b();
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            a = u(1:3);
            a0 = a(1); a1 = a(2); a2 = a(3);

            J11 = obj.J(1,1);
            J22 = obj.J(2,2);
            J33 = obj.J(3,3);

            dx0dx = [
                zeros(3,1);

                1;
                0;
                0;

                0;
                0;
                0;
                0;

                zeros(6,1)
                ]';

            dx1dx = [
                zeros(3,1);

                0;
                1;
                0;

                0;
                0;
                0;
                0;

                zeros(6,1)
                ]';

            dx2dx = [
                zeros(3,1);

                0;
                0;
                1;

                0;
                0;
                0;
                0;

                zeros(6,1)
                ]';

            dx3dx = [
                zeros(3, 1);

                0;
                0;
                0;
                
                2*a0*e0 - 2*a1*e3 + 2*a2*e2;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;
                2*a1*e1 - 2*a0*e2 + 2*a2*e0;
                2*a2*e1 - 2*a0*e3 - 2*a1*e0;

                0;
                0;
                0

                0;
                0;
                0
                ]';

            dx4dx = [
                zeros(3, 1);
                0;
                0;
                0;

                2*a1*e0 + 2*a0*e3 - 2*a2*e1;
                2*a0*e2 - 2*a1*e1 - 2*a2*e0;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;
                2*a0*e0 - 2*a1*e3 + 2*a2*e2;

                0;
                0;
                0;

                0;
                0;
                0;
                ]';


            dx5dx = [
                zeros(3, 1);
                0;
                0;
                0;

                2*a1*e1 - 2*a0*e2 + 2*a2*e0;
                2*a1*e0 + 2*a0*e3 - 2*a2*e1;
                2*a1*e3 - 2*a0*e0 - 2*a2*e2;
                2*a0*e1 + 2*a1*e2 + 2*a2*e3;

                0;
                0;
                0;

                0;
                0;
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

                zeros(3,1);
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

                zeros(3,1);
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

                zeros(3,1);
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

                zeros(3,1);
                ]';

            dx10dx = [
                zeros(10, 1);
                0;
                w2 * (J22 - J33) / J11;
                w1 * (J22 - J33) / J11;

                zeros(3,1);
                ]';

            dx11dx = [
                zeros(10, 1);
                w2 * (J33 - J11) / J22;
                0;
                w0 * (J33 - J11) / J22;

                zeros(3,1);
                ]';

            dx12dx = [
                zeros(10, 1);
                w1 * (J11 - J22) / J33;
                w0 * (J11 - J22) / J33;
                0;

                zeros(3,1);
                ]';

            dx13dx = [
                zeros(13,1);
                0;
                0;
                0
                ]';

            dx14dx = [
                zeros(13,1);
                0;
                0;
                0
                ]';

            dx15dx = [zeros(13,1);
                0;
                0;
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
              dx13dx;
              dx14dx;
              dx15dx;
            ];
        end

        function y = h(obj)
            y = h@EKF_Basic_Kinematics(obj);

            b_g = obj.get_b_g();

            y(8:10) = y(8:10) + b_g;
        end
    end
end