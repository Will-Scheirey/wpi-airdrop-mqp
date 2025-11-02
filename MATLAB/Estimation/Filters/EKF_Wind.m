classdef EKF_Wind < EKF_Basic_Kinematics

    properties
    end

    methods
        function obj = EKF_Wind(R, Q, x0, H0, P0, dt, J, tau_d)
            obj = obj@EKF_Basic_Kinematics(R, Q, x0, H0, P0, dt, J, tau_d);

            obj.x_inds.wind = 17:18;
        end

        function wind_out = get_wind(obj)
            wind_out = obj.x_curr(obj.x_inds.wind);
        end

        function dxdt = f(obj, u)
            V_b = obj.get_V_B();
            e   = obj.get_e();
            w_b = obj.get_w_b();
            d_a = obj.get_d_a();

            a_b = u(1:3);

            C_BE = ecef2body_rotm(e);

            dP_dt = C_BE' * V_b;
            dV_dt = a_b + d_a + C_BE * obj.g_vec_e - cross(w_b, V_b);

            de_dt = -1/2 * quat_kinematic_matrix(w_b) * e;
            dw_dt = obj.J \ (-cross(w_b, obj.J*w_b));

            dd_dt = -(1/obj.tau_d) * d_a;

            dxdt = [dP_dt; dV_dt; de_dt; dw_dt; dd_dt; 0; 0];
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

                zeros(6,1);
                0;
                0
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

                zeros(6,1);
                0;
                0
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

                zeros(6,1);
                0;
                0
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

                1;
                0;
                0;
                0;
                0
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

                0;
                1;
                0;
                0;
                0
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

                0;
                0;
                1;
                0;
                0
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
                0;
                0
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
                0;
                0
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
                0;
                0
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
                0;
                0
                ]';

            dx10dx = [
                zeros(10, 1);
                0;
                w2 * (J22 - J33) / J11;
                w1 * (J22 - J33) / J11;

                zeros(3,1);
                0;
                0
                ]';

            dx11dx = [
                zeros(10, 1);
                w2 * (J33 - J11) / J22;
                0;
                w0 * (J33 - J11) / J22;

                zeros(3,1);
                0;
                0
                ]';

            dx12dx = [
                zeros(10, 1);
                w1 * (J11 - J22) / J33;
                w0 * (J11 - J22) / J33;
                0;

                zeros(3,1);
                0;
                0
                ]';

            dx13dx = [
                zeros(13,1);
                -1/obj.tau_d;
                0;
                0;
                0;
                0
                ]';

            dx14dx = [
                zeros(13,1);
                0;
                -1/obj.tau_d;
                0;
                0;
                0
                ]';

            dx15dx = [zeros(13,1);
                0;
                0;
                -1/obj.tau_d;
                0;
                0
                ]';

            dx16dx = [
                zeros(1, 18);
                ];

            dx17dx = [
                zeros(1, 18);
                ];

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
                dx17dx
                ];
        end

        function dvgdx = v_g_jacobian(obj)
            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            v_g = obj.get_V_B();
            v0 = v_g(1); v1 = v_g(2); v2 = v_g(3);

            dvgdx = [
                (2*(2*e0*e3 + 2*e1*e2)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2) + 2*(e0^2 + e1^2 - e2^2 - e3^2)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                -(2*(2*e0*e3 - 2*e1*e2)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) - 2*(e0^2 - e1^2 + e2^2 - e3^2)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                (2*(2*e0*e2 + 2*e1*e3)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) - 2*(2*e0*e1 - 2*e2*e3)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                (2*(2*e0*v0 + 2*e2*v2 - 2*e3*v1)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) + 2*(2*e0*v1 - 2*e1*v2 + 2*e3*v0)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                -(2*(2*e0*v2 + 2*e1*v1 - 2*e2*v0)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2) - 2*(2*e1*v0 + 2*e2*v1 + 2*e3*v2)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                (2*(2*e0*v2 + 2*e1*v1 - 2*e2*v0)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) + 2*(2*e1*v0 + 2*e2*v1 + 2*e3*v2)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2));

                -(2*(2*e0*v1 - 2*e1*v2 + 2*e3*v0)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) - 2*(2*e0*v0 + 2*e2*v2 - 2*e3*v1)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2))
                ];

            dvgdx = dvgdx';
        end

        function dhdx = h_jacobian_states(obj)
            dvgdx = obj.v_g_jacobian();

            dhdx = [
                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;

                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                0, 0, 0, dvgdx(1), dvgdx(2), dvgdx(3), dvgdx(4), dvgdx(5), dvgdx(6), dvgdx(7), 0, 0, 0, 0, 0, 0, 0, 0
                ];
        end

        function y = h(obj)
            P_E = obj.get_P_E();
            p0 = P_E(1); p1 = P_E(2); p2 = P_E(3);

            e = obj.get_e();
            e0 = e(1); e1 = e(2); e2 = e(3); e3 = e(4);

            w_b = obj.get_w_b();
            w0 = w_b(1); w1 = w_b(2); w2 = w_b(3);

            v_g = obj.get_V_B();
            v_g_ecef = ecef2body_rotm(e)'*v_g;
            v_g0 = norm(v_g_ecef(1:2));


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
                w2;

                v_g0
                ];
        end
    end
end