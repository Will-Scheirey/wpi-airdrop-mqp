clc; close all

syms a0 a1 a2 e0 e1 e2 e3 ba0 ba1 ba2 real;

e = [e0; e1; e2; e3];
a_b = [a0+ba0; a1+ba1; a2+ba2];

v_dot_e = ecef2body_rotm(e)' * a_b

dvde0 = jacobian(v_dot_e, e0)
dvde1 = jacobian(v_dot_e, e1)
dvde2 = jacobian(v_dot_e, e2)
dvde3 = jacobian(v_dot_e, e3)

dvdb0 = jacobian(v_dot_e, ba0)
dvdb1 = jacobian(v_dot_e, ba1)
dvdb2 = jacobian(v_dot_e, ba2)
