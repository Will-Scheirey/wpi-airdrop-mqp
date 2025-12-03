% clear; clc;
% syms e0 e1 e2 e3 v0 v1 v2 real;
% 
% e = [e0; e1; e2; e3];
% v = [v0; v1; v2];
% 
% n = ecef2body_rotm(e)' * v;
% 
% thing = norm(n(1:2));
% 
% simplify(jacobian(thing, v0))
% simplify(jacobian(thing, v1))
% simplify(jacobian(thing, v2))
% simplify(jacobian(thing, e0))
% simplify(jacobian(thing, e1))
% simplify(jacobian(thing, e2))
% simplify(jacobian(thing, e3))

e0 = 1; e1 = 2; e2 = 3; e3 = 4;
v0 = 10; v1 = 20; v2 = 30;

x1 = (2*(2*e0*e2 + 2*e1*e3)*(v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2) - 2*(2*e0*e1 - 2*e2*e3)*(v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2))/(2*((v0*e0^2 + 2*v2*e0*e2 - 2*v1*e0*e3 + v0*e1^2 + 2*v1*e1*e2 + 2*v2*e1*e3 - v0*e2^2 - v0*e3^2)^2 + (v1*e0^2 - 2*v2*e0*e1 + 2*v0*e0*e3 - v1*e1^2 + 2*v0*e1*e2 + v1*e2^2 + 2*v2*e2*e3 - v1*e3^2)^2)^(1/2))

x2_a = 4*e0*e2*(4*e1*e3*v2 + 2*e0*e2*v2 + e0^2*v0 + e1^2*v0 - e2^2*v0 - e3^2*v0 + 2*e1*e2*v1 - 2*e0*e3*v1) + 4*e1*e3*(2*e1*e3*v2 - e0^2*v0 + e1^2*v0 - e2^2*v0 - e3^2*v0 + 2*e1*e2*v1 - 2*e0*e3*v1);
x2_b = (e0^4*v0^2 + e1^4*v0^2 + e2^4*v0^2 + e3^4*v0^2 + 2*e0^2*e1^2*v0^2 - 2*e0^2*e2^2*v0^2 - 2*e0^2*e3^2*v0^2 - 2*e1^2*e2^2*v0^2 - 2*e1^2*e3^2*v0^2 + 2*e2^2*e3^2*v0^2 + 4*e1^2*e2^2*v1^2 - 8*e0*e1*e2*e3*v1^2 + 4*e0^2*e3^2*v1^2 + 4*e1^2*e3^2*v2^2 + 8*e0*e1*e2*e3*v2^2 + 4*e0^2*e2^2*v2^2 + 4*e0^2*e1*e2*v0*v1 - 4*e0^3*e3*v0*v1 + 4*e1^3*e2*v0*v1 - 4*e0*e1^2*e3*v0*v1 - 4*e1*e2^3*v0*v1 + 4*e0*e2^2*e3*v0*v1 - 4*e1*e2*e3^2*v0*v1 + 4*e0*e3^3*v0*v1 - 4*e0^2*e1*e3*v0*v2 + 4*e0^3*e2*v0*v2 + 4*e1^3*e3*v0*v2 - 4*e0*e2*e3^2*v0*v2 + 8*e1^2*e2*e3*v1*v2 + 8*e0*e1*e2^2*v1*v2 - 8*e0*e1*e3^2*v1*v2 - 8*e0^2*e2*e3*v1*v2)
x2 = x2_a/(2*x2_b)

east = (e0^2 + e1^2 + e2^2 + e3^2)*v0 + 2*(e1*e2 - e0*e3)*v1 + 2*(e1*e3 + e0*e2)*v2;
north = 2*(e1*e2 + e0*e3)*v0 + (e0^2 - e1^2 + e2^2 - e3^2)*v1 + 2*(e2*e3 + e0*e1)*v2;

val = east*north
