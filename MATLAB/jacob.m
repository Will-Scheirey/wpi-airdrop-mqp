clear; clc;
syms e0 e1 e2 e3 v0 v1 v2 real;

e = [e0; e1; e2; e3];
v = [v0; v1; v2];

n = ecef2body_rotm(e)' * v;

thing = norm(n(1:2));

simplify(jacobian(thing, v0))
simplify(jacobian(thing, v1))
simplify(jacobian(thing, v2))
simplify(jacobian(thing, e0))
simplify(jacobian(thing, e1))
simplify(jacobian(thing, e2))
simplify(jacobian(thing, e3))
