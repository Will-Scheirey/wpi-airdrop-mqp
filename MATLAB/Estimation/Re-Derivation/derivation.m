clear; clc;
syms p0 p1 p2 v0 v1 v2 e0 e1 e2 e3 ...
     bw0 bw1 bw2 ba0 ba1 ba2 bp0 bp1 bp2 bm0 bm1 bm2 bb bv0 bv1 bv2 real;

syms a0 a1 a2 w0 w1 w2 real;
syms g0 g1 g2 m00 m01 m02;
g = [g0; g1; g2];
m0 = [m00; m01; m02];

p =   [p0;  p1;  p2];
v =   [v0;  v1;  v2];
e   = [e0;  e1;  e2; e3];
bw  = [bw0; bw1; bw2];
ba  = [ba0; ba1; ba2];
bp  = [bp0; bp1; bp2];
bm  = [bm0; bm1; bm2];
bv  = [bv0; bv1; bv2];
a   = [a0;  a1;  a2];
w   = [w0;  w1;  w2];

C_EB = body2enu_rotm(e);
C_BE = C_EB';

dp_dt = v;
dv_dt = C_EB * (a - ba) - g;
de_dt = 1/2 * quatmultiply(e', [0; (w - bw)]')';

x = [
    p;
    v;
    e;
    bw;
    ba;
    bp;
    bm;
    bb
    bv;
];

f = [dp_dt;
    dv_dt;
    de_dt;
    zeros(16, 1);
    ]



F = jacobian(f, x)

h = [
    p - bp;
    C_BE * m0 - bm;
    p(3) - bb;
    v - bv;
]

H = jacobian(h, x)
