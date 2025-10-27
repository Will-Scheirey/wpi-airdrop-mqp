clear; clc

syms w0 w1 w2 I11 I12 I13 I21 I22 I23 I31 I32 I33;

I = [
    I11, 0, 0;
    0, I22, 0;
    0, 0, I33;
];

w = [w0; w1; w2];

res = I \ (-cross(w, I*w))

res_simp = simplify(res)

jacobian(res, [w0, w1, w2])