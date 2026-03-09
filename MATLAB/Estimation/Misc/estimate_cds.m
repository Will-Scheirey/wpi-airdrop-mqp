function CdS = estimate_cds(T, rho, m, u)

g = 9.81;

u_total = @(CdS) (2*m)./(rho*CdS) .* log( cosh( g*T ./ sqrt( (2*m*g)./(rho*CdS) ) ) );

CdS = fsolve(@(x) u_total(x) - u, 0.01, optimoptions('fsolve', 'Display', 'off'));

end