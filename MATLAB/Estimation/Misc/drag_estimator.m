clear; clc; close all

num_sec = 100;
steps = 10000;
tspan = linspace(0, num_sec, steps);
[t, y, model] = propagate_model('tspan', tspan, 'riser', false);

%% Estimation

g = 9.81;

rho_avg = (StandardAtmosphereModel.Density(y(1, 3)) + StandardAtmosphereModel.Density(y(end, 3))) / 2;
% rho_avg = StandardAtmosphereModel.Density(0);

m = model.payload.mass;

u = y(1, 3) - y(end, 3);

CdS = estimate_cds(num_sec, rho_avg, m, u)

CdS_actual = model.payload.CdS(0)

figure(1);
clf
plot(t, y(:,3))

figure(2)
clf
plot(t, vecnorm(y(:, 4:6), 2, 2))

figure(3)
clf

F_g = 9.81 * m;

CdS_list = linspace(0, 5, 1000);

T = num_sec;

u_total = @(CdS, rho) (2*m)./(rho*CdS) .* log( cosh( g*T ./ sqrt( (2*m*g)./(rho*CdS) ) ) );

u_totals = u_total(CdS_list, rho_avg);

plot(CdS_list, u_totals); hold on
yline(u);


