function plot_parachute_drag_components(t, y, parachute, extractor)
% Figure 9: Fx,Fy,Fz of parachute drag vs time, computed from (t,y).
% extractor: function handle -> [Vc_B, rho] = extractor(tk, xk)

N = numel(t);
Fx = zeros(N,1); Fy = Fx; Fz = Fx;

for k = 1:N
    [Vc_B, rho] = extractor(t(k), y(k,:).');      % 3x1, scalar
    F_B = parachute_drag_dynamic(parachute, rho, Vc_B);  % uses Cd(aoa)
    Fx(k) = F_B(1); Fy(k) = F_B(2); Fz(k) = F_B(3);
end

figure(9); clf; set(gcf,'Name','Figure 9: Parachute Drag Components','Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile; plot(t, Fx, 'LineWidth',1.4); grid on; ylabel('F_x [N]');
title('Parachute drag in body axes');

nexttile; plot(t, Fy, 'LineWidth',1.4); grid on; ylabel('F_y [N]');

nexttile; plot(t, Fz, 'LineWidth',1.4); grid on; ylabel('F_z [N]'); xlabel('Time [s]');
end
