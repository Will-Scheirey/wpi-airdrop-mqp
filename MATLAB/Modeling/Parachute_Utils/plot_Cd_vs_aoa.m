function plot_Cd_vs_aoa(parachute)
% Figure 8: C_D vs angle of attack (0–90 deg). Uses angle-only Cd(aoa).

    N   = 361;
    aoa = linspace(0, pi/2, N);        % radians
    Cd  = arrayfun(@(a) parachute.Cd(a), aoa);

    Cd0   = parachute.Cd(0);
    Cedge = parachute.Cd(pi/2);

    figure(8); clf;                     % enforce Figure 8 after sim
    set(gcf,'Name','Figure 8: C_D vs AOA','Color','w');

    plot(aoa*180/pi, Cd, 'LineWidth', 1.8); hold on; grid on;
    yline(Cd0,'k:','LineWidth',1);
    yline(Cedge,'k:','LineWidth',1);
    scatter([0 90],[Cd0 Cedge],25,'filled');

    xlabel('\alpha [deg]');
    ylabel('C_D');
    title('C_D(\alpha) — Parachute angle-only model');
    legend('C_D(\alpha)','C_{D0}','C_{edge}','Location','best');
end