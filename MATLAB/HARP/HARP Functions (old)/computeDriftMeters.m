function drift_m = computeDriftMeters(wind_speed_kt, time_s)
% computeDriftMeters - convert wind knots and time (s) to meters drift
% AFMAN uses 1.94 constant to convert knots to m/s (1 kt = 0.514444 m/s; 1.94 is used in formulas that multiply knots*time to yield meters?).
% We'll use the exact conversion: 1 knot = 0.514444 m/s
ms_per_kt = 0.514444;
drift_m = wind_speed_kt * ms_per_kt * time_s;
end
