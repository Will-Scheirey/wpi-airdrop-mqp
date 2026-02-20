% AF4015_allitems.m
% Compute all 55 items of AF Form 4015 (best-effort implementation from AFMAN 11-231 §5.8)
% Save helper functions below as separate .m files or keep them in this file (local functions).

clear; clc;

%% ----------------------- INPUTS (Edit these for your mission) -----------------------
% Basic altitude & settings
inputs.Drop_IndicatedTrueAlt_ft = 20000;    % Item 1, ft MSL
inputs.DZ_AltimeterSetting_inHg  = 30.30;   % DZ altimeter setting (inHg)
inputs.D_value_ft                 = 750;    % Item 4, 'D' value (ft) (from forecaster or enroute measurement)
inputs.PI_elevation_ft            = 250;    % Item 6, PI elevation (MSL) ft
inputs.VD_ft                       = 2900;   % Item 8, VD (stabilization descent) ft (from ballistic data)
% Actuation / deployment
inputs.Actuation_IndicatedAGL_ft = 4000;    % Item 11a, actuation altitude intended by jumper (AGL)
% Ballistic / parachute parameters (examples from sample HARP)
inputs.HV_RoF_fps                 = 156.6;  % High-velocity rate of fall (ft/s) (if applicable)
inputs.HV_TFC_s                   = 11.2;   % time of fall constant for HV (s)
inputs.Deployed_RoF_fps           = 19.2;   % Deployed RoF (ft/s)
inputs.ET_s                       = 2.3;    % ET (exit time) seconds
inputs.DQ_s                       = 7.5;    % Deceleration quotient seconds
inputs.DD_ft                      = 380;    % Deceleration distance (DD) ft
inputs.DT_s                       = 3.4;    % deceleration time (s)
% Airspeed / temps / winds
inputs.Drop_IAS_KIAS              = 130;    % Indicated airspeed for the drop (KIAS)
inputs.Drop_Altitude_TempC        = -24;    % Temperature at drop altitude (C)
inputs.Surface_TempC              = 15;     % Surface temp (C)
% Winds (example arrays: [alt_ft, dir_deg_true, speed_kt])
% Provide as Nx3 matrix. Use available forecast/in-flight winds. Example below:
inputs.Winds = [...
    20000, 150, 60; ...
    18000, 155, 55; ...
    16000, 150, 47; ...
    14000, 160, 50; ...
    12000, 170, 42; ...
    10000, 175, 40; ...
     8000, 180, 40; ...
     6000, 170, 35; ...
     4000, 170, 30; ...
     2000, 165, 25; ...
     1000, 160, 20; ...
       0, 150, 13];  % surface last row
% DZ course and other metadata
inputs.DZ_course_true_deg = 180;   % Item 23
% Safety / LAR factors
inputs.SF_ft = 2000;               % Safety factor in feet (example)
inputs.SF_percent = 0.8;           % Safety percentage (80% typical)
% Misc
inputs.units = 'ft';               % Use 'ft' for most items; many internal computations use meters when specified

%% ----------------------- BEGIN COMPUTATION -----------------------
R = struct(); % result container

% Item 1
R.Item1_DropIndicatedTrueAlt_ft = inputs.Drop_IndicatedTrueAlt_ft;

% Item 2: PAV
R.Item2_PAV_ft = computePAV(inputs.DZ_AltimeterSetting_inHg);

% Item 3: Drop pressure altitude
R.Item3_DropPressureAlt_ft = dropPressureAlt(R.Item1_DropIndicatedTrueAlt_ft, R.Item2_PAV_ft);

% Item 4: D value (given input)
R.Item4_D_value_ft = inputs.D_value_ft;

% Item 5: Drop true altitude (Item3 + D)
R.Item5_DropTrueAlt_ft = R.Item3_DropPressureAlt_ft + R.Item4_D_value_ft;

% Item 6: PI elevation (given)
R.Item6_PI_elevation_ft = inputs.PI_elevation_ft;

% Item 7: Drop absolute altitude (Item5 - Item6)
R.Item7_DropAbsoluteAlt_ft = R.Item5_DropTrueAlt_ft - R.Item6_PI_elevation_ft;

% Item 8: VD (given ballistic)
R.Item8_VD_ft = inputs.VD_ft;

% Item 9: Stabilization altitude (Item7 - Item8) in feet AGL
R.Item9_StabilizationAlt_ft = R.Item7_DropAbsoluteAlt_ft - R.Item8_VD_ft;

% Item 10: PI pressure altitude (Item6 + Item2)
R.Item10_PI_PressureAlt_ft = R.Item6_PI_elevation_ft + R.Item2_PAV_ft;

% Items 11a & 11b: Actuation indicated altitude (AGL) and actuation indicated true altitude (MSL)
R.Item11a_ActuationIndicatedAGL_ft = inputs.Actuation_IndicatedAGL_ft;
R.Item11b_ActuationIndicatedTrueAlt_ft = computeActuationTrueAlt(R.Item11a_ActuationIndicatedAGL_ft, R.Item6_PI_elevation_ft);

% Item 12: Actuation pressure altitude (use PAV)
% Actuation pressure altitude = actuation true alt + PAV (same approach as other pressure-alt conversions)
R.Item12_ActuationPressureAlt_ft = R.Item11b_ActuationIndicatedTrueAlt_ft + R.Item2_PAV_ft;

% Item 13: Absolute actuation altitude (ft above PI) = actuation true alt - PI elevation
R.Item13_AbsActuationAlt_ft = R.Item11b_ActuationIndicatedTrueAlt_ft - R.Item6_PI_elevation_ft;

% Item 14: High-velocity mid-pressure altitude (midpoint between drop pressure altitude and actuation pressure altitude)
R.Item14_HV_MidPresAlt_ft = hvMidPressureAlt(R.Item3_DropPressureAlt_ft, R.Item12_ActuationPressureAlt_ft);

% Item 15: High-velocity adjusted rate of fall (ARoF) - density corrected
% we use function computeAdjustedRoF which applies a density/temperature correction
R.Item15_HV_ARoF_fps = computeAdjustedRoF(inputs.HV_RoF_fps, inputs.Drop_Altitude_TempC, inputs.Surface_TempC, R.Item14_HV_MidPresAlt_ft);

% Item 16: High-velocity time of fall (from stabilization to actuation or vice versa)
% Use ToF = total fall distance (stabilization altitude - abs actuation altitude?) / ARoF
% For HV phase, free-fall begins at exit and goes to actuation (or stabilization to actuation when 2-stage)
R.Item16_HV_ToF_s = computeTimeOfFall_HV(R.Item9_StabilizationAlt_ft, R.Item13_AbsActuationAlt_ft, R.Item15_HV_ARoF_fps, inputs.HV_TFC_s);

% Item 17: High-velocity drift effect (convert ballistic wind and time to lateral displacement)
% First compute HV ballistic wind vector (mean from stabilization down to actuation) -> use winds table
R.Item17_HV_BallisticWind = ballisticWind(inputs.Winds, R.Item9_StabilizationAlt_ft, R.Item13_AbsActuationAlt_ft, 'HV');
% convert to drift (meters or yards). AFMAN uses 1.94 to convert knots to m/s (for meters).
R.Item17_HV_Drift_m = computeDriftMeters(R.Item17_HV_BallisticWind.speed_kt, R.Item16_HV_ToF_s);

% Item 18: Delay distance (distance travelled during deceleration/transition if DD given)
R.Item18_DelayDistance_m = computeDelayDistance(inputs.DD_ft);

% Item 19: Deployed mid-pressure altitude (midpoint between actuation pressure altitude and PI pressure altitude)
R.Item19_Deployed_MidPresAlt_ft = deployedMidPressureAlt(R.Item12_ActuationPressureAlt_ft, R.Item10_PI_PressureAlt_ft);

% Item 20: Deployed adjusted rate of fall (ARoF) (density corrected for deployed mid-altitude)
R.Item20_Deployed_ARoF_fps = computeAdjustedRoF(inputs.Deployed_RoF_fps, inputs.Drop_Altitude_TempC, inputs.Surface_TempC, R.Item19_Deployed_MidPresAlt_ft);

% Item 21: Total deployed time (time from actuation to PI under deployed canopy)
R.Item21_TotalDeployedTime_s = computeTotalDeployedTime(R.Item13_AbsActuationAlt_ft, R.Item8_VD_ft, R.Item20_Deployed_ARoF_fps);

% Item 22: Deployed drive time (time parachute has forward drive while descending under canopy)
R.Item22_DeployedDriveTime_s = deployedDriveTime(R.Item21_TotalDeployedTime_s, inputs.DT_s);

% Item 23: Deployed wind effect (wind from actuation to surface, integrated)
R.Item23_DeployedBallisticWind = ballisticWind(inputs.Winds, R.Item13_AbsActuationAlt_ft, 0, 'Deployed');
R.Item23_DeployedWindEffect_m = computeDriftMeters(R.Item23_DeployedBallisticWind.speed_kt, R.Item21_TotalDeployedTime_s);

% Item 24: Deployed drive distance (approx forward drive under canopy) - use K-factor or D=KAV method placeholder
% Provide two options: if K-factor known, use D=K*A*V (requires canopy area A and forward velocity V). Otherwise estimate from empirical DDD formula.
R.Item24_DeployedDriveDistance_m = estimateDeployedDriveDistance(R.Item22_DeployedDriveTime_s, R.Item20_Deployed_ARoF_fps);

% Item 25: Forward travel distance (FTD) (Item29 conv to distance) — compute from forward travel time (FTT)
% We need FTT = ET + DQ (or as ballistic data)
R.Item25_ET_s = inputs.ET_s;       % Item 27
R.Item25_DQ_s = inputs.DQ_s;       % Item 28
R.Item25_FTT_s = R.Item25_ET_s + R.Item25_DQ_s; % Item 29
R.Item25_FTD_m = computeFTD(R.Item25_FTT_s, inputs.Drop_IAS_KIAS, R.Item3_DropPressureAlt_ft); % Item 30

% Item 26: Stopwatch distance (yard along track from timing point to CARP) -- user input or computed from plotting; placeholder
R.Item26_StopwatchDistance_m = computeStopwatchDistancePlaceholder();

% Item 27: Stopwatch time (Item26 -> time via groundspeed)
R.Item27_StopwatchTime_s = computeStopwatchTime(R.Item26_StopwatchDistance_m, R.Item25_FTD_m, inputs.Drop_IAS_KIAS, R.Item3_DropPressureAlt_ft);

% Item 28: Usable green light length (compute from LAR or usable DZ) - placeholder method using deployed wind effect and LAR dims
R.Item28_UsableGreenLightLength_m = computeUsableGreenLightLength(R.Item24_DeployedDriveDistance_m, R.Item23_DeployedWindEffect_m);

% Item 29: Usable green light time (convert length to time using groundspeed)
R.Item29_UsableGreenLightTime_s = distanceToTime(R.Item28_UsableGreenLightLength_m, inputs.Drop_IAS_KIAS, R.Item3_DropPressureAlt_ft);

% Items 30-33: Usable DZ and timing blocks (map to CARP fields). We'll implement typical conversions:
R.Item30_UsableDZLength_m = computeUsableDZLengthFromDZmetadata();  % TODO: fill with DZ specifics
R.Item31_UsableDZTime_s = distanceToTime(R.Item30_UsableDZLength_m, inputs.Drop_IAS_KIAS, R.Item3_DropPressureAlt_ft);

% Item 34: Red light time (stopwatch time + usable DZ time)
R.Item32_RedLightTime_s = R.Item27_StopwatchTime_s + R.Item31_UsableDZTime_s;

% Items 35-55: In-flight winds, GS, drift, green-light timing, red-light timing, notes, etc.
% We'll gather and compute commonly used ones:
R.Item33_BallisticWind_overall = ballisticWind(inputs.Winds, R.Item9_StabilizationAlt_ft, 0, 'Total');
R.Item34_Groundspeed_kt = estimateGroundspeed(inputs.Drop_IAS_KIAS, R.Item3_DropPressureAlt_ft, inputs.Drop_Altitude_TempC);
R.Item35_Drift_correction_deg = computeDriftCorrection(R.Item33_BallisticWind_overall.dir_deg, inputs.DZ_course_true_deg);

% Items 36-55: These are primarily operational fields (green/red method, TOT, formation, raw CE, corrected CE, etc.)
% We'll populate with computed or placeholder values and guidance for replacement with in-flight measurements:
R.Item36_GreenLightTimeMethod = 'LAR/Computed'; % e.g., 'LAR' or 'Timing point'
R.Item37_TOT = datetime('now'); % actual TOT; placeholder
R.Item38_FormationPosition = 'SS'; % single ship placeholder
R.Item39_RawCircularError_yards = NaN; % requires post-flight score
R.Item40_CorrectedCircularError_yards = NaN; % post-flight corrections
R.Item41_Notes = 'Replace MB-4 table lookups with exact MB-4 readouts where TODO flagged.';

% Fill remaining items with NaNs and comments
for k = 42:55
    R.(['Item' num2str(k)]) = NaN;
end

%% ----------------------- DISPLAY / SAVE -----------------------
disp('=== AF Form 4015 computed items (selected) ===');
disp(sprintf('1 Drop Indicated True Alt (ft): %.0f', R.Item1_DropIndicatedTrueAlt_ft));
disp(sprintf('2 PAV (ft): %.0f', R.Item2_PAV_ft));
disp(sprintf('3 Drop Pressure Alt (ft): %.0f', R.Item3_DropPressureAlt_ft));
disp(sprintf('5 Drop True Alt (ft): %.0f', R.Item5_DropTrueAlt_ft));
disp(sprintf('7 Drop Absolute Alt (ft AGL): %.0f', R.Item7_DropAbsoluteAlt_ft));
disp(sprintf('9 Stabilization Alt (ft AGL): %.0f', R.Item9_StabilizationAlt_ft));
disp(sprintf('11b Actuation True Alt (ft MSL): %.0f', R.Item11b_ActuationIndicatedTrueAlt_ft));
disp(sprintf('14 HV Mid Pressure Alt (ft): %.0f', R.Item14_HV_MidPresAlt_ft));
disp(sprintf('15 HV ARoF (ft/s): %.2f', R.Item15_HV_ARoF_fps));
disp(sprintf('16 HV ToF (s): %.2f', R.Item16_HV_ToF_s));
disp(sprintf('17 HV Drift (m): %.2f', R.Item17_HV_Drift_m));
disp(sprintf('21 Total Deployed Time (s): %.2f', R.Item21_TotalDeployedTime_s));
disp(sprintf('23 Deployed Wind Effect (m): %.2f', R.Item23_DeployedWindEffect_m));
disp(sprintf('24 Deployed Drive Distance (m) (estimate): %.2f', R.Item24_DeployedDriveDistance_m));
disp(sprintf('25 Forward Travel Distance (m): %.2f', R.Item25_FTD_m));
disp('Note: Items with TODO require MB-4/ballistic table inputs for exact operational use.');


%%----------------- NOTES-----------------------------
%Items 35–55 on AF Form 4015 are largely mission/administration fields 
% (TOT, formation, in-flight actual winds, green/red light times, CE records, remarks)
%  The script provides computed defaults or NaNs as placeholders
%  Post-flight entries (drop scores) must be entered after mission.