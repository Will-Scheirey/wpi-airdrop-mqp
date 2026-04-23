%% analyze_true_vs_estimated_position.m
% Compares true GPS position vs estimated position for multiple flights.
%
% True GPS files:
%   /Users/paigerust/wpi-airdrop-mqp/Computer Vision/logs/gps_{flight_name}.csv
%
% Estimated files:
%   /Users/paigerust/Desktop/MQP/haars_flights/{flight_name}/coords_out.csv
%
% Assumptions:
% 1. Each true GPS file contains latitude and longitude columns.
% 2. Each estimated coords_out.csv contains latitude and longitude columns.
% 3. If the two files have different lengths, both are resampled onto a
%    normalized time axis before comparison.
%
% Outputs:
% - Per-flight mean error, RMSE, max error, and sample count
% - Summary table
% - Several plots for inspection
%
% Notes:
% - Edit the latitude/longitude column detection section if your headers
%   are unusual.
% - Distance error is computed using the haversine formula in meters.

clear; clc; close all;

%% Paths
trueBaseDir = '/Users/paigerust/wpi-airdrop-mqp/Computer Vision/logs';
estBaseDir  = '/Users/paigerust/Desktop/MQP/haars_videos';

%% Settings
N_COMMON = 500;              % number of normalized samples for resampling
MAKE_PLOTS_PER_FLIGHT = false;

%% Get flight folders
flightDirs = dir(estBaseDir);
flightDirs = flightDirs([flightDirs.isdir]);
flightDirs = flightDirs(~ismember({flightDirs.name}, {'.','..'}));

%% Storage
results = {};
allErrCurves = {};
allFlightNames = {};

fprintf('Analyzing %d flight folders...\n\n', numel(flightDirs));

for i = 1:numel(flightDirs)
    flightName = flightDirs(i).name;

    estFile  = fullfile(estBaseDir, flightName, 'coords_in.csv');
    trueFile = fullfile(trueBaseDir, ['gps_' flightName '.csv']);

    fprintf('Flight: %s\n', flightName);

    if ~isfile(estFile)
        fprintf('  Missing estimated file: %s\n\n', estFile);
        continue;
    end

    if ~isfile(trueFile)
        fprintf('  Missing true GPS file: %s\n\n', trueFile);
        continue;
    end

    try
        %% Read files
        T_est  = readtable(estFile);
        T_true = readtable(trueFile);

        %% Extract lat/lon columns
        [latEst, lonEst, estLatName, estLonName]     = extractLatLon(T_est);
        [latTrue, lonTrue, trueLatName, trueLonName] = extractLatLon(T_true);

        fprintf('  Estimated columns: %s, %s\n', estLatName, estLonName);
        fprintf('  True columns:      %s, %s\n', trueLatName, trueLonName);

        %% Clean invalid rows
        goodEst  = isfinite(latEst)  & isfinite(lonEst);
        goodTrue = isfinite(latTrue) & isfinite(lonTrue);

        latEst = latEst(goodEst);
        lonEst = lonEst(goodEst);

        latTrue = latTrue(goodTrue);
        lonTrue = lonTrue(goodTrue);

        if isempty(latEst) || isempty(latTrue)
            fprintf('  No valid lat/lon data after cleaning.\n\n');
            continue;
        end

        %% Resample onto common normalized axis if lengths differ
        tCommon = linspace(0, 1, N_COMMON)';

        tEst  = linspace(0, 1, numel(latEst))';
        tTrue = linspace(0, 1, numel(latTrue))';

        latEstI  = interp1(tEst,  latEst,  tCommon, 'linear', 'extrap');
        lonEstI  = interp1(tEst,  lonEst,  tCommon, 'linear', 'extrap');
        latTrueI = interp1(tTrue, latTrue, tCommon, 'linear', 'extrap');
        lonTrueI = interp1(tTrue, lonTrue, tCommon, 'linear', 'extrap');

        %% Compute position error metrics
        errMeters = haversineMeters(latTrueI, lonTrueI, latEstI, lonEstI);

        dLatDeg = latEstI - latTrueI;
        dLonDeg = lonEstI - lonTrueI;

        meanErr = mean(errMeters, 'omitnan');
        rmseErr = sqrt(mean(errMeters.^2, 'omitnan'));
        maxErr  = max(errMeters, [], 'omitnan');
        medErr  = median(errMeters, 'omitnan');
        stdErr  = std(errMeters, 'omitnan');

        %% Save results
        results(end+1, :) = { ...
            flightName, ...
            numel(errMeters), ...
            meanErr, ...
            medErr, ...
            rmseErr, ...
            stdErr, ...
            maxErr ...
            }; %#ok<SAGROW>

        allErrCurves{end+1} = errMeters; %#ok<SAGROW>
        allFlightNames{end+1} = flightName; %#ok<SAGROW>

        fprintf('  Mean Error:   %.3f m\n', meanErr);
        fprintf('  Median Error: %.3f m\n', medErr);
        fprintf('  RMSE:         %.3f m\n', rmseErr);
        fprintf('  STD:          %.3f m\n', stdErr);
        fprintf('  Max Error:    %.3f m\n\n', maxErr);

        %% Optional per-flight plots
        if MAKE_PLOTS_PER_FLIGHT
            figure('Name', flightName, 'Color', 'w');

            subplot(2,2,1)
            plot(lonTrueI, latTrueI, 'k-', 'LineWidth', 1.5); hold on;
            plot(lonEstI,  latEstI,  'r-', 'LineWidth', 1.2);
            xlabel('Longitude (deg)')
            ylabel('Latitude (deg)')
            title([flightName ' Trajectory'])
            legend('True','Estimated','Location','best')
            grid on
            axis equal

            subplot(2,2,2)
            plot(tCommon, errMeters, 'b', 'LineWidth', 1.2)
            xlabel('Normalized Time')
            ylabel('Position Error (m)')
            title('Position Error vs Normalized Time')
            grid on

            subplot(2,2,3)
            plot(tCommon, dLatDeg, 'LineWidth', 1.2)
            xlabel('Normalized Time')
            ylabel('\Delta Latitude (deg)')
            title('Latitude Error')
            grid on

            subplot(2,2,4)
            plot(tCommon, dLonDeg, 'LineWidth', 1.2)
            xlabel('Normalized Time')
            ylabel('\Delta Longitude (deg)')
            title('Longitude Error')
            grid on
        end

    catch ME
        fprintf('  Error processing %s\n', flightName);
        fprintf('  %s\n\n', ME.message);
        continue;
    end
end

%% Build results table
if isempty(results)
    error('No flights were successfully processed.');
end

summaryTable = cell2table(results, 'VariableNames', ...
    {'FlightName','NSamples','MeanError_m','MedianError_m','RMSE_m','StdError_m','MaxError_m'});

disp(summaryTable)

%% Save summary table
outCSV = fullfile(estBaseDir, 'position_error_summary.csv');
writetable(summaryTable, outCSV);
fprintf('\nSaved summary table to:\n%s\n', outCSV);

%% Sort by RMSE
summarySorted = sortrows(summaryTable, 'RMSE_m');
disp('Flights sorted by RMSE:')
disp(summarySorted(:, {'FlightName','RMSE_m','MeanError_m','MaxError_m'}))

%% Plot 1: RMSE per flight
figure('Color', 'w');
plot(1:height(summaryTable), summaryTable.RMSE_m, '-o', 'LineWidth', 1.5)
xticks(1:height(summaryTable))
xticklabels(summaryTable.FlightName)
xtickangle(45)
ylabel('RMSE (m)')
title('Position RMSE per Flight')
grid on

%% Plot 2: Mean error per flight
figure('Color', 'w');
plot(1:height(summaryTable), summaryTable.MeanError_m, '-o', 'LineWidth', 1.5)
xticks(1:height(summaryTable))
xticklabels(summaryTable.FlightName)
xtickangle(45)
ylabel('Mean Position Error (m)')
title('Mean Position Error per Flight')
grid on

%% Plot 3: Sorted RMSE bar chart
figure('Color', 'w');
bar(summarySorted.RMSE_m)
xticks(1:height(summarySorted))
xticklabels(summarySorted.FlightName)
xtickangle(45)
ylabel('RMSE (m)')
title('Flights Sorted by Position RMSE')
grid on

%% Plot 4: Mean error curve across flights on normalized time axis
maxLen = max(cellfun(@numel, allErrCurves));
M = nan(maxLen, numel(allErrCurves));

for i = 1:numel(allErrCurves)
    x = allErrCurves{i};
    M(1:numel(x), i) = x;
end

meanErrCurve = mean(M, 2, 'omitnan');
stdErrCurve  = std(M, 0, 2, 'omitnan');
tPlot = linspace(0, 1, maxLen)';

lower = max(meanErrCurve - stdErrCurve, 0);
upper = meanErrCurve + stdErrCurve;

figure('Color', 'w'); hold on;
fill([tPlot; flipud(tPlot)], [upper; flipud(lower)], ...
    [0.85 0.85 1], 'EdgeColor', 'none');
plot(tPlot, meanErrCurve, 'b', 'LineWidth', 2)
xlabel('Normalized Time')
ylabel('Position Error (m)')
title('Mean Position Error Across Flights (\pm 1 STD)')
grid on

%% Plot 5: Histogram of per-flight RMSE
figure('Color', 'w');
histogram(summaryTable.RMSE_m)
xlabel('RMSE (m)')
ylabel('Count')
title('Distribution of Flight RMSE Values')
grid on

%% Overall stats
fprintf('\nOverall Summary Across Flights:\n');
fprintf('  Mean of flight RMSE values:   %.3f m\n', mean(summaryTable.RMSE_m, 'omitnan'));
fprintf('  Median of flight RMSE values: %.3f m\n', median(summaryTable.RMSE_m, 'omitnan'));
fprintf('  Std of flight RMSE values:    %.3f m\n', std(summaryTable.RMSE_m, 'omitnan'));
fprintf('  Best flight RMSE:             %.3f m (%s)\n', ...
    min(summaryTable.RMSE_m), summaryTable.FlightName{find(summaryTable.RMSE_m == min(summaryTable.RMSE_m), 1)});
fprintf('  Worst flight RMSE:            %.3f m (%s)\n', ...
    max(summaryTable.RMSE_m), summaryTable.FlightName{find(summaryTable.RMSE_m == max(summaryTable.RMSE_m), 1)});

%% -------- Local functions --------

function [lat, lon, latName, lonName] = extractLatLon(T)
% Attempts to find latitude and longitude columns from a table.

    varNames = T.Properties.VariableNames;
    varNamesLower = lower(varNames);

    latCandidates = find(contains(varNamesLower, 'latitude'));
    lonCandidates = find(contains(varNamesLower, 'longitude'));

    if isempty(latCandidates) || isempty(lonCandidates)
        error('Could not automatically find latitude/longitude columns.');
    end

    latIdx = latCandidates(1);
    lonIdx = lonCandidates(1);

    latName = varNames{latIdx};
    lonName = varNames{lonIdx};

    lat = T.(latName);
    lon = T.(lonName);

    if istable(lat), lat = table2array(lat); end
    if istable(lon), lon = table2array(lon); end

    lat = double(lat(:));
    lon = double(lon(:));
end

function d = haversineMeters(lat1, lon1, lat2, lon2)
% Computes great-circle distance between two lat/lon vectors in meters.

    R = 6371000; % Earth radius in meters

    lat1 = deg2rad(lat1);
    lon1 = deg2rad(lon1);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);

    dlat = lat2 - lat1;
    dlon = lon2 - lon1;

    a = sin(dlat/2).^2 + cos(lat1).*cos(lat2).*sin(dlon/2).^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));

    d = R * c;
end