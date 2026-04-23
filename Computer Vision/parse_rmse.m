baseDir = '/Users/paigerust/Desktop/MQP/haars_videos';

folders = dir(baseDir);
folders = folders([folders.isdir]);
folders = folders(~ismember({folders.name},{'.','..'}));

data = cell(length(folders), 2);

for i = 1:length(folders)
    folderPath = fullfile(baseDir, folders(i).name);
    filePath = fullfile(folderPath, 'rms_out.csv');
    
    if isfile(filePath)
        x = readmatrix(filePath);
        data{i,1} = folders(i).name;
        data{i,2} = x;
    else
        warning('rms_out.csv not found in %s', folders(i).name);
        data{i,1} = folders(i).name;
        data{i,2} = [];
    end
end

T = cell2table(data, 'VariableNames', {'FolderName','RMS_Data'});

figure; hold on

for i = 1:height(T)
    rmse = T.RMS_Data{i};
    
    t = linspace(0,1,length(rmse)); % normalize time
    plot(t, rmse, 'Color', [0.7 0.7 0.7]); % light gray
end

xlabel('Normalized Time')
ylabel('RMSE (m)')
title('RMSE vs Normalized Time Across Flights')

meanRMSE = cellfun(@mean, T.RMS_Data);

figure
plot(meanRMSE, '-o')
xticks(1:height(T))
xticklabels(T.FolderName)
xtickangle(45)

ylabel('Average RMSE (m)')
title('RMSE per Flight')

allVals = [];
allGroups = [];

for i = 1:height(T)
    rmse = T.RMS_Data{i};
    rmse = rmse(:);  % make sure column vector
    
    allVals = [allVals; rmse];
    allGroups = [allGroups; repmat(i, length(rmse), 1)];
end

figure
boxplot(allVals, allGroups, 'Labels', T.FolderName)
xtickangle(45)
ylabel('RMSE (m)')
title('RMSE Distribution per Flight')

nFlights = height(T);

meanRMSE   = zeros(nFlights,1);
medianRMSE = zeros(nFlights,1);
stdRMSE    = zeros(nFlights,1);
q25        = zeros(nFlights,1);
q75        = zeros(nFlights,1);

for i = 1:nFlights
    x = T.RMS_Data{i};
    meanRMSE(i)   = mean(x);
    medianRMSE(i) = median(x);
    stdRMSE(i)    = std(x);
    q25(i) = prctile(x,25);
    q75(i) = prctile(x,75);
end

figure
hold on

errLow  = medianRMSE - q25;
errHigh = q75 - medianRMSE;

errorbar(1:nFlights, medianRMSE, errLow, errHigh, 'o-', 'LineWidth', 1.5)

xticks(1:nFlights)
xticklabels(T.FolderName)
xtickangle(45)
ylabel('RMSE (m)')
title('Median RMSE per Flight with IQR')
grid on

[sortedMedian, idx] = sort(medianRMSE);
sortedNames = T.FolderName(idx);

figure
bar(sortedMedian)
xticks(1:nFlights)
xticklabels(sortedNames)
xtickangle(45)
ylabel('Median RMSE (m)')
title('Flights Sorted by Median RMSE')
grid on

t_common = linspace(0, 1, 100);  % 100 points (adjust as needed)
nFlights = height(T);
M = nan(length(t_common), nFlights);

for i = 1:nFlights
    rmse = T.RMS_Data{i};
    
    t = linspace(0,1,length(rmse));  % original normalized time
    
    % interpolate onto common grid
    M(:,i) = interp1(t, rmse, t_common, 'linear', 'extrap');
end
meanRMSE = mean(M, 2, 'omitnan');
stdRMSE  = std(M, 0, 2, 'omitnan');
figure
hold on

% shaded region
fill([t_common fliplr(t_common)], ...
     [meanRMSE'+stdRMSE' fliplr(meanRMSE'-stdRMSE')], ...
     [0.8 0.8 1], 'EdgeColor', 'none')

% mean line
plot(t_common, meanRMSE, 'b', 'LineWidth', 2)

xlabel('Normalized Time')
ylabel('RMSE (m)')
title('Mean RMSE Across Flights (±1 STD)')
grid on