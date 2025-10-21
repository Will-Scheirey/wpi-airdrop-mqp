function fg_sim(t, lla, rpy)

ts = timeseries([lla, rpy], t);

h = Aero.FlightGearAnimation;

h.TimeseriesSourceType = 'Timeseries';
h.TimeseriesSource = ts;

h.FlightGearBaseDirectory = '/Applications/FlightGear.app';
h.GeometryModelName = 'Parachutist';
h.DestinationIpAddress = '127.0.0.1';
h.DestinationPort = '5502';

h.AirportId = 'KSFO';
h.RunwayId = '10L';
h.InitialAltitude = 7224;
h.InitialHeading = 113;
h.OffsetDistance = 4.72;
h.OffsetAzimuth = 0;

h.InstallScenery = true;
h.DisableShaders = true;
h.TimeScaling = 5;

setenv('FG_ROOT', '/Applications/FlightGear.app/Contents/Resources/data');

h.OutputFileName = 'runfg.sh';

get(h)

GenerateRunScript(h)

% --- Patch the launcher to use the correct executable and FG_ROOT ---
fid = fopen('runfg.sh','r');
txt = fread(fid,'*char')';
fclose(fid);

% Replace the broken fgfs line with the correct one
txt = regexprep(txt, '\./\.\./MacOS/fgfs', ...
    '/Applications/FlightGear.app/Contents/MacOS/FlightGear');

% Add FG_ROOT if it isn't already there
if ~contains(txt, 'export FG_ROOT')
    txt = sprintf('#!/bin/zsh\nexport FG_ROOT="/Applications/FlightGear.app/Contents/Resources/data"\n%s', txt);
end

fid = fopen('runfg.sh','w');
fwrite(fid, txt);
fclose(fid);

% --------------------------------------------------------------

system('bash runfg.sh &');
pause(5);
play(h);

end