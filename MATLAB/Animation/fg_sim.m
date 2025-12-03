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
% 
% GenerateRunScript(h)
% 
% system('runfg.bat &');

end