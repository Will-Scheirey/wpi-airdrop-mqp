clc; clear variables; close all;

lat0 = 42.273611;
lon0 = -71.809444;

delta_lon = 6767.371657;
delta_lat = 12553.954070;

[lat1, lon1] = offsetFeetToLLA(lat0, lon0, delta_lon, 0);
[lat2, lon2] = offsetFeetToLLA(lat0, lon0, 0, delta_lat);
[lat3, lon3] = offsetFeetToLLA(lat0, lon0, -delta_lon, 0);
[lat4, lon4] = offsetFeetToLLA(lat0, lon0, 0, -delta_lat);

[lat5, lon5] = offsetFeetToLLA(lat0, lon0, delta_lon, delta_lat);
[lat6, lon6] = offsetFeetToLLA(lat0, lon0, -delta_lon, delta_lat);
[lat7, lon7] = offsetFeetToLLA(lat0, lon0, -delta_lon, -delta_lat);
[lat8, lon8] = offsetFeetToLLA(lat0, lon0, delta_lon, -delta_lat);

lats = [lat1 lat2 lat3 lat4 lat5 lat6 lat7 lat8];
lons = [lon1 lon2 lon3 lon4 lon5 lon6 lon7 lon8];

for i = 1:4
    fprintf("i=%d\tLat: %0.6f\tLon: %0.6f\n", i, lats(i), lons(i))
end
