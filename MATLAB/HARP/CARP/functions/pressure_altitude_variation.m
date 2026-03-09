function PAV = pressure_altitude_variation(dz_altimeter, true_altitude)

if dz_altimeter < 29.92
    PAV = 29.92 + true_altitude;
else
    PAV = true_altitude - 29.92;
end