function alt = corrected_drop_altitude(drop_altitude,true_altitude_temperature, pressure_altitude)
    alt = (drop_altitude*pressure_altitude)/true_altitude_temperature ;
end