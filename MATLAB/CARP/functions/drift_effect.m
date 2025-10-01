function drift = drift_effect(ballistic_wind, total_time_of_fall)
drift = (ballistic_wind * total_time_of_fall)/1.78;%1.78 is used to convert from knots to yards/s
end