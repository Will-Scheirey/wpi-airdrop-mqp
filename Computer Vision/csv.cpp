#include "libraries.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>

/*
***************************************************************************************************
LOAD_COORDS_CSV Reads GPS coordinates and timestamps from CSV into vectors for use

INPUTS:
    csv_path        :   Path to CSV file containing GPS coordinates
    all_coords      :   Vector for storing loaded GPS coordinates
    all_timestamps  :   Vector for storing loaded timestamps
OUTPUTS:
    bool    :   Indicates successful/unsuccessful CSV conversion
***************************************************************************************************
*/
bool load_coords_csv(const std::string& csv_path, std::vector<cv::Point2d>& all_coords, std::vector<double>* all_timestamps=nullptr) {
    std::ifstream data_in(csv_path);
    if (!data_in.is_open()) {
        std::cerr << "Failed to open CSV: " << csv_path << "\n";
        return false;
    }

    all_coords.clear();
    if (all_timestamps) all_timestamps->clear();

    // Read header of CSV file (containing variable titles)
    std::string line;
    std::getline(data_in, line);

    // Read GPS coordinates line-by-line from CSV file
    while (std::getline(data_in, line)) {
        if (line.empty()) continue;

        std::stringstream value(line);
        std::string timestamp, latitude, longitude;

        // Read timestamp, latitude, and longitude for each line of CSV
        if (!std::getline(value, timestamp, ',')) continue;
        if (!std::getline(value, latitude, ',')) continue;
        if (!std::getline(value, longitude, ',')) continue;
        
        // Store zeros in GPS coordinate vector if CSV input is invalid
        if (latitude == "NaN" || longitude == "NaN") {
            cv::Point2d zeros(0.0, 0.0);
            all_coords.push_back(zeros);
        } // Store GPS coordinates from CSV into coordinate vector
        else {
            cv::Point2d point(std::stod(latitude), std::stod(longitude));
            all_coords.push_back(point);
        }

        // If timestamps are being stored, add CSV timestamp to timestamp vector
        if (all_timestamps) all_timestamps->push_back(std::stod(timestamp));
    }
    return true;
}