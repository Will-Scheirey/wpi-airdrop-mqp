#include "libraries.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>

bool load_coords_csv(const std::string& csv_path, std::vector<cv::Point2d>& all_coords, std::vector<double>* all_timestamps=nullptr) {
    std::ifstream data_in(csv_path);
    if (!data_in.is_open()) {
        std::cerr << "Failed to open CSV: " << csv_path << "\n";
        return false;
    }

    all_coords.clear();
    if (all_timestamps) all_timestamps->clear();

    std::string line;

    // Read header
    std::getline(data_in, line);

    // Read data lines
    while (std::getline(data_in, line)) {
        if (line.empty()) continue;

        std::stringstream value(line);
        std::string timestamp, latitude, longitude;

        if (!std::getline(value, timestamp, ',')) continue;
        if (!std::getline(value, latitude, ',')) continue;
        if (!std::getline(value, longitude, ',')) continue;
        
        if (latitude == "NaN" || longitude == "NaN") {
            cv::Point2d zeros(0.0, 0.0);
            all_coords.push_back(zeros);
        } else {
            cv::Point2d point(std::stod(latitude), std::stod(longitude));
            all_coords.push_back(point);
        }

        if (all_timestamps) all_timestamps->push_back(std::stod(timestamp));
    }
    return true;
}