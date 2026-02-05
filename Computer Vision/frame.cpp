#include "frame.hpp"
#include "grid.hpp"

cv::Point2d average_coords(const std::vector<cv::Point2d>& coords) {
    if (coords.empty()) return cv::Point2d(0.0, 0.0);
    cv::Point2d average_coord(0.0, 0.0);

    for (auto& coord : coords) {
        average_coord += coord;
    }

    return average_coord * (1.0 / (double)coords.size());
}

void add_marker(cv::Mat& img_sat, const cv::Point2d& coordinate) {

    cv::Point2d grid = coord_to_grid(coordinate);

    double px_per_grid_x = static_cast<double>(img_sat.cols) / GRID_WIDTH;
    double px_per_grid_y = static_cast<double>(img_sat.rows) / GRID_HEIGHT;

    int px = static_cast<int>(floor(grid.y * px_per_grid_x));
    int py = static_cast<int>(floor(grid.x * px_per_grid_y));

    std::cout << "grid(x,row?)=" << grid.x << " grid(y,col?)=" << grid.y
              << " -> px=" << px << " py=" << py
              << " (cols=" << img_sat.cols << ", rows=" << img_sat.rows << ")\n";

    if (px < 0 || px >= img_sat.cols || py < 0 || py >= img_sat.rows) {
        std::cout << "Marker off-frame; not drawing.\n";
        return;
    }

    cv::circle(img_sat, cv::Point(px, py), 10, cv::Scalar(0,255,255), -1);
}

void flight_statistics() {
    for (const auto& coord : all_coords) {
        centroid += coord;
        num_points++;
    }
    centroid /= num_points;
    for (const cv::Point2d coord : all_coords) {
        double difference_lat = coord.x - centroid.x;
        double difference_lon = coord.y - centroid.y;
        rms_lat += difference_lat * difference_lat;
        rms_lon += difference_lon * difference_lon;
    }
    rms_lat = std::sqrt(rms_lat / num_points);
    rms_lon = std::sqrt(rms_lon / num_points);
}

cv::Point2d degrees_to_m() {
    double dlat = (rms_lat) * DEG2RAD;
    double dlon = (rms_lon) * DEG2RAD;

    double north_m = dlat * R_EARTH;
    double east_m  = dlon * R_EARTH * std::cos(rms_lat);

    cv::Point2d rms_m(north_m, east_m);

    return rms_m;
}

