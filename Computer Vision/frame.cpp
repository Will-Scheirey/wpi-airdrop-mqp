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

void add_marker(cv::Mat& img_sat, const cv::Point2d& coordinate, const cv::Point2d& rms) {

    cv::Point2d grid = coord_to_grid(coordinate);

    double px_per_grid_x = static_cast<double>(img_sat.cols) / GRID_WIDTH;
    double px_per_grid_y = static_cast<double>(img_sat.rows) / GRID_HEIGHT;

    int px = static_cast<int>(floor(grid.y * px_per_grid_x));
    int py = static_cast<int>(floor(grid.x * px_per_grid_y));
    
    int rms_px = static_cast<int>(ceil(rms.x * px_per_grid_y / RECT_HEIGHT));
    int rms_py = static_cast<int>(ceil(rms.y * px_per_grid_x / RECT_WIDTH));

    std::cout << "grid(x,row?)=" << grid.x << " grid(y,col?)=" << grid.y
              << " -> px=" << px << " py=" << py
              << " (cols=" << img_sat.cols << ", rows=" << img_sat.rows << ")\n";

    if (px < 0 || px >= img_sat.cols || py < 0 || py >= img_sat.rows) {
        std::cout << "Marker off-frame; not drawing.\n";
        return;
    }

    cv::circle(img_sat, cv::Point(px, py), 10, cv::Scalar(0,255,255), -1);
    cv::ellipse(img_sat, cv::Point(px, py), cv::Size(rms_py,rms_px), 0, 0, 360, cv::Scalar(0,255,255), 2);
}

// void flight_statistics(int window, cv::Mat& img_sat, cv::VideoWriter& vid_out) {    
//     for (int i = 0; i < floor(all_coords.size()/window); i++) {
//         cv::Point2d rms(0.0,0.0);
//         cv::Point2d centroid(0.0, 0.0);

//         for (int j = 0; j < window; j++) {
//             centroid += all_coords[j*i];
//         }
//         centroid /= window;
//         for (int k = 0; k < window; k++) {
//             double difference_lat = all_coords[k*i].x - centroid.x;
//             double difference_lon = all_coords[k*i].y - centroid.y;

//             rms.x += difference_lat * difference_lat;
//             rms.y += difference_lon * difference_lon;
//         }
//         rms.x = std::sqrt(rms.x / window);
//         rms.y = std::sqrt(rms.y / window);

//         cv::Mat out = img_sat.clone();
//         add_marker(out, centroid, rms);
//         for (int i = 0; i < WINDOW; i++) {
//             vid_out.write(out);
//         }

//         cv::Point2d rms_m = degrees_to_m(rms);
//         std::cout << "=========================================================\n";
//         std::cout << "Centroid:\t" << centroid << "\n";
//         std::cout << "Latitudinal RMS (deg):\t" << rms.x << "\t| Latitudinal RMS (m)\t" << rms_m.x << "\n";
//         std::cout << "Longitudinal RMS (deg):\t" << rms.y << "\t| Longitudinal RMS (m)\t" << rms_m.y << "\n";
//     }
// }

void flight_statistics(const cv::Mat& img_sat, cv::VideoWriter& vid_out) {
    if (WINDOW <= 0 || STRIDE <= 0) return;
    const int N = static_cast<int>(all_coords.size());
    if (N < WINDOW) return;

    for (int index = 0; index + WINDOW <= N; index += STRIDE) {

        cv::Point2d centroid(0.0, 0.0);
        cv::Point2d rms(0.0, 0.0);

        for (int j = 0; j < WINDOW; ++j) {
            centroid += all_coords[index + j];
        }
        centroid *= (1.0 / WINDOW);

        for (int k = 0; k < WINDOW; ++k) {
            const double dlat = all_coords[index + k].x - centroid.x;
            const double dlon = all_coords[index + k].y - centroid.y;
            rms.x += dlat * dlat;
            rms.y += dlon * dlon;
        }
        rms.x = std::sqrt(rms.x / WINDOW);
        rms.y = std::sqrt(rms.y / WINDOW);

        cv::Mat out = img_sat.clone();
        add_marker(out, centroid, rms);

        for (int m = 0; m < STRIDE; ++m) {
            vid_out.write(out);
        }

        const cv::Point2d rms_m = degrees_to_m(rms);
        std::cout << "=========================================================\n";
        std::cout << "Centroid:\t" << centroid << "\n";
        std::cout << "Latitudinal RMS (deg):\t" << rms.x << "\t| Latitudinal RMS (m)\t" << rms_m.x << "\n";
        std::cout << "Longitudinal RMS (deg):\t" << rms.y << "\t| Longitudinal RMS (m)\t" << rms_m.y << "\n";
    }
}

cv::Point2d degrees_to_m(cv::Point2d& rms) {
    double dlat = (rms.x) * DEG2RAD;
    double dlon = (rms.y) * DEG2RAD;

    double north_m = dlat * R_EARTH;
    double east_m  = dlon * R_EARTH * std::cos(rms.x);

    cv::Point2d rms_m(north_m, east_m);

    return rms_m;
}

