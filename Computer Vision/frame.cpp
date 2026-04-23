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

void add_marker(cv::Mat& img_sat, const cv::Point2d& est_coord, const bool& have_truth, const cv::Point2d& rms, const cv::Point2d* true_coord=nullptr) {
    cv::Point2d true_grid;
    int true_px; int true_py;
    cv::Point2d est_grid = coord_to_grid(est_coord);
    if (have_truth) true_grid = coord_to_grid(*true_coord);

    double px_per_grid_x = static_cast<double>(img_sat.cols) / grid_width;
    double px_per_grid_y = static_cast<double>(img_sat.rows) / grid_height;

    int est_px = static_cast<int>(floor(est_grid.y * px_per_grid_x));
    int est_py = static_cast<int>(floor(est_grid.x * px_per_grid_y));
    if (have_truth) true_px = static_cast<int>(floor(true_grid.y * px_per_grid_x));
    if (have_truth) true_py = static_cast<int>(floor(true_grid.x * px_per_grid_y));
    
    int rms_px = static_cast<int>(ceil(rms.x * px_per_grid_y / rect_width));
    int rms_py = static_cast<int>(ceil(rms.y * px_per_grid_x / rect_height));

    if (est_px < 0 || est_px >= img_sat.cols || est_py < 0 || est_py >= img_sat.rows) {
        std::cout << "Marker off-frame; not drawing.\n";
        return;
    }

    print_scale(img_sat, meters_per_px, 2000.0);
    cv::circle(img_sat, cv::Point(est_px, est_py), 20, cv::Scalar(0,255,255), -1);
    if (have_truth) cv::circle(img_sat, cv::Point(true_px, true_py), 20, cv::Scalar(255,0,0), -1);
    cv::ellipse(img_sat, cv::Point(est_px, est_py), cv::Size(rms_py,rms_px), 0, 0, 360, cv::Scalar(0,255,255), 10);
}

void flight_statistics(const cv::Mat& img_sat, cv::VideoWriter& vid_out, const bool& have_truth) {
    if (WINDOW <= 0 || STRIDE <= 0) return;
    const int N = static_cast<int>(all_coords.size());

    if (N < WINDOW) return;

    std::vector<cv::Point2d> rms(N);

    double sum = 0;
    int values = 0;

    for (int index = 0; index + WINDOW <= N; index += STRIDE) {
        cv::Point2d centroid(0.0, 0.0);
        cv::Point2d truth(0.0, 0.0);

        for (int j = 0; j < WINDOW; ++j) {
            centroid += all_coords[index + j];
            if (have_truth) truth += true_coords[index + j];
        }
        centroid *= (1.0 / WINDOW);
        if (have_truth) truth *= (1.0 / WINDOW);

        for (int k = 0; k < WINDOW; ++k) {
            const double dlat = all_coords[index + k].x - centroid.x;
            const double dlon = all_coords[index + k].y - centroid.y;
            rms[index].x += dlat * dlat;
            rms[index].y += dlon * dlon;
        }
        rms[index].x = std::sqrt(rms[index].x / WINDOW);
        rms[index].y = std::sqrt(rms[index].y / WINDOW);

        cv::Mat out = img_sat.clone();

        if (have_truth) add_marker(out, centroid, have_truth, rms[index], &truth);
        else add_marker(out, centroid, have_truth, rms[index], NULL);

        for (int m = 0; m < STRIDE; ++m) {
            vid_out.write(out);
        }

        const cv::Point2d rms_m = degrees_to_m(rms[index]);
        double rms_mag_m = std::sqrt(rms_m.x * rms_m.x + rms_m.y * rms_m.y);
        if ((rms_mag_m > 4000) || (rms_mag_m < 1)) rms_mag_m = 0;
        rms_out << rms_mag_m << std::endl;
        if (rms_mag_m != 0) {
            sum += rms_mag_m;
            values++;
        }
        std::cout << "=========================================================\n";
        std::cout << "Centroid:\t" << centroid << "\n";
        std::cout << "Latitudinal RMS (deg):\t" << rms[index].x << "\t| Latitudinal RMS (m)\t" << rms_m.x << "\n";
        std::cout << "Longitudinal RMS (deg):\t" << rms[index].y << "\t| Longitudinal RMS (m)\t" << rms_m.y << "\n";
    }

    double rms_total_m = sum / values;

    std::cout << "=========================================================\n";
    std::cout << "Average RMS (m):\t" << rms_total_m << "\n";
}

void interpolate(std::vector<cv::Point2d>& all_coords) {
    int i = 0;
    while (i < (int)all_coords.size()) {
        if ((all_coords[i].x == 0 || all_coords[i].y == 0) && (i != 0)) {
            cv::Point2d first_coord = all_coords[i - 1];

            int j = i; int gap = 0;
            while ((j < ((int)all_coords.size() - i)) && (all_coords[j].x == 0 || all_coords[j].y == 0)) {
                j++; 
            }

            cv::Point2d interp(0.0,0.0);
            if (j < ((int)all_coords.size() - i)) {
                cv::Point2d last_coord = all_coords[j];
                gap = j - i + 1;
                interp = (last_coord - first_coord) / gap;
            } else {
                gap = j;
            }

            for (int k = 1; k < gap; k++) {
                all_coords[i + k - 1] = first_coord + k*interp;
            }

            i = j;
        } else {
            i++;
        }
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

