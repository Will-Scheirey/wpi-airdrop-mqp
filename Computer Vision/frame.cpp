#include "frame.hpp"
#include "grid.hpp"

cv::Point2d average_coords(const std::vector<cv::Point2d>& coords) {
    // Return 0 if input vector is empty
    if (coords.empty()) return cv::Point2d(0.0, 0.0);
    cv::Point2d average_coord(0.0, 0.0);

    // Summate GPS coordinates in coordinate vector
    for (auto& coord : coords) {
        average_coord += coord;
    }

    // Return avera GPS coordinate
    return average_coord * (1.0 / (double)coords.size());
}

void add_marker(cv::Mat& img_sat, const cv::Point2d& est_coord, const bool& have_truth, const cv::Point2d& rmse, const cv::Point2d* true_coord=nullptr) {
    cv::Point2d true_grid;
    int true_px; int true_py;

    // Convert estimated and ground truth payload GPS coordinates from degrees to pixels
    cv::Point2d est_grid = coord_to_grid(est_coord);
    if (have_truth) true_grid = coord_to_grid(*true_coord);
    double px_per_grid_x = (double)(img_sat.cols) / grid_width;
    double px_per_grid_y = (double)(img_sat.rows) / grid_height;

    // Center estimated and ground truth payload GPS coordinates, RMSE distances relative to satellite image frame
    int est_px = (int)(floor(est_grid.y * px_per_grid_x));
    int est_py = (int)(floor(est_grid.x * px_per_grid_y));
    if (have_truth) true_px = (int)(floor(true_grid.y * px_per_grid_x));
    if (have_truth) true_py = (int)(floor(true_grid.x * px_per_grid_y));
    int rmse_px = (int)(ceil(rmse.x * px_per_grid_y / rect_width));
    int rmse_py = (int)(ceil(rmse.y * px_per_grid_x / rect_height));

    // Print image scale and ground truth payload GPS coordinates to output video
    print_scale(img_sat, meters_per_px, 2000.0);
    if (have_truth) cv::circle(img_sat, cv::Point(true_px, true_py), 20, cv::Scalar(255,0,0), -1);

    // Exit early if estimated payload GPS coordinate is off-screen
    if (est_px < 0 || est_px >= img_sat.cols || est_py < 0 || est_py >= img_sat.rows) {
        std::cout << "Marker off-frame; not drawing." << std::endl;
        return;
    }
    
    // Print estimated payload GPS coordinates and RMSE to output video
    cv::circle(img_sat, cv::Point(est_px, est_py), 20, cv::Scalar(0,255,255), -1);
    cv::ellipse(img_sat, cv::Point(est_px, est_py), cv::Size(rmse_py,rmse_px), 0, 0, 360, cv::Scalar(0,255,255), 10);
}

void flight_statistics(const cv::Mat& img_sat, cv::VideoWriter& vid_out, const bool& have_truth) {
    // If number of coordinates is smaller than window size, exit early
    const int N = (int)(all_coords.size());
    if (N < WINDOW) return;

    std::vector<cv::Point2d> rmse(N);

    // Iterate through each WINDOW-size group of estimated payload GPS coordinates, skipping by STRIDE frames
    for (int index = 0; index + WINDOW <= N; index += STRIDE) {
        cv::Point2d centroid(0.0, 0.0);
        cv::Point2d truth(0.0, 0.0);

        // Calculate window centroid of estimated and ground truth payload GPS coordinates
        for (int j = 0; j < WINDOW; ++j) {
            centroid += all_coords[index + j];
            if (have_truth) truth += true_coords[index + j];
        }
        centroid *= (1.0 / WINDOW);
        if (have_truth) truth *= (1.0 / WINDOW);

        // Calculate window RMSE of estimated payload GPS coordinates
        for (int k = 0; k < WINDOW; ++k) {
            const double dlat = all_coords[index + k].x - centroid.x;
            const double dlon = all_coords[index + k].y - centroid.y;
            rmse[index].x += dlat * dlat;
            rmse[index].y += dlon * dlon;
        }
        rmse[index].x = std::sqrt(rmse[index].x / WINDOW);
        rmse[index].y = std::sqrt(rmse[index].y / WINDOW);

        // Clome satellite image, print window statistics, and write to output video
        cv::Mat out = img_sat.clone();
        if (have_truth) add_marker(out, centroid, have_truth, rmse[index], &truth);
        else add_marker(out, centroid, have_truth, rmse[index], NULL);

        for (int m = 0; m < STRIDE; ++m) {
            vid_out.write(out);
        }

        // Calculate and write average RMSE for windowed coordinates
        const cv::Point2d rmse_m = degrees_to_m(rmse[index]);
        double rmse_mag_m = std::sqrt(rmse_m.x * rmse_m.x + rmse_m.y * rmse_m.y);
        // To prevent erroneous calculations, cap RMSE if value is greater than 4,000 meters or less than 1 meter
        if ((rmse_mag_m > 4000) || (rmse_mag_m < 1)) rmse_mag_m = 0;
        rmse_out << rmse_mag_m << std::endl;


        // Calculate and write average error (ground truth - estimation) for windowed coordinates
        cv::Point2d error(truth.x - centroid.x, truth.y - centroid.y);
        const cv::Point2d error_m = degrees_to_m(error);
        double error_mag_m = std::sqrt(error_m.x * error_m.x + error_m.y * error_m.y);
        if ((error_mag_m > 10000) || (error_mag_m < 1)) error_mag_m = 0;
        error_out << error_mag_m << std::endl;
    }
}

void interpolate(std::vector<cv::Point2d>& all_coords) {
    int i = 0;
    while (i < (int)all_coords.size()) {
        if ((all_coords[i].x == 0 || all_coords[i].y == 0) && (i != 0)) {
            // Record the last GPS coordinate before the gap
            cv::Point2d first_coord = all_coords[i - 1];

            int j = i; int gap = 0;
            while ((j < ((int)all_coords.size() - i)) && (all_coords[j].x == 0 || all_coords[j].y == 0)) {
                j++; 
            }

            // Calculate size of gap + record the first GPS coordinate after the gap
            cv::Point2d interp(0.0,0.0);
            if (j < ((int)all_coords.size() - i)) {
                cv::Point2d last_coord = all_coords[j];
                gap = j - i + 1;
                interp = (last_coord - first_coord) / gap;
            } else {
                gap = j;
            }

            // Fill in gap with interpolated value
            for (int k = 1; k < gap; k++) {
                all_coords[i + k - 1] = first_coord + k*interp;
            }

            i = j;
        } else {
            i++;
        }
    }
}

cv::Point2d degrees_to_m(cv::Point2d& rmse) {
    // Convert GPS coordinates from degrees to radians
    double dlat = (rmse.x) * DEG2RAD;
    double dlon = (rmse.y) * DEG2RAD;

    // Calculate distance in meters using flat Earth approximation
    double north_m = dlat * R_EARTH;
    double east_m  = dlon * R_EARTH * std::cos(rmse.x);

    cv::Point2d rmse_m(north_m, east_m);
    return rmse_m;
}

bool cropped_match(cv::Mat &img) {
    if (img.empty()) return false;

    // Convert input image to grayscale and applies binary mask to image to identify non-image (black) areas
    cv::Mat gray; cv::Mat mask;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);

    const int rows = mask.rows, cols = mask.cols;
    std::vector<int> prev(cols, 0), curr(cols, 0);
    int maxSize = 0, maxRow = 0, maxCol = 0;

    // Increase cropped image frame size for each pixel that is not black
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (mask.at<uchar>(i, j) != 0) {
                curr[j] = (i == 0 || j == 0) ? 1 : std::min({prev[j], curr[j-1], prev[j-1]}) + 1;
                if (curr[j] > maxSize) { maxSize = curr[j]; maxRow = i; maxCol = j; }
            } else {
                curr[j] = 0;
            }
        }
        std::swap(prev, curr);
        std::fill(curr.begin(), curr.end(), 0);
    }

    // If no valid square larger than 20x20 pixels is found, abort crop
    const int minSize = 20;
    if (maxSize < minSize) return false;

    int x = maxCol - maxSize + 1;
    int y = maxRow - maxSize + 1;

    // Clamp cropped image to smallest image bounds
    x = std::max(0, std::min(x, cols - maxSize));
    y = std::max(0, std::min(y, rows - maxSize));

    // Convert image to square and return clone
    cv::Rect square(x, y, maxSize, maxSize);
    img = img(square).clone();
    return true;
}

