#include "grid.hpp"

double grid_width = 0.0;
double grid_height = 0.0;
double rect_width = 0.0;
double rect_height = 0.0;
double meters_per_px = 0.0;

void grid_sizing() {
    // Convert satellite camera field of view to geographic distances (meters) using flat Earth approximation
    double alpha = VERTICAL_FOV * (PI/180.0);
    double beta  = HORIZONTAL_FOV * (PI/180.0);

    double width_m  = 2.0 * ALTITUDE * tan(0.5 * beta);
    double height_m = 2.0 * ALTITUDE * tan(0.5 * alpha);

    double m_per_deg_lat = 111320.0;
    double m_per_deg_lon = 111320.0 * cos(CENTER_LAT * (PI/180.0));

    // Split satellite image into a number of evenly sized rectangles such that different groups of pixels are grouped into one grid rectangle
    rect_width  = (width_m  / m_per_deg_lon) / (double)IMAGE_WIDTH;
    rect_height = (height_m / m_per_deg_lat) / (double)IMAGE_HEIGHT;

    grid_width  = (double)IMAGE_WIDTH;
    grid_height = (double)IMAGE_HEIGHT;

    // Calculate conversion scale for satellite image meters-per-pixel
    meters_per_px = height_m / grid_height;
}

void print_scale(cv::Mat& img_sat, double m_per_px, double length_m) {
    // Set scale parameters based on desired scale distances
    int margin = 200;
    int thickness = 30;
    int bar_length = (int)(std::lround(length_m / m_per_px));
    int bar_piece  = bar_length / 4;

    // Define scale colors
    cv::Scalar black(0, 0, 0);
    cv::Scalar white(255, 255, 255);
    cv::Scalar grey(200, 200, 200);

    // Compute and print length of each scale bar
    cv::Point origin(margin, margin);
    int padding = 40;
    cv::rectangle(img_sat,
        cv::Point(origin.x - padding, origin.y - thickness - 60),
        cv::Point(origin.x + bar_length + padding + 60, origin.y + thickness + 60),
        grey,
        cv::FILLED);

    for (int i = 0; i < 4; i++) {
        int x1 = origin.x + i * bar_piece;
        int x2 = origin.x + (i + 1) * bar_piece;
        int y1 = origin.y - thickness / 2;
        int y2 = origin.y + thickness / 2;

        cv::Scalar color = (i % 2 == 0) ? white : black;
        cv::rectangle(img_sat, cv::Point(x1, y1), cv::Point(x2, y2), color, cv::FILLED);
    }

    // Print corresponding distance text above each scale bar
    for (int i = 0; i <= 4; i++) {
        cv::Point tick = origin + cv::Point(i * bar_piece, 0);

        int label_val = (int)(std::lround(i * length_m / 4.0));
        cv::putText(img_sat,
                    std::to_string(label_val),
                    tick + cv::Point(-10, -20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1,
                    black,
                    2,
                    cv::LINE_AA);
    }

    cv::putText(img_sat,
                "METERS",
                origin + cv::Point(bar_length / 2 - 30, thickness + 35),
                cv::FONT_HERSHEY_SIMPLEX,
                1,
                black,
                2,
                cv::LINE_AA);
}

cv::Point2d get_coordinates(cv::Mat& p_cam, cv::Mat& s_cam) {
    // Exit early if either image is empty
    if (p_cam.empty() || s_cam.empty()) return {NAN, NAN};
    if (p_cam.rows > s_cam.rows || p_cam.cols > s_cam.cols) return {NAN, NAN};
    if (p_cam.type() != s_cam.type()) return {NAN, NAN};
    
    cv::Mat final;
    cv::Point min_loc, max_loc;
    cv::Point2d center, pcam_grid, closest_coord;
    double min_val, max_val;

    // Warp and match payload camera image to satellite image, identify strongest feature match between images
    cv::matchTemplate(s_cam, p_cam, final, cv::TM_CCORR_NORMED, cv::noArray());
    cv::minMaxLoc(final, &min_val, &max_val, &min_loc, &max_loc, cv::noArray());

    // Find the center pixel of payload camera image
    center.x = (max_loc.x + (0.5)*p_cam.cols);
    center.y = (max_loc.y + (0.5)*p_cam.rows);
    
    // Find corresponding grid point of payload camera image center
    double grid_per_px_x = (double)grid_width/s_cam.cols;
    double grid_per_px_y = (double)grid_height/s_cam.rows;
    pcam_grid.x = floor(center.y*grid_per_px_y);
    pcam_grid.y = floor(center.x*grid_per_px_x);

    // Calculate the closest satellite image coordinate to payload camera image center
    closest_coord = grid_to_coord(pcam_grid);

    return closest_coord;
}

cv::Point2d grid_to_coord(cv::Point2d grid_point) {
    // Compute half-extent of the grid in degrees from the center
    double delta_lat = 0.5 * grid_height * rect_height;
    double delta_lon = 0.5 * grid_width  * rect_width;

    // Compute top-left corner of the grid in geographic coordinates
    double lat_top_left = CENTER_LAT + delta_lat;
    double lon_top_left = CENTER_LON - delta_lon;

    // Map grid indices to coordinate latitude and longitude
    cv::Point2d coord;
    coord.x = lat_top_left - grid_point.x * rect_height; // latitude decreases downward
    coord.y = lon_top_left + grid_point.y * rect_width;  // longitude increases rightward

    return coord;
}

cv::Point2d coord_to_grid(cv::Point2d coord_point) {
    // Compute half-extent of the grid in degrees from the center
    double delta_lat = 0.5 * grid_height * rect_height;
    double delta_lon = 0.5 * grid_width  * rect_width;

    // Compute top-left corner of the grid in geographic coordinates
    double lat_top_left = CENTER_LAT + delta_lat;
    double lon_top_left = CENTER_LON - delta_lon;

    // Map coordinate latitude and longitude back to grid indices
    cv::Point2d grid;
    grid.x = (lat_top_left - coord_point.x) / rect_height; // vertical index
    grid.y = (coord_point.y - lon_top_left) / rect_width;  // horizontal index

    return grid;
}