#include "grid.hpp"

cv::Point2d get_coordinates(cv::Mat& p_cam, cv::Mat& s_cam) {
    cv::Mat final;
    cv::Point min_loc, max_loc;
    cv::Point2d center, pcam_grid, closest_coord;
    double min_val, max_val;

    cv::matchTemplate(s_cam, p_cam, final, cv::TM_CCORR_NORMED, cv::noArray());
    cv::minMaxLoc(final, &min_val, &max_val, &min_loc, &max_loc, cv::noArray());

    center.x = (max_loc.x + (0.5)*p_cam.cols);
    center.y = (max_loc.y + (0.5)*p_cam.rows);
    
    double grid_per_px_x = (double)GRID_WIDTH/s_cam.cols;
    double grid_per_px_y = (double)GRID_HEIGHT/s_cam.rows;

    pcam_grid.x = floor(center.y*grid_per_px_y);
    pcam_grid.y = floor(center.x*grid_per_px_x);

    closest_coord = grid_to_coord(pcam_grid);

    return closest_coord;
}

cv::Point2d grid_to_coord(cv::Point2d grid_point) {
    double delta_lat = (0.5)*GRID_HEIGHT*RECT_HEIGHT;
    double delta_lon = (0.5)*GRID_WIDTH*RECT_WIDTH;

    double lat_top_left = CENTER_LAT + delta_lat;
    double lon_top_left = CENTER_LON - delta_lon;

    cv::Point2d closest_coord;

    closest_coord.x = lat_top_left - grid_point.x*RECT_HEIGHT;
    closest_coord.y = lon_top_left + grid_point.y*RECT_WIDTH;

    return closest_coord;
}

// cv::Point2d coord_to_grid(cv::Point2d coord_point) {
//     double delta_lat = (0.5)*GRID_HEIGHT*RECT_WIDTH;
//     double delta_lon = (0.5)*GRID_WIDTH*RECT_HEIGHT;

//     double lat_top_left = CENTER_LAT + delta_lat;
//     double lon_top_left = CENTER_LON - delta_lon;

//     cv::Point2d closest_grid;

//     closest_grid.x = (lat_top_left - coord_point.x) / RECT_HEIGHT;
//     closest_grid.y = (lon_top_left + coord_point.y) / RECT_WIDTH;

//     return closest_grid;
// }

cv::Point2d coord_to_grid(cv::Point2d coord_point) {
    // must match grid_to_coord() exactly
    double delta_lat = 0.5 * GRID_HEIGHT * RECT_HEIGHT;
    double delta_lon = 0.5 * GRID_WIDTH  * RECT_WIDTH;

    double lat_top_left = CENTER_LAT + delta_lat;
    double lon_top_left = CENTER_LON - delta_lon;

    cv::Point2d grid;
    grid.x = (lat_top_left - coord_point.x) / RECT_HEIGHT;   // row-like
    grid.y = (coord_point.y - lon_top_left) / RECT_WIDTH;    // col-like

    return grid;
}


// void grid_sizing() {
//     double alpha = VERTICAL_FOV*(PI/180.0);
//     double beta = HORIZONTAL_FOV*(PI/180.0);

//     double delta_x = ALTITUDE*tan(0.5*alpha);
//     double delta;

// }