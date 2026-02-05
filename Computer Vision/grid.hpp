#include "libraries.hpp"

#define VERTICAL_FOV 
#define HORIZONTAL_FOV
#define ALTITUDE

#define GRID_WIDTH 23305
#define GRID_HEIGHT 9296

#define RECT_WIDTH 0.000004
#define RECT_HEIGHT 0.000004

#define CENTER_LAT 42.273611
#define CENTER_LON -71.809444

cv::Point2d get_coordinates(cv::Mat&, cv::Mat&);
cv::Point2d grid_to_coord(cv::Point2d);
cv::Point2d coord_to_grid(cv::Point2d);

