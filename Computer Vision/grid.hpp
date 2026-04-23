// H_FOV = 2arctan(tan(V_FOV/2) * AR)

#include "libraries.hpp"

// #define IMAGE_WIDTH 2585
// #define IMAGE_HEIGHT 1491

// #define VERTICAL_FOV 55
// #define HORIZONTAL_FOV 84.1428
// #define ALTITUDE 9144

// #define CENTER_LAT 32.868588
// #define CENTER_LON -114.395957

#define IMAGE_WIDTH 2596
#define IMAGE_HEIGHT 1492
#define VERTICAL_FOV 55
#define HORIZONTAL_FOV 84.338
#define CENTER_LAT 42.274222
#define CENTER_LON -71.797367
#define ALTITUDE 6096

extern double grid_width;
extern double grid_height;
extern double rect_width;
extern double rect_height;
extern double meters_per_px;

void grid_sizing();
void print_scale(cv::Mat&, double, double);
cv::Point2d get_coordinates(cv::Mat&, cv::Mat&);
cv::Point2d grid_to_coord(cv::Point2d);
cv::Point2d coord_to_grid(cv::Point2d);

