#include "libraries.hpp"

//----------GLOBAL VARIABLES----------
// Satellite image characteristics
#define IMAGE_WIDTH 2596        // pixels
#define IMAGE_HEIGHT 1492       // pixels
#define VERTICAL_FOV 55         // degrees
// H_FOV = 2arctan(tan(V_FOV/2) * AR)
#define HORIZONTAL_FOV 84.338   // degrees

// Satellite position
#define CENTER_LAT 42.274222    // degrees
#define CENTER_LON -71.797367   // degrees
#define ALTITUDE 6096           // meters

//----------EXTERNAL VARIABLES----------
extern double grid_width;
extern double grid_height;
extern double rect_width;
extern double rect_height;
extern double meters_per_px;

//----------FUNCTION DECLARATIONS----------
/*
***************************************************************************************************
GRID_SIZING Converts satellite position and field of view to satellite bounding image in pixel coordinates

FUNCTION TAKES NO INPUTS
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void grid_sizing();

/*
***************************************************************************************************
PRINT_SCALE Prints pixel-to-meter scale on output video

INPUTS:
    img_sat     : Satellite image
    m_per_px    : Conversion factor of number of meters distance corresponding to each satellite image pixel
    length_m    : Desired scale distance (meters)
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void print_scale(cv::Mat&, double, double);

/*
***************************************************************************************************
GET_COORDINATES Calculates estimated payload GPS coordinate

INPUTS:
    p_cam   : Payload camera image
    s_cam   : Satellite image
OUTPUTS:
    closest_coord   : Closest estimated payload GPS coordinate given grid sizing
***************************************************************************************************
*/
cv::Point2d get_coordinates(cv::Mat&, cv::Mat&);

/*
***************************************************************************************************
GRID_TO_COORD Converts a GPS coordinate from grid-specific pixels to degrees latitude and longitude

INPUTS:
    grid_point      :   GPS coordinate in grid-specific pixels
OUTPUTS:
    closest_coord   :   GPS coordinate in degrees latitude and longitude
***************************************************************************************************
*/
cv::Point2d grid_to_coord(cv::Point2d);

/*
***************************************************************************************************
COORD_TO_GRID Converts a GPS coordinate from degrees latitude and longitude to grid-specific pixels

INPUTS:
    coord_point     :   GPS coordinate in degrees latitude and longitude
OUTPUTS:
    closest_grid    :   GPS coordinate in grid-specific pixels
***************************************************************************************************
*/
cv::Point2d coord_to_grid(cv::Point2d);

