#include "libraries.hpp"

//----------GLOBAL VARIABLES----------
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double R_EARTH = 6378137.0;

//----------EXTERNAL VARIABLES----------
extern std::vector<cv::Point2d> all_coords;
extern std::vector<cv::Point2d> true_coords;
extern bool have_truth;
extern std::ofstream rmse_out, error_out;

//----------FUNCTION DECLARATIONS----------
/*
***************************************************************************************************
AVERAGE_COORDS Computes the average/centroid coordinate of a vector of GPS coordinates

INPUTS:
    coords :   Vector of GPS coordinates (latitude, longitude)
OUTPUTS:
    average_coords  :   Average/centroid GPS coordinate (latitude, longitude) of coords vector input
***************************************************************************************************
*/
cv::Point2d average_coords(const std::vector<cv::Point2d>&);

/*
***************************************************************************************************
ADD_MARKER Prints a visual marker on output video of payload true/estimated GPS location

INPUTS:
    img_sat     :   Satellite image
    est_coord   :   Payload GPS coordinate estimate
    have_truth  :   Indicates whether true GPS coordinates were provided
    rmse        :   Root mean square error associated with est_coord (latitudinal, longitudinal)
    true_coord  :   Ground truth payload GPS coordinate
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void add_marker(cv::Mat&, const cv::Point2d&, const bool&, const cv::Point2d&, const cv::Point2d*);

/*
***************************************************************************************************
FLIGHT_STATISTICS Calculates and plots estimated and ground truth payload GPS locations

INPUTS:
    img_sat     :   Satellite image
    vid_out     :   Output video
    have_truth  :   Indicates whether true GPS coordinates were provided
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void flight_statistics(const cv::Mat&, cv::VideoWriter&, const bool&);

/*
***************************************************************************************************
INTERPOLATE Interpolates GPS coordinates for gaps in payload GPS coordinate estimates

INPUTS:
    all_coords  :   Vector of estimated payload GPS coordinates
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void interpolate(std::vector<cv::Point2d>& all_coords);

/*
***************************************************************************************************
DEGREES_TO_M Converts a GPS coordinate distance from degrees to meters

INPUTS:
    rmse    :   Root mean square error (degrees) associated with estimated payload GPS coordinate (latitudinal, longitudinal)
OUTPUTS:
    rmse_m  :   Root mean square error (m) (latitudinal, longitudinal)
***************************************************************************************************
*/
cv::Point2d degrees_to_m(cv::Point2d&);

/*
***************************************************************************************************
CROPPED_MATCH Crops the incoming payload camera image to the largest interior bounding square

INPUTS:
    img :   Payload camera image frame
OUTPUTS:
    bool    :   Indicates successful/unsuccessful crop
***************************************************************************************************
*/
bool cropped_match(cv::Mat&);

