#include "libraries.hpp"

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double R_EARTH = 6378137.0;

extern std::vector<cv::Point2d> all_coords;
extern cv::Point2d centroid;
extern float rms_lat;
extern float rms_lon;
extern int num_points;

cv::Point2d average_coords(const std::vector<cv::Point2d>&);
void add_marker(cv::Mat&, const cv::Point2d&, const cv::Point2d&);
void flight_statistics(int, cv::Mat&, cv::VideoWriter&);
cv::Point2d degrees_to_m(cv::Point2d&);

