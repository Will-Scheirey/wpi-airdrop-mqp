#include "libraries.hpp"

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double R_EARTH = 6378137.0;

extern std::vector<cv::Point2d> all_coords;
extern std::vector<cv::Point2d> true_coords;
extern bool have_truth;
extern std::ofstream rms_out;

cv::Point2d average_coords(const std::vector<cv::Point2d>&);
void add_marker(cv::Mat&, const cv::Point2d&, const bool&, const cv::Point2d&, const cv::Point2d*);
void flight_statistics(int, cv::Mat&, cv::VideoWriter&, const bool&);
cv::Point2d degrees_to_m(cv::Point2d&);

