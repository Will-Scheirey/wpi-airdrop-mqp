#include "libraries.hpp"
#include "crop.cpp"
#include "grid.cpp"
#include "frame.cpp"

#define N_FEATURES 100

static int window_index = 0;
const int window = 5;

static std::vector<cv::Point2f> sat_points, payload_points;
std::vector<cv::Point2d> all_coords;

static bool have_last = false;
static bool have_prev = false;
static cv::Point2d last_valid(0.0, 0.0), prev_valid(0.0, 0.0);

cv::Point2d centroid(0.0,0.0);
float rms_lat = 0.0;
float rms_lon = 0.0;
int num_points = 0;

void push_coord(std::vector<cv::Point2d>&, const cv::Point2d&);
void push_frame(std::vector<cv::Mat>&, const cv::Mat&);

int main (void) {
    std::cout << std::fixed << std::setprecision(5);

    cv::Mat img_sat = cv::imread("/Users/paigerust/Desktop/fg_cv_test_1/sat_view.png", -1);
    cv::cvtColor(img_sat, img_sat, cv::COLOR_BGRA2BGR);
    // cv::Mat img_fg = cv::imread ("/Users/paigerust/Desktop/fg_cv_test_1/payload_view_5.png", -1);

    cv::VideoCapture vid_fg("/Users/paigerust/Desktop/fg_cv_test_1/fg_cv_vid_1.mov");

    int frame_count = static_cast<int>(vid_fg.get(cv::CAP_PROP_FRAME_COUNT));
    double fps = vid_fg.get(cv::CAP_PROP_FPS);
    int frame_width = static_cast<int>(vid_fg.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(vid_fg.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "Frame count: " << frame_count << "\n";
    std::cout << "FPS: " << fps << "\n";

    cv::VideoWriter vid_out("/Users/paigerust/Desktop/fg_cv_test_1/fg_cv_out_1.mov", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps, cv::Size(frame_width, frame_height));
    cv::Mat frame_fg;

    if (!vid_fg.isOpened()) {
        std::cerr << "Error: Could not open video file.\n";
        return -1;
    }

    cv::Ptr<cv::ORB> ORB = cv::ORB::create(N_FEATURES);

    std::vector<cv::KeyPoint> sat_kp, payload_kp;
    cv::Mat sat_desc, payload_desc;
    ORB->detectAndCompute(img_sat, cv::noArray(), sat_kp, sat_desc);

    std::vector<cv::Mat> window_frames;
    std::vector<cv::Point2d> window_coords;

    double video_timestamp = 0.0;

    for (int i = 0; i < frame_count; i++) {
        bool compute = false;

        vid_fg >> frame_fg;
        if (frame_fg.empty()) {
            std::cerr << "No frame loaded.\n";
            break;
        }

        video_timestamp += 1.0 / fps;

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knnMatches;
        cv::Mat inlierMask, H;
        ORB->detectAndCompute(frame_fg, cv::noArray(), payload_kp, payload_desc);

        if (payload_desc.empty() || sat_desc.empty()) {
            compute = false;
        } else {
            matcher.knnMatch(sat_desc, payload_desc, knnMatches, 2);

            const float threshold = 0.75;
            std::vector<cv::DMatch> goodMatches;
            for (int j = 0; j < knnMatches.size(); j++) {
                if (knnMatches[j].size() == 2 && knnMatches[j][0].distance < threshold * knnMatches[j][1].distance) {
                    goodMatches.push_back(knnMatches[j][0]);
                }
            }
            if (goodMatches.size() >= 4) {
                sat_points.clear();
                payload_points.clear();
                sat_points.reserve(goodMatches.size());
                payload_points.reserve(goodMatches.size());

                for (const auto &m : goodMatches) {
                    sat_points.push_back(sat_kp[m.queryIdx].pt);
                    payload_points.push_back(payload_kp[m.trainIdx].pt);
                }
                H = cv::findHomography(payload_points, sat_points, cv::RANSAC, 5.0, inlierMask);

                compute = !H.empty();
            } else {
                std::cerr << "Not enough matches (" << goodMatches.size() << ") to compute homography.\n";
            }
        }

        if (compute) {
            cv::Mat warped;
            cv::warpPerspective(frame_fg, warped, H, img_sat.size());

            cv::Mat cropped = warped.clone();
            cropped_match(cropped);

            cv::Point2d coord = get_coordinates(cropped, img_sat);

            push_frame(window_frames, cropped);
            push_coord(window_coords, coord);

            prev_valid = last_valid;
            have_prev = have_last;
            last_valid = coord;
            have_last = true;

            window_index = (window_index + 1) % window;
        } else {
            if (have_prev && have_last) {
                cv::Point2d interp = last_valid + (last_valid - prev_valid);

                if (window_coords.size() < window) window_coords.push_back(interp);
                else window_coords[window_index] = interp;

                prev_valid = last_valid;
                last_valid = interp;
                have_prev = true;
                have_last = true;

                window_index = (window_index + 1) % window;
            } else if (have_last) {
                cv::Point2d interp = last_valid;

                if (window_coords.size() < window) window_coords.push_back(interp);
                else window_coords[window_index] = interp;

                window_index = (window_index + 1) % window;
            }
        }

        cv::Point2d averaged_coordinate = average_coords(window_coords);
        all_coords.push_back(averaged_coordinate);
        std::cout << "=========================================================\n";
        std::cout << "Video Timestamp:\t" << video_timestamp << "\n";
        std::cout << "Estimated Latitude:\t" << averaged_coordinate.x << "\n";
        std::cout << "Estimated Longitude:\t" << averaged_coordinate.y << "\n";

        cv::Mat out = img_sat.clone();
        add_marker(out, averaged_coordinate);
        vid_out.write(out); 
    }

    flight_statistics();
    cv::Point2d rms_m = degrees_to_m();
    std::cout << "=========================================================\n";
    std::cout << "Centroid:\t" << centroid << "\n";
    std::cout << "Latitudinal RMS (deg):\t" << rms_lat << "\t| Latitudinal RMS (m)\t" << rms_m.x << "\n";
    std::cout << "Longitudinal RMS (deg):\t" << rms_lon << "\t| Longitudinal RMS (m)\t" << rms_m.y << "\n";

    vid_fg.release();
    vid_out.release();
    cv::destroyAllWindows();

    return 0;
}

void push_coord(std::vector<cv::Point2d>& window_coords, const cv::Point2d& coord) {
    if (window_coords.size() < window) window_coords.push_back(coord);
    else window_coords[window_index] = coord;
}

void push_frame(std::vector<cv::Mat>& window_frames, const cv::Mat& frame){
    if (window_frames.size() < window) window_frames.push_back(frame);
    else window_frames[window_index] = frame;
}



