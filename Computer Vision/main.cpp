#include "libraries.hpp"
#include "grid.cpp"
#include "frame.cpp"
#include "csv.cpp"

/*
------------SAMPLE INPUT------------
./main \
/path/to/satellite_image.png \
/path/to/output_video.mov \
/path/to/output_coordinate_log.csv \
--load_video /path/to/input_video.mov \ OR --load_csv /path/to/input_gps.csv \
--load_truth /path/to/ground_truth_gps.csv
*/

//----------GLOBAL VARIABLES----------
static int window_index = 0;

static std::vector<cv::Point2f> sat_points, payload_points;
std::vector<cv::Point2d> all_coords, true_coords;
std::vector<double> truth_timestamps, coord_timestamps;

static bool have_last = false;
static bool have_prev = false;
bool have_truth = false;
static bool have_csv = false;
static cv::Point2d last_valid(0.0, 0.0), prev_valid(0.0, 0.0);

std::ofstream rmse_out("rmse_out.csv");
std::ofstream error_out("error_out.csv");

//----------FUNCTION DECLARATIONS----------
void push_coord(std::vector<cv::Point2d>&, const cv::Point2d&);
void push_frame(std::vector<cv::Mat>&, const cv::Mat&);

int main (int argc, char** argv) {
    // Initialize grid given satellite image characteristics
    grid_sizing();
    std::cout << std::fixed << std::setprecision(5);
    cv::Mat img_sat;
    cv::VideoCapture video_in; cv::VideoWriter video_out;
    std::ofstream coords_out;

    // Read command-line inputs
    if (argc > 1) {
        img_sat = cv::imread(argv[1]);
        cv::cvtColor(img_sat, img_sat, cv::COLOR_BGRA2BGR);
        char *compare;

        // Open output video from path
        video_out.open(argv[2], cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60.0, img_sat.size());

        // Open coordinate output CSV from path
        coords_out.open(argv[3]);
        coords_out << std::fixed << std::setprecision(5);
        coords_out << "timestamp" << "," << "latitude" << "," << "longitude" << std::endl;

        compare = argv[4];
        bool csv = strcmp(compare, "--load_csv");
        bool video = strcmp(compare, "--load_video");
        // Check if GPS coordinates are being loaded from a CSV or if a video is being passed in for GPS coordinate generation
        if (!csv) { // Load from CSV
            have_csv = true;
            std::string csv_path = argv[5];

            // Load CSV contents into vectors
            if (!load_coords_csv(csv_path, all_coords, &coord_timestamps)) {
                return -1;
            }
        } else if (!video) { // Load from video
            video_in.open(argv[5]);
        }

        // Check if ground truth payload GPS coordinates are provided
        bool truth;
        compare = argv[6];
        if (compare) truth = strcmp(compare, "--load_truth");
        else truth = false;
        if (!truth) {
            have_truth = true;
            std::string truth_path = argv[7];

            // Load CSV contents into vectors
            if (!load_coords_csv(truth_path, true_coords, &truth_timestamps)) {
                return -1;
            }
        }
    }

    int frame_count; double fps;
    if (!video_out.isOpened()) {
        std::cerr << "Error: Could not open video_out file.\n";
        return -1;
    }

    // Calculate estimated payload GPS for each input video frame
    if (video_in.isOpened()) {
        frame_count = static_cast<int>(video_in.get(cv::CAP_PROP_FRAME_COUNT));
        fps = video_in.get(cv::CAP_PROP_FPS);

        std::cout << "Frame count: " << frame_count << "\n";
        std::cout << "FPS: " << fps << "\n";
    
        cv::Ptr<cv::ORB> ORB = cv::ORB::create(N_FEATURES);

        std::vector<cv::KeyPoint> sat_kp, payload_kp;
        cv::Mat sat_desc, payload_desc;
        ORB->detectAndCompute(img_sat, cv::noArray(), sat_kp, sat_desc);

        std::vector<cv::Mat> window_frames;
        std::vector<cv::Point2d> window_coords;

        // If input video is not at the same FPS as data was streamed, down-sample input video to correct framerate
        double video_timestamp = 0.0;
        const double true_fps = 5.0;
        const int stride = std::max(1, (int)std::lround(fps / true_fps));

        cv::Mat frame;
        if (!have_csv) { // Only compute coordinates if GPS coordinate CSV was not provided
            cv::Mat frame;
            // Iterate through each frame of input video
            for (int i = 0; i < frame_count; ++i) {
                video_in >> frame;
                if (frame.empty()) break;

                // Skip duplicated frames
                if (i % stride != 0) continue;
                std::cout << i << std::endl;
                video_timestamp += 1.0 / true_fps;

                // Compare and match features between satellite and payload camera images
                cv::BFMatcher matcher(cv::NORM_HAMMING);
                std::vector<std::vector<cv::DMatch>> knnMatches;
                cv::Mat inlierMask, H;
                ORB->detectAndCompute(frame, cv::noArray(), payload_kp, payload_desc);

                // Only compute homography matrix if matches between images were identified
                bool compute = false;
                if (payload_desc.empty() || sat_desc.empty()) compute = false;
                else {
                    matcher.knnMatch(sat_desc, payload_desc, knnMatches, 2);

                    const float threshold = 0.75;
                    std::vector<cv::DMatch> goodMatches;
                    for (int j = 0; j < knnMatches.size(); j++) {
                        if (knnMatches[j].size() == 2 && knnMatches[j][0].distance < threshold * knnMatches[j][1].distance) {
                            goodMatches.push_back(knnMatches[j][0]);
                        }
                    }
                    // Only compute GPS coordinate if more than 4 matches between images were found
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
                    } else std::cerr << "Not enough matches (" << goodMatches.size() << ") to compute homography.\n";
                }

                // Warp payload camera image into satellite image frame
                cv::Point2d sample(0.0, 0.0);
                if (compute) {
                    cv::Mat warped;
                    cv::warpPerspective(frame, warped, H, img_sat.size());

                    // Crop payload camera image into largest bounding square
                    cv::Mat cropped = warped.clone();
                    // Exit matching if image could not be cropped
                    if (!cropped_match(cropped)) {
                        compute = false;
                    } 
                    // Exit matching if cropped image does not fit into satellite image (erroneous crop)
                    else if (cropped.rows > img_sat.rows || cropped.cols > img_sat.cols) {
                        compute = false;
                    } else {
                        // Add GPS coordinates to estimated payload GPS coordinate vector
                        sample = get_coordinates(cropped, img_sat);

                        push_frame(window_frames, cropped);
                        push_coord(window_coords, sample);

                        prev_valid = last_valid;
                        have_prev = have_last;
                        last_valid = sample;
                        have_last = true;
                    }
                } else {
                    // If no new GPS coordinates were calculated, duplicate and store previous GPS coordinate
                    if (have_last) sample = last_valid;
                    // If previous GPS coordinate is not available, store zeros
                    else sample = {0.0, 0.0};
                }

                // Compute estimated payload GPS coordinate average if a window's worth of estimates is stored
                if (window_coords.size() < WINDOW) window_coords.push_back(sample);
                else window_coords[window_index] = sample;
                coords_out << video_timestamp << "," << sample.x << "," << sample.y << std::endl;
                window_index = (window_index + 1) % WINDOW;

                cv::Point2d averaged_coordinate = average_coords(window_coords);
                all_coords.push_back(averaged_coordinate);
            }
        }
    }

    // Fill in gaps in payload GPS coordinate estimation and print flight statistics to output video
    interpolate(all_coords);
    flight_statistics(img_sat, video_out, have_truth);

    // Close videos and images
    if (video_in.isOpened()) video_in.release();
    video_out.release();
    cv::destroyAllWindows();

    return 0;
}

/*
***************************************************************************************************
PUSH_COORD Adds new estimated payload GPS coordinate to window of GPS coordinates

INPUTS:
    window_coords   : Vector of estimated payload GPS coordinates
    coord           : Estimated payload GPS coordinate to be added at the end of the window
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void push_coord(std::vector<cv::Point2d>& window_coords, const cv::Point2d& coord) {
    if (window_coords.size() < WINDOW) window_coords.push_back(coord);
    else window_coords[window_index] = coord;
}

/*
***************************************************************************************************
PUSH_FRAME Adds new payload camera frame to window of frames

INPUTS:
    window_frames   : Vector of payload camera frames
    frame           : Payload camera frame to be added at the end of the window
FUNCTION RETURNS NO OUTPUTS
***************************************************************************************************
*/
void push_frame(std::vector<cv::Mat>& window_frames, const cv::Mat& frame){
    if (window_frames.size() < WINDOW) window_frames.push_back(frame);
    else window_frames[window_index] = frame;
}
