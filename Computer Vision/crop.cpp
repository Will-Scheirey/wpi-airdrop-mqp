#include "crop.hpp"

bool cropped_match(cv::Mat &img) {
    if (img.empty()) return false;

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat mask;
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);

    const int rows = mask.rows, cols = mask.cols;
    std::vector<int> prev(cols, 0), curr(cols, 0);
    int maxSize = 0, maxRow = 0, maxCol = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (mask.at<uchar>(i, j) != 0) {
                curr[j] = (i == 0 || j == 0) ? 1
                          : std::min({prev[j], curr[j-1], prev[j-1]}) + 1;
                if (curr[j] > maxSize) { maxSize = curr[j]; maxRow = i; maxCol = j; }
            } else {
                curr[j] = 0;
            }
        }
        std::swap(prev, curr);
        std::fill(curr.begin(), curr.end(), 0);
    }

    // Guard: no valid square found
    const int minSize = 20; // tune; prevents tiny garbage crops
    if (maxSize < minSize) return false;

    int x = maxCol - maxSize + 1;
    int y = maxRow - maxSize + 1;

    // Clamp ROI to image bounds (extra safety)
    x = std::max(0, std::min(x, cols - maxSize));
    y = std::max(0, std::min(y, rows - maxSize));

    cv::Rect square(x, y, maxSize, maxSize);
    img = img(square).clone(); // clone to avoid weird ROI lifetime issues
    return true;
}