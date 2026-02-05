#include "crop.hpp"

void cropped_match(cv::Mat &img) {
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat mask;
    cv::threshold(gray, mask, 1, 255, cv::THRESH_BINARY);
    mask /= 255;

    int rows = mask.rows;
    int cols = mask.cols;

    std::vector<int> prev(cols, 0), curr(cols, 0);
    int maxSize = 0, maxRow = 0, maxCol = 0;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (mask.at<uchar>(i, j)) {
                if (i == 0 || j == 0) {
                    curr[j] = 1;
                } else {
                    curr[j] = std::min({prev[j], curr[j - 1], prev[j - 1]}) + 1;
                }
                if (curr[j] > maxSize) {
                    maxSize = curr[j];
                    maxRow = i;
                    maxCol = j;
                }
            } else {
                curr[j] = 0;
            }
        }
        std::swap(prev, curr);
        std::fill(curr.begin(), curr.end(), 0);
    }

    int x = maxCol - maxSize + 1;
    int y = maxRow - maxSize + 1;
    cv::Rect square(x, y, maxSize, maxSize);

    img = img(square);
}