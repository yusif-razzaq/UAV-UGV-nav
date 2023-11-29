#include <opencv2/opencv.hpp>
#include <stdio.h> 
#include "processImage.h"
#include <random>

using namespace cv;
using std::pair, std::cout;

Mat applyMask() {
    Mat image = imread("roomSquare.png");
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat thresholded;
    cv::threshold(gray_image, thresholded, 200, 255, cv::THRESH_BINARY);
    imshow("Thresholded Image", thresholded);
    waitKey(0);

    return thresholded;
}


void GridSpace::fillGrid(cv::Mat image) {
    width = image.cols;
    height = image.rows;
    grid = image.clone(); 
    cv::threshold(grid, grid, 0, 255, cv::THRESH_BINARY);
    std::vector<std::vector<int>> occupancyGrid(height, std::vector<int>(width, 0));
}

void GridSpace::setGrid(cv::Mat image) {
    grid = image;
}

bool GridSpace::checkLine(pair<int, int> start, pair<int, int> end) {
    int x1 = start.first;
    int y1 = start.second;  
    int x2 = end.first;
    int y2 = end.second;
    cv::LineIterator it(grid, cv::Point(x1, y1), cv::Point(x2, y2), 8);
    for (int i = 0; i < it.count; i++, ++it) {
        int x = it.pos().x;
        int y = it.pos().y;
        if (grid.at<uchar>(y, x) != 255) return false;
    }
    return true;
}

pair<int, int> GridSpace::getRandomPoint() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> random_row(0, width - 1);
    std::uniform_int_distribution<int> random_col(0, height - 1);
    int i = random_row(gen);
    int j = random_col(gen);
    return {i, j};
}

void GridSpace::runPRM(pair<int, int> init, pair<int, int> goal, int n) {
    points[0] = init;
    points[1] = goal;
    int ind = 2;
    pair<int, int> qRand;
    while (points.size() < n) {
        qRand = getRandomPoint();
        if (grid.at<uchar>(qRand.first, qRand.second) == 255) {
            points[ind] = qRand;
            std::cout << "Adding Point\n";
            ind++;
        }
    }
    connectNieghbors();
}

void GridSpace::connectNieghbors() { 
    int norm;
    int n = points.size();
    int r = int(width/10);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) { 
            norm = manhattanDistance(points[i], points[j]);
            if (norm < r) {
                if (checkLine(points[i], points[j])) {
                    connectedNodes.push_back({points[i], points[j]});
                }
            }
        }
    }
}

int GridSpace::manhattanDistance(pair<int, int> cell1, pair<int, int> cell2) {
    return abs(cell1.first - cell2.first) + abs(cell1.second - cell2.second);
}


Path GridSpace::searchGraph(std::pair<int, int> init, std::pair<int, int> goal) {
    Path path();
    
}
