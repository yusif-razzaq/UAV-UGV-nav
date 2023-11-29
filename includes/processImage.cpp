#include <opencv2/opencv.hpp>
#include <stdio.h> 
#include "processImage.h"
#include <random>
#include <chrono>

using namespace cv;
using std::pair, std::cout;

Mat applyMask(const Mat& image) {
    cv::Scalar lowerColor = cv::Scalar(154, 154, 154);  // Lower bound for (155, 155, 155)
    cv::Scalar upperColor = cv::Scalar(156, 156, 156); 
    cv::Mat mask;
    cv::inRange(image, lowerColor, upperColor, mask);
    // cv::imshow("CV Image", mask);
    // cv::waitKey(0);
    return mask;
}


void GridSpace::fillGrid(const cv::Mat& image) {
    width = image.cols;
    height = image.rows;
    grid = image.clone(); 
    cv::threshold(grid, grid, 0, 255, cv::THRESH_BINARY);
    std::vector<std::vector<int>> occupancyGrid(height, std::vector<int>(width, 0));
}

void GridSpace::setGrid(const cv::Mat& image) {
    width = image.cols;
    height = image.rows;
    grid = image.clone();
    // cv::imshow("GRID", grid);
    // cv::waitKey(0);
}

bool GridSpace::checkLine(pair<int, int> start, std::pair<int, int> end) {
    int x1 = start.second;
    int y1 = start.first;  
    int x2 = end.second;
    int y2 = end.first;
    cv::LineIterator it(grid, cv::Point(x1, y1), cv::Point(x2, y2), 8);
    for (int i = 0; i < it.count; i++, ++it) {
        int x = it.pos().x;
        int y = it.pos().y;
        if (grid.at<uchar>(y, x) != 255) return false;
    }
    return true;
}

pair<int, int> GridSpace::getRandomPoint() {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> random_row(0, width - 1);
    std::uniform_int_distribution<int> random_col(0, height - 1);
    int i = random_row(gen);
    int j = random_col(gen);
    return {i, j};
}

void GridSpace::runPRM(const std::pair<int, int>& init, const std::pair<int, int>& goal, int n) {
    int ind = 0;
    std::pair<int, int> qRand;
    if (points.size() == 0) points[0] = {0, 0};
    while (ind < n && points.size() < 750) {
        qRand = getRandomPoint();
        if (grid.at<uchar>(qRand.first, qRand.second) != 0) {
            points[points.size()] = qRand;
            connectNewNode(points.size() - 1);
        } else {
            failedPoints[failedPoints.size()] = qRand;
        }
        ind++;
    }
    std::cout << "NODES IN PRM:" << points.size() << "\n";
}

void GridSpace::connectNieghbors() { 
    int norm;
    int n = points.size();
    int r = int(width/8);
    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = i + 1; j < n; j++) { 
            norm = manhattanDistance(points[i], points[j]);
            if (norm < r) {
                if (checkLine(points[i], points[j])) {
                    // connectedNodes.push_back({points[i], points[j]});
                    adjacencyList[i].push_back({j, norm});
                    adjacencyList[j].push_back({i, norm});
                }
            }
        }
    }
}

void GridSpace::connectNewNode(uint32_t node) { 
    int norm;
    int n = points.size();
    int r = int(width/12);
    uint32_t j = node;
    for (uint32_t i = 0; i < n; i++) {
        if (i != j) {
            // std::cout << "Checking: " << i << " and "<< j << "\n";
            norm = manhattanDistance(points[i], points[j]);
            if (norm < r) {
                if (checkLine(points[i], points[j])) {
                    adjacencyList[i].push_back({j, norm});
                    adjacencyList[j].push_back({i, norm});
                }
            }
        }
    }
}

int GridSpace::manhattanDistance(const std::pair<int, int>& cell1, const std::pair<int, int>& cell2) {
    return abs(cell1.first - cell2.first) + abs(cell1.second - cell2.second);
}

void GridSpace::showPRM(const cv::Mat& image) {
    for (const auto& pair : adjacencyList) {
        uint32_t ind = pair.first;
        const std::vector<Edge>& edges = pair.second;

        // Get the coordinates of the node
        int x = points[ind].first;
        int y = points[ind].second;

        // Draw a circle representing the node on the image

        for (const Edge& edge : edges) {
            uint32_t destNodeIdx = edge.nodeIdx;

            // Get coordinates of the destination node
            int destX = points[destNodeIdx].first;
            int destY = points[destNodeIdx].second;

            // Draw a line representing the edge on the image
            cv::line(image, cv::Point(y, x), cv::Point(destY, destX), cv::Scalar(10, 135, 215), 1);  // Red line
        }
        cv::circle(image, cv::Point(y, x), 5, cv::Scalar(243, 150, 33), -1);  // Blue circle
    }

    cv::imshow("Nodes and Edges", image);
    cv::waitKey(0);  // Wait for a key press
}

std::list<uint32_t> GridSpace::searchGraph(uint32_t init, uint32_t goal) {
    ANode currNode, node;
    currNode.ind = init;
    std::list<ANode> queue = {currNode};
    std::map<uint32_t, ANode> openList;
    std::map<uint32_t, ANode> processed;
    openList[init] = currNode;
    uint32_t index;
    int cost;
    int success = -1;
    while(success < 0) {
            queue.sort(compareCost);
            currNode = queue.back();
            queue.pop_back();
            openList.erase(currNode.ind);
            processed[currNode.ind] = currNode;

            for (const Edge& edge : adjacencyList[currNode.ind]) {
                index = edge.nodeIdx;
                cost = edge.cost;
                // cout << "Checking Child at node: " << index << "\n";
                if (index == goal) cout << "BING\n";
                if (processed.count(index) == 0) {
                    node = {index, currNode.ind, currNode.cost + cost, currNode.cost + cost};
                    if (openList.count(index) == 0) {
                        queue.push_back(node);
                        openList[index] = node;
                        // cout << "Added node " << index << " with cost " << currNode.cost + edges[i] << "\n";
                    } else {
                        // cout << "Node already in list " << index << " with cost " << openList[index].cost << "\nComparing to new path cost: " << currNode.cost + edges[i] << "\n";
                        if (openList[index].cost > currNode.cost + cost) {
                            queue.remove_if([index](const ANode& aNode) {
                                return aNode.ind == index;
                            });
                            // openList.erase(index);
                            queue.push_back(node);
                            openList[index] = node;
                        }
                    }
                }
            }
            if (currNode.ind == goal) success = 0;
            else if (queue.empty()) success = 1;
    }
    std::list<uint32_t> path;
    if (success == 0) {
        cout << "SUCCESS\n";
        index = currNode.ind;
        while (index != init) {
            path.push_front(index);
            index = processed[index].parent;
        }
    }

    return path;

}

Path GridSpace::getWaypoints(const std::pair<double, double>& init, const std::pair<double, double>& goal) {
    Path path;
    uint32_t initNode = findNearest(coordToPixel(init));
    uint32_t goalNode = findNearest(coordToPixel(goal));
    cout << "Init nodes " << points[initNode].first << ", " << points[initNode].second << "\n";
    cout << "Goal nodes " << points[goalNode].first << ", " << points[goalNode].second << "\n";
    std::list<uint32_t> nodePath = searchGraph(initNode, goalNode);
    for (uint32_t node : nodePath) {
        path.waypoints.push_back(pixelToCoord(points[node]));
    }
    if (nodePath.size() == 0) path.valid = false;
    return path;
}

bool GridSpace::compareCost(const ANode& a, const ANode& b) {
    return a.priority > b.priority;
}

uint32_t GridSpace::findNearest(const std::pair<int, int>& pixel) { 
    std::pair<int, int> nearest = points[0];
    int n = points.size();
    uint32_t ind = 0;
    for (uint32_t i = 1; i < points.size(); i++) {
        if (manhattanDistance(pixel, points[i]) < manhattanDistance(pixel, nearest)) {
            nearest = points[i];
            ind = i;
        }
    }
    cout << ind << "\n";
    return ind;
}

std::pair<int, int> GridSpace::coordToPixel(const std::pair<double, double>& coordinate) {
    // Given parameters
    float fov_horizontal_rad = 1.25f;
    float fov_vertical_rad = 1.25f;
    int image_width = 1000;  // Image width in pixels
    int image_height = 1000;  // Image height in pixels
    float x = -coordinate.first;
    float y = -coordinate.second;

    // Calculate focal length in horizontal and vertical directions
    float focal_length_horizontal = image_width / (2 * tan(fov_horizontal_rad / 2));
    float focal_length_vertical = image_height / (2 * tan(fov_vertical_rad / 2));

    int pixel_x = static_cast<int>((focal_length_horizontal * x) / captureHeight + (image_width / 2));
    int pixel_y = static_cast<int>((focal_length_vertical * y) / captureHeight + (image_height / 2));

    std::pair<int, int> pixel_offset = {pixel_x, pixel_y};
    std::cout << "Pixel offset: (" << pixel_offset.first << ", " << pixel_offset.second << ")" << std::endl;
    return pixel_offset;
}


std::pair<double, double> GridSpace::pixelToCoord(const std::pair<int, int>& pixel) {
    // Given parameters
    float fov_horizontal_rad = 1.25f;
    float fov_vertical_rad = 1.25f;
    int image_width = 1000;  // Image width in pixels
    int image_height = 1000;  // Image height in pixels

    // Calculate focal length in horizontal and vertical directions
    float focal_length_horizontal = image_width / (2 * tan(fov_horizontal_rad / 2));
    float focal_length_vertical = image_height / (2 * tan(fov_vertical_rad / 2));

    // Calculate x and y based on pixel offsets
    float x = ((pixel.first - (image_width / 2)) * captureHeight) / focal_length_horizontal;
    float y = ((pixel.second - (image_height / 2)) * captureHeight) / focal_length_vertical;

    std::pair<double, double> coordinate = {-x, -y};
    std::cout << "Coordinate: (" << coordinate.first << ", " << coordinate.second << ")" << std::endl;
    return coordinate;
}


std::map<uint32_t, std::pair<int, int>> GridSpace::getPoints() {
    return points;
}
