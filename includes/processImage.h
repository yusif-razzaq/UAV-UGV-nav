#pragma once

struct Path {
    Path() = default;
    std::vector<std::pair<double, double>> waypoints;
    bool valid = true;
};

struct Edge {
    uint32_t nodeIdx;  
    int cost;   
};

struct ANode {
    uint32_t ind;
    uint32_t parent;
    int cost;
    int priority;
};

cv::Mat applyMask(const cv::Mat& image);

class GridSpace {
    public:
        // GridSpace(int w, int h) : width(w), height(h) {}
        void fillGrid(const cv::Mat& image);
        void setGrid(const cv::Mat& image);
        void showPRM(const cv::Mat& image);
        bool checkLine(std::pair<int, int> start, std::pair<int, int> end);
        std::pair<int, int> getRandomPoint();
        void runPRM(const std::pair<int, int>& init, const std::pair<int, int>& goal, int n);
        std::list<uint32_t> searchGraph(uint32_t init, uint32_t goal);
        Path getWaypoints(const std::pair<double, double>& init, const std::pair<double, double>& goal);
        void connectNieghbors();
        void connectNewNode(uint32_t node);
        int manhattanDistance(const std::pair<int, int>& cell1, const std::pair<int, int>& cell2);
        std::pair<int, int> coordToPixel(const std::pair<double, double>& coordinate);
        std::pair<double, double> pixelToCoord(const std::pair<int, int>& pixel);
        uint32_t findNearest(const std::pair<int, int>& pixel);
        static bool compareCost(const ANode& a, const ANode& b);
        std::map<uint32_t, std::pair<int, int>> getPoints();
    private:
        int width, height;
        double captureHeight = 10.0;
        cv::Mat grid; 
        std::map<uint32_t, std::pair<int, int>> points;
        std::map<uint32_t, std::pair<int, int>> failedPoints;
        std::map<uint32_t, std::vector<Edge>> adjacencyList;
};