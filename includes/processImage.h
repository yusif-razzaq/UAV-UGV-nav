#pragma once

struct Path {
    Path() = default;
    std::vector<std::pair<int, int>> waypoints;
    bool valid;
};

cv::Mat applyMask();

class GridSpace {
    public:
        // GridSpace(int w, int h) : width(w), height(h) {}
        void fillGrid(cv::Mat image);
        void setGrid(cv::Mat image);
        bool checkLine(std::pair<int, int> start, std::pair<int, int> end);
        std::pair<int, int> getRandomPoint();
        void runPRM(std::pair<int, int> init, std::pair<int, int> goal, int n);
        Path searchGraph(std::pair<int, int> init, std::pair<int, int> goal);
        void connectNieghbors();
        int manhattanDistance(std::pair<int, int> cell1, std::pair<int, int> cell2);
        std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> connectedNodes;
    private:
        int width, height;
        cv::Mat grid; 
        std::map<int, std::pair<int, int>> points;
};