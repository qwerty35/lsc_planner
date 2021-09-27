#include "Astar-3D/map.h"

#include <climits>
#include <vector>
#include <stdexcept>


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

Map::Map() {
    height = -1;
    width = -1;
    altitude = 0;
    start_i = -1;
    start_j = -1;
    start_h = -1;
    goal_i = -1;
    goal_j = -1;
    goal_h = -1;
    min_altitude_limit = 0;
    max_altitude_limit = INT_MAX;
}

Map::~Map() {
    Grid.clear();
}

bool Map::CellIsTraversable(int i, int j) const {
    return Grid[i][j][0] == CN_GC_NOOBS;
}

bool Map::CellIsTraversable(int i, int j, int h) const {
    return !CellIsObstacle(i, j, h);
}

bool Map::CellIsObstacle(int i, int j) const {
    return (Grid[i][j][0] != CN_GC_NOOBS);
}

bool Map::CellIsObstacle(int i, int j, int h) const {
    return Grid[i][j][h] != CN_GC_NOOBS;
}

bool Map::CellOnGrid(int i, int j) const {
    return (i < height && i >= 0 && j < width && j >= 0);
}

bool Map::CellOnGrid(int i, int j, int h) const {
    return h >= min_altitude_limit && h <= max_altitude_limit && CellOnGrid(i, j);
}

int Map::getValue(int i, int j, int h) const {
    if (i < 0 || i >= height)
        throw std::out_of_range("i coordinate is out of grid");

    if (j < 0 || j >= width)
        throw std::out_of_range("j coordinate is out of grid");

    if (h < 0 || h >= altitude)
        throw std::out_of_range("j coordinate is out of grid");

    return Grid[i][j][h];
}
