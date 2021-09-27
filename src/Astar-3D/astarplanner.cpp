#include "Astar-3D/astarplanner.h"
#include "Astar-3D/astar.h"
#include "Astar-3D/gl_const.h"

AstarPlanner::AstarPlanner()
{
    search = nullptr;
}

AstarPlanner::~AstarPlanner() {
    if (search != nullptr) delete search;
}

SearchResult AstarPlanner::plan(const std::vector<std::vector<std::vector<int>>>& _grid,
                        std::array<int, 3> _start,
                        std::array<int, 3> _goal,
                        EnvironmentOptions _options) {
    updateMap(_grid, _start, _goal);
    setEnvironmentOptions(_options);
    createSearch();
    startSearch();
    return sr;
}

void AstarPlanner::setEnvironmentOptions(EnvironmentOptions _options) {
    options = _options;
}

void AstarPlanner::createSearch() {
    if(search != nullptr) {
        delete search;
    }
    search = new Astar(1.0, CN_SP_BT_GMAX, -1, map.height);
}

void AstarPlanner::startSearch() {
    sr = search->startSearch(map, options);
}

void AstarPlanner::printSearchResultsToConsole() {

    std::cout << "Path ";
    if (!sr.pathfound)
        std::cout << "NOT ";
    std::cout << "found!" << std::endl;

    std::cout << "numberofsteps=" << sr.numberofsteps << std::endl;
    std::cout << "nodescreated=" << sr.nodescreated << std::endl;

    if (sr.pathfound)
        std::cout << "pathlength=" << sr.pathlength << std::endl;
    std::cout << "time=" << sr.time << std::endl;

}

void AstarPlanner::updateMap(const std::vector<std::vector<std::vector<int>>>& _grid,
                             std::array<int, 3> _start, std::array<int, 3> _goal)
{
    map.Grid = _grid;
    map.height = _grid.size();
    map.width = _grid[0].size();
    map.altitude = _grid[0][0].size();
    map.start_i = _start[0];
    map.start_j = _start[1];
    map.start_h = _start[2];
    map.goal_i = _goal[0];
    map.goal_j = _goal[1];
    map.goal_h = _goal[2];
    map.min_altitude_limit = 0;
    map.max_altitude_limit = map.altitude - 1;
}
