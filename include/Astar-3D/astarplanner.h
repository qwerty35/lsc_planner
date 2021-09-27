#ifndef MISSION_H
#define	MISSION_H

#include "map.h"
#include "isearch.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "gl_const.h"
#include <iostream>


class AstarPlanner
{
    public:
        AstarPlanner();
        ~AstarPlanner();

        SearchResult plan(const std::vector<std::vector<std::vector<int>>>& _map,
                          std::array<int, 3> _start,
                          std::array<int, 3> _goal,
                          EnvironmentOptions _options);

        void printSearchResultsToConsole();

    private:
        Map                     map;
        EnvironmentOptions      options;
        ISearch*                search;
        SearchResult            sr;

        void updateMap(const std::vector<std::vector<std::vector<int>>>& _grid,
                       std::array<int, 3> _start,
                       std::array<int, 3> _goal);
        void createSearch();
        void setEnvironmentOptions(EnvironmentOptions _options);
        void startSearch();
};

#endif

