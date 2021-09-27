#ifndef SEARCHRESULT_H
#define SEARCHRESULT_H

#include "list.h"
#include <ctime>
#include <unordered_map>

struct SearchResult
{
        bool pathfound;
        float pathlength;
        const NodeList* lppath;
        const NodeList* hppath;
        unsigned int nodescreated;
        unsigned int numberofsteps;
        double time;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
            lppath = NULL;
            hppath = NULL;
            nodescreated = 0;
            numberofsteps = 0;
            time = 0;
        }

};

#endif
