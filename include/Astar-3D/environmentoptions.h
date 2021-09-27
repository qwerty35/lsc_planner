#ifndef ENVIRONMENTOPTIONS_H
#define ENVIRONMENTOPTIONS_H

#include "gl_const.h"

class EnvironmentOptions
{
public:
    EnvironmentOptions(int MT, bool AS, double LC, double DC, int AD, bool AC);
    EnvironmentOptions();

    int    metrictype;
    bool   allowsqueeze;
    bool   allowcutcorners;
    double  linecost;
    double  diagonalcost;
    int    allowdiagonal;
    bool   useresetparent;
};

#endif // ENVIRONMENTOPTIONS_H
