#include "Astar-3D/environmentoptions.h"

EnvironmentOptions::EnvironmentOptions(int MT, bool AS, double LC, double DC, int AD, bool AC)
{
    metrictype = MT;
    allowsqueeze = AS;
    linecost = LC;
    diagonalcost = DC;
    allowdiagonal = AD;
    allowcutcorners = AC;
}

EnvironmentOptions::EnvironmentOptions()
{
    metrictype = CN_SP_MT_EUCL;
    allowsqueeze = CN_SP_AS_FALSE;
    linecost = CN_MC_LINE;
    diagonalcost = CN_MC_DIAG;
    allowdiagonal = CN_SP_AD_FALSE; //CN_SP_AD_TRUE
    allowcutcorners = CN_SP_AC_FALSE;
}

