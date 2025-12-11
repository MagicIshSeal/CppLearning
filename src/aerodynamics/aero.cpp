#include "aero.hpp"
#include <cmath>

// Linear lift coefficient (legacy)
double calcCL(double alpha, double CL_alpha)
{
    return CL_alpha * alpha; // alpha in radians
}

// Drag coefficient (parabolic drag polar - legacy)
double calcCD(double CL, double CD0, double k)
{
    return CD0 + k * CL * CL;
}

// Lift coefficient from table data
double calcCL(double alpha, const AeroDataTable *table)
{
    if (table && !table->isEmpty())
    {
        return table->getCL(alpha);
    }
    return 0.0;
}

// Drag coefficient from table data
double calcCD(double alpha, const AeroDataTable *table)
{
    if (table && !table->isEmpty())
    {
        return table->getCD(alpha);
    }
    return 0.0;
}

// Lift force [N]
double calcLift(double rho, double V, double S, double CL)
{
    return 0.5 * rho * V * V * S * CL;
}

// Drag force [N]
double calcDrag(double rho, double V, double S, double CD)
{
    return 0.5 * rho * V * V * S * CD;
}

// Weight [N]
double calcWeight(double mass, double g)
{
    return mass * g;
}

// Thrust [N] (simplified linear with throttle)
double calcThrust(double throttle, double maxThrust)
{
    return throttle * maxThrust;
}
