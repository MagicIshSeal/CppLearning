#include "atmosphere.hpp"
#include <cmath>

// Temperature in Kelvin (linear lapse in troposphere)
double getTemperature(double h) {
    return T0 - L * h;
}

// Pressure using barometric formula
double getPressure(double h) {
    double T = getTemperature(h);
    return p0 * pow(1 - ((L * h) / T0), g / (R * L));
}

// Density using ideal gas law
double getDensity(double h) {
    double p = getPressure(h);
    double T = getTemperature(h);
    return p / (R * T);
}

// Speed of sound
double getSpeedOfSound(double h) {
    double T = getTemperature(h);
    return sqrt(gamma * R * T);
}