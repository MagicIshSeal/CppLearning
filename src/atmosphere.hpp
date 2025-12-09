#ifndef ATMOSPHERE_HPP
#define ATMOSPHERE_HPP

// Constants for ISA
const double T0 = 288.15;      // Sea level temperature [K]
const double p0 = 101325.0;    // Sea level pressure [Pa]
const double L  = 0.0065;      // Temperature lapse rate [K/m]
const double R  = 287.0;       // Gas constant [J/kgK]
const double g  = 9.80665;     // Gravity [m/s^2]
const double gamma = 1.4;      // Heat capacity ratio

// Functions to calculate atmospheric properties
double getTemperature(double altitude); // K
double getPressure(double altitude);    // Pa
double getDensity(double altitude);     // kg/m^3
double getSpeedOfSound(double altitude);// m/s

#endif