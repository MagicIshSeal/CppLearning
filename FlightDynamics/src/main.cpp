#include <iostream>
#include "atmosphere.hpp"

int main() {
    double altitude;

    std::cout << "Enter altitude (meters): ";
    std::cin >> altitude;

    if (altitude < 0 || altitude > 11000) {
        std::cout << "Altitude out of troposphere range (0-11000 m)\n";
        return 1;
    }

    double T = getTemperature(altitude);
    double p = getPressure(altitude);
    double rho = getDensity(altitude);
    double a = getSpeedOfSound(altitude);

    std::cout << "Temperature: " << T - 273.15 << " C\n";
    std::cout << "Pressure: " << p / 100 << " hPa\n";
    std::cout << "Density: " << rho << " kg/m^3\n";
    std::cout << "Speed of sound: " << a << " m/s\n";

    return 0;
}
