#include <iostream>
#include <cmath>

int main(){
    double h;
    const double T0 = 288.15;
    const double p0 = 101325.0;
    const double L  = 0.0065;
    const double R  = 287.0;
    const double g  = 9.80665;

    std::cout << "Enter altitude in meters: ";
    std::cin >> h;
    double T = T0 - L * h;
    double p = p0 * std::pow(1 - L * h / T0, (g / (R * L)));
    double rho = p / (R * T);

    std::cout << "At altitude " << h << " meters:\n";
    std::cout << "Temperature: " << T << " K\n";
    std::cout << "Pressure: " << p << " Pa\n";
    std::cout << "Density: " << rho << " kg/m^3\n";
    return 0;
}