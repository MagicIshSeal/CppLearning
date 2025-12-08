#define CATCH_CONFIG_MAIN  // This tells Catch to provide main()
#include "catch_amalgamated.hpp"
#include "../src/atmosphere.hpp"

// Tolerance for floating point comparisons
const double tol = 1e-2;

TEST_CASE("ISA temperature at known altitudes") {
    REQUIRE(std::abs(getTemperature(0) - 288.15) < tol);       // Sea level
    REQUIRE(std::abs(getTemperature(5000) - 288.15 + 0.0065*5000) < tol);
}

TEST_CASE("ISA pressure at known altitudes") {
    double p0_val = 101325.0;
    REQUIRE(std::abs(getPressure(0) - p0_val) < 1.0);
}

TEST_CASE("Density calculation") {
    double rho0 = 1.225;
    REQUIRE(std::abs(getDensity(0) - rho0) < 0.01);
}

TEST_CASE("Speed of sound calculation") {
    double a0 = 340.3; // m/s at sea level
    REQUIRE(std::abs(getSpeedOfSound(0) - a0) < 1.0);
}
