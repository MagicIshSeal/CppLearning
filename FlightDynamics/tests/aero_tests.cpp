#define CATCH_CONFIG_MAIN  // Catch provides main()
#include "catch_amalgamated.hpp"
#include "../src/aero.hpp"
#include "../src/atmosphere.hpp" // for g if needed

const double tol = 1e-6; // Tolerance for floating-point comparisons

TEST_CASE("Lift calculation") {
    double rho = 1.225;   // kg/m³ (sea level)
    double V = 50;        // m/s
    double S = 16;        // m²
    double CL = 0.5;      // sample lift coefficient

    double L = calcLift(rho, V, S, CL);
    double expected = 0.5 * rho * V * V * S * CL;
    REQUIRE( std::abs(L - expected) < tol );
}

TEST_CASE("Drag calculation") {
    double rho = 1.225;
    double V = 50;
    double S = 16;
    double CD = 0.02;

    double D = calcDrag(rho, V, S, CD);
    double expected = 0.5 * rho * V * V * S * CD;
    REQUIRE( std::abs(D - expected) < tol );
}

TEST_CASE("Weight calculation") {
    double mass = 1200; // kg
    double W = calcWeight(mass, g); // g from atmosphere.hpp
    double expected = mass * g;
    REQUIRE( std::abs(W - expected) < tol );
}

TEST_CASE("Thrust calculation") {
    double throttle = 0.7;
    double maxThrust = 5000; // N
    double T = calcThrust(throttle, maxThrust);
    double expected = throttle * maxThrust;
    REQUIRE( std::abs(T - expected) < tol );
}

TEST_CASE("Lift coefficient calculation") {
    double alpha = 5 * 3.14159/180; // 5 degrees in rad
    double CL_alpha = 5.7;
    double CL = calcCL(alpha, CL_alpha);
    double expected = CL_alpha * alpha;
    REQUIRE( std::abs(CL - expected) < tol );
}

TEST_CASE("Drag coefficient calculation") {
    double CL = 0.5;
    double CD0 = 0.02;
    double k = 0.04;
    double CD = calcCD(CL, CD0, k);
    double expected = CD0 + k * CL * CL;
    REQUIRE( std::abs(CD - expected) < tol );
}
