#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include "vec2.hpp"

// Generic RK4 integrator for position and velocity
// Uses Runge-Kutta 4th order method (RK4)
void integrateRK4(Vec2& position, Vec2& velocity, const Vec2& acceleration, double dt);

#endif
