#include "integrator.hpp"

// Runge-Kutta 4th Order (RK4) integrator
// 
// WHY RK4?
// - 4th order accuracy: O(dt^4) error per step
// - Single-step method: only needs current state
// - Self-starting: no initialization needed
// - Stable and reliable for most physics problems
// - Good balance between accuracy and computational cost
//
// METHOD:
// For dy/dt = f(t, y), RK4 computes:
//   k1 = f(t, y)
//   k2 = f(t + dt/2, y + k1*dt/2)
//   k3 = f(t + dt/2, y + k2*dt/2)
//   k4 = f(t + dt, y + k3*dt)
//   y_next = y + (k1 + 2*k2 + 2*k3 + k4) * dt/6
//
// For our case:
//   position: dx/dt = velocity
//   velocity: dv/dt = acceleration
//
void integrateRK4(Vec2& position, Vec2& velocity, const Vec2& acceleration, double dt) {
    // k1: derivatives at current state
    Vec2 k1_vel = acceleration;
    Vec2 k1_pos = velocity;
    
    // k2: derivatives at midpoint using k1
    Vec2 vel_mid1 = velocity + k1_vel * (dt * 0.5);
    Vec2 k2_vel = acceleration;  // acceleration is constant over this step
    Vec2 k2_pos = vel_mid1;
    
    // k3: derivatives at midpoint using k2
    Vec2 vel_mid2 = velocity + k2_vel * (dt * 0.5);
    Vec2 k3_vel = acceleration;
    Vec2 k3_pos = vel_mid2;
    
    // k4: derivatives at endpoint using k3
    Vec2 vel_end = velocity + k3_vel * dt;
    Vec2 k4_vel = acceleration;
    Vec2 k4_pos = vel_end;
    
    // Update position and velocity using weighted average
    velocity = velocity + (k1_vel + k2_vel * 2.0 + k3_vel * 2.0 + k4_vel) * (dt / 6.0);
    position = position + (k1_pos + k2_pos * 2.0 + k3_pos * 2.0 + k4_pos) * (dt / 6.0);
}
