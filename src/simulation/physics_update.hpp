#pragma once

#include "simulation_state.hpp"
#include "../environment/atmosphere.hpp"
#include "../aerodynamics/aero.hpp"
#include "../core/integrator.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Update simulation physics for one timestep
inline void updatePhysics(SimulationState &state)
{
    if (state.paused)
        return;

    double altitude = state.position.y;
    double speed = state.velocity.magnitude();

    // Autopilot: Speed control with PID
    if (state.autopilot_speed)
    {
        if (state.pid_kp != state.prev_pid_kp || state.pid_ki != state.prev_pid_ki || state.pid_kd != state.prev_pid_kd)
        {
            state.speed_pid = PIDController(state.pid_kp, state.pid_ki, state.pid_kd, 0.0, 1.0);
            state.prev_pid_kp = state.pid_kp;
            state.prev_pid_ki = state.pid_ki;
            state.prev_pid_kd = state.pid_kd;
        }
        state.throttle = static_cast<float>(state.speed_pid.update(state.speed_setpoint, speed, state.dt));
    }

    // Autopilot: Altitude control with PID
    if (state.autopilot_altitude)
    {
        if (state.alt_pid_kp != state.prev_alt_pid_kp || state.alt_pid_ki != state.prev_alt_pid_ki || state.alt_pid_kd != state.prev_alt_pid_kd)
        {
            state.altitude_pid = PIDController(state.alt_pid_kp, state.alt_pid_ki, state.alt_pid_kd, -10.0, 15.0);
            state.prev_alt_pid_kp = state.alt_pid_kp;
            state.prev_alt_pid_ki = state.alt_pid_ki;
            state.prev_alt_pid_kd = state.alt_pid_kd;
        }
        state.alpha_deg = static_cast<float>(state.altitude_pid.update(state.altitude_setpoint, altitude, state.dt));
    }

    Vec2 velocityDir = (speed > 1e-6) ? state.velocity.normalized() : Vec2(1.0, 0.0);
    double alpha = state.alpha_deg * M_PI / 180.0;

    // Get atmospheric properties
    double rho = getDensity(std::max(0.0, altitude));

    // Calculate aerodynamic coefficients
    double CL, CD;
    if (state.aircraft.hasAeroTable())
    {
        // Use table-based data
        CL = calcCL(alpha, state.aircraft.aeroTable.get());
        CD = calcCD(alpha, state.aircraft.aeroTable.get());
    }
    else
    {
        // Use legacy linear/parabolic model
        CL = calcCL(alpha, state.aircraft.CL_alpha);
        CD = calcCD(CL, state.aircraft.CD0, state.aircraft.k);
    }

    // Calculate force magnitudes
    double L_mag = calcLift(rho, speed, state.aircraft.S, CL);
    double D_mag = calcDrag(rho, speed, state.aircraft.S, CD);
    double W_mag = calcWeight(state.aircraft.mass, g);
    double T_mag = calcThrust(state.throttle, state.aircraft.maxThrust);

    // Force vectors
    Vec2 F_thrust = velocityDir.rotated(alpha) * T_mag;
    Vec2 F_drag = (speed > 1e-6) ? velocityDir * (-D_mag) : Vec2(0.0, 0.0);
    Vec2 F_lift = velocityDir.rotated(M_PI / 2.0) * L_mag;
    Vec2 F_weight(0.0, -W_mag);

    // Net force and acceleration
    Vec2 F_net = F_thrust + F_drag + F_lift + F_weight;
    Vec2 acceleration = F_net / state.aircraft.mass;

    // Store force vectors for visualization
    state.F_thrust_viz = F_thrust;
    state.F_drag_viz = F_drag;
    state.F_lift_viz = F_lift;
    state.F_weight_viz = F_weight;

    // Integrate using RK4
    integrateRK4(state.position, state.velocity, acceleration, state.dt);

    // Ground constraint
    if (state.position.y < 0.0)
    {
        state.position.y = 0.0;
        if (state.velocity.y < 0.0)
            state.velocity.y = 0.0;
        if (state.velocity.magnitude() < 0.1 && state.throttle < 0.01)
            state.velocity = Vec2(0.0, 0.0);
    }

    // Update flight path
    if (state.flightPath.size() < static_cast<size_t>(state.maxPathPoints))
    {
        state.flightPath.push_back({static_cast<float>(state.position.x), static_cast<float>(state.position.y)});
    }
    else
    {
        state.flightPath.erase(state.flightPath.begin());
        state.flightPath.push_back({static_cast<float>(state.position.x), static_cast<float>(state.position.y)});
    }

    state.t += state.dt;
}
