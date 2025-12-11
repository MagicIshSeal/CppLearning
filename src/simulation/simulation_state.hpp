#pragma once

#include "../core/vec2.hpp"
#include "../aircraft/aircraft.hpp"
#include "../control/pid.hpp"
#include <vector>

// Flight history point for visualization
struct FlightPoint
{
    float x, z;
};

// Main simulation state
class SimulationState
{
public:
    // Aircraft and physics state
    Aircraft aircraft;
    Vec2 position;
    Vec2 velocity;
    double t;
    double dt;

    // Control inputs
    float throttle;
    float elevator;   // Elevator control stick (-1 to +1, + is nose up)
    float pitch_deg;  // Pitch angle (orientation of aircraft)
    float pitch_rate; // Pitch rate (deg/s)
    float alpha_deg;  // Angle of attack (calculated)
    bool paused;
    bool reset_requested;

    // Autopilot - Speed Control
    bool autopilot_speed;
    float speed_setpoint;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    PIDController speed_pid;
    float prev_pid_kp, prev_pid_ki, prev_pid_kd;

    // Autopilot - Altitude Control
    bool autopilot_altitude;
    float altitude_setpoint;
    float alt_pid_kp;
    float alt_pid_ki;
    float alt_pid_kd;
    PIDController altitude_pid;
    float prev_alt_pid_kp, prev_alt_pid_ki, prev_alt_pid_kd;

    // Flight path history
    std::vector<FlightPoint> flightPath;
    int maxPathPoints;

    // Force vectors for visualization
    Vec2 F_thrust_viz;
    Vec2 F_drag_viz;
    Vec2 F_lift_viz;
    Vec2 F_weight_viz;

    SimulationState()
        : aircraft(),
          position(0.0, 0.0),
          velocity(0.0, 0.0),
          t(0.0),
          dt(0.016),
          throttle(0.0f),
          elevator(0.0f),
          pitch_deg(0.0f),
          pitch_rate(0.0f),
          alpha_deg(0.0f),
          paused(false),
          reset_requested(false),
          autopilot_speed(false),
          speed_setpoint(40.0f),
          pid_kp(0.02f),
          pid_ki(0.001f),
          pid_kd(0.01f),
          speed_pid(0.02f, 0.001f, 0.01f, 0.0, 1.0),
          prev_pid_kp(0.02f),
          prev_pid_ki(0.001f),
          prev_pid_kd(0.01f),
          autopilot_altitude(false),
          altitude_setpoint(100.0f),
          alt_pid_kp(0.1f),
          alt_pid_ki(0.001f),
          alt_pid_kd(0.5f),
          altitude_pid(0.1f, 0.001f, 0.5f, -10.0, 15.0),
          prev_alt_pid_kp(0.1f),
          prev_alt_pid_ki(0.001f),
          prev_alt_pid_kd(0.5f),
          maxPathPoints(1000),
          F_thrust_viz(0.0, 0.0),
          F_drag_viz(0.0, 0.0),
          F_lift_viz(0.0, 0.0),
          F_weight_viz(0.0, 0.0)
    {
    }

    void reset()
    {
        position = Vec2(0.0, 0.0);
        velocity = Vec2(0.0, 0.0);
        throttle = 0.3f;
        elevator = 0.0f;
        pitch_deg = 5.0f;
        pitch_rate = 0.0f;
        alpha_deg = 0.0f;
        t = 0.0;
        flightPath.clear();
        speed_pid.reset();
        altitude_pid.reset();
        reset_requested = false;
    }
};
