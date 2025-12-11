#pragma once

#include "imgui.h"
#include "../simulation/simulation_state.hpp"
#include "../environment/atmosphere.hpp"
#include "../aircraft/aircraft_loader.hpp"
#include <string>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Aircraft configuration entry
struct AircraftConfigUI
{
    std::string name;
    std::string filepath;
};

// UI state for panels
struct UIState
{
    bool show_demo;
    bool show_metrics;
    bool show_vectors;
    ImVec4 clear_color;
    float avg_fps;
    float avg_frame_time;

    std::string load_message;
    bool load_error;
    int selected_aircraft;
    std::vector<AircraftConfigUI> aircraft_configs;
    std::vector<std::string> aircraft_name_storage;
    std::vector<const char *> aircraft_names;

    UIState()
        : show_demo(false),
          show_metrics(false),
          show_vectors(true),
          clear_color(0.45f, 0.55f, 0.60f, 1.00f),
          avg_fps(0.0f),
          avg_frame_time(0.0f),
          load_message(""),
          load_error(false),
          selected_aircraft(0)
    {
    }
};

// Render the flight controls panel
inline void renderControlPanel(SimulationState &state, UIState &ui_state)
{
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Flight Controls");

    ImGui::Text("2D Flight Simulator");
    ImGui::Separator();

    // Pause/Resume and Reset buttons
    if (ImGui::Button(state.paused ? "Resume" : "Pause", ImVec2(120, 0)))
    {
        state.paused = !state.paused;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset", ImVec2(120, 0)))
    {
        state.reset_requested = true;
    }

    ImGui::Separator();
    ImGui::Text("Controls:");
    ImGui::SliderFloat("Throttle %%", &state.throttle, 0.0f, 1.0f, "%.2f");
    if (state.autopilot_speed)
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[AUTO]");
    }
    ImGui::SliderFloat("Pitch Angle (deg)", &state.pitch_deg, -10.0f, 15.0f, "%.1f");
    if (state.autopilot_altitude)
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[AUTO]");
    }

    // Speed Autopilot
    ImGui::Separator();
    ImGui::Text("Autopilot - Speed Control:");
    if (ImGui::Checkbox("Enable Speed Autopilot", &state.autopilot_speed))
    {
        if (state.autopilot_speed)
            state.speed_pid.reset();
    }

    if (state.autopilot_speed)
    {
        ImGui::SliderFloat("Target Speed (m/s)", &state.speed_setpoint, 10.0f, 100.0f, "%.1f");
        ImGui::Text("PID Gains:");
        if (ImGui::SliderFloat("Kp (Proportional)", &state.pid_kp, 0.0f, 0.1f, "%.4f"))
            state.speed_pid = PIDController(state.pid_kp, state.pid_ki, state.pid_kd, 0.0, 1.0);
        if (ImGui::SliderFloat("Ki (Integral)", &state.pid_ki, 0.0f, 0.01f, "%.5f"))
            state.speed_pid = PIDController(state.pid_kp, state.pid_ki, state.pid_kd, 0.0, 1.0);
        if (ImGui::SliderFloat("Kd (Derivative)", &state.pid_kd, 0.0f, 0.05f, "%.4f"))
            state.speed_pid = PIDController(state.pid_kp, state.pid_ki, state.pid_kd, 0.0, 1.0);

        ImGui::Text("PID Terms:");
        ImGui::Text("  P: %.4f  I: %.4f  D: %.4f",
                    state.speed_pid.getProportionalTerm(),
                    state.speed_pid.getIntegralTerm(),
                    state.speed_pid.getDerivativeTerm());
        ImGui::Text("Speed Error: %.2f m/s", state.speed_setpoint - state.velocity.magnitude());
    }

    // Altitude Autopilot
    ImGui::Separator();
    ImGui::Text("Autopilot - Altitude Control:");
    if (ImGui::Checkbox("Enable Altitude Autopilot", &state.autopilot_altitude))
    {
        if (state.autopilot_altitude)
            state.altitude_pid.reset();
    }

    if (state.autopilot_altitude)
    {
        ImGui::SliderFloat("Target Altitude (m)", &state.altitude_setpoint, 0.0f, 1000.0f, "%.1f");
        ImGui::Text("PID Gains:");
        if (ImGui::SliderFloat("Kp (Proportional)##alt", &state.alt_pid_kp, 0.0f, 1.0f, "%.4f"))
            state.altitude_pid = PIDController(state.alt_pid_kp, state.alt_pid_ki, state.alt_pid_kd, -10.0, 15.0);
        if (ImGui::SliderFloat("Ki (Integral)##alt", &state.alt_pid_ki, 0.0f, 0.01f, "%.5f"))
            state.altitude_pid = PIDController(state.alt_pid_kp, state.alt_pid_ki, state.alt_pid_kd, -10.0, 15.0);
        if (ImGui::SliderFloat("Kd (Derivative)##alt", &state.alt_pid_kd, 0.0f, 2.0f, "%.4f"))
            state.altitude_pid = PIDController(state.alt_pid_kp, state.alt_pid_ki, state.alt_pid_kd, -10.0, 15.0);

        ImGui::Text("PID Terms:");
        ImGui::Text("  P: %.4f  I: %.4f  D: %.4f",
                    state.altitude_pid.getProportionalTerm(),
                    state.altitude_pid.getIntegralTerm(),
                    state.altitude_pid.getDerivativeTerm());
        ImGui::Text("Altitude Error: %.2f m", state.altitude_setpoint - state.position.y);
    }

    // Flight Data
    ImGui::Separator();
    ImGui::Text("Flight Data:");
    ImGui::Text("Time:         %.1f s", state.t);
    ImGui::Text("Altitude:     %.1f m", state.position.y);
    ImGui::Text("Speed:        %.1f m/s (%.1f km/h)", state.velocity.magnitude(), state.velocity.magnitude() * 3.6);
    ImGui::Text("Distance:     %.1f m", state.position.x);
    ImGui::Text("Climb Angle:  %.2f deg", state.velocity.angle() * 180.0 / M_PI);
    ImGui::Text("Vertical Speed: %.1f m/s", state.velocity.y);
    ImGui::Text("Pitch Angle:  %.1f deg", state.pitch_deg);
    ImGui::Text("Angle of Attack: %.1f deg", state.alpha_deg);

    // Calculate current aerodynamic coefficients
    double current_alpha = state.alpha_deg * M_PI / 180.0;
    double current_CL, current_CD;
    if (state.aircraft.hasAeroTable())
    {
        current_CL = calcCL(current_alpha, state.aircraft.aeroTable.get());
        current_CD = calcCD(current_alpha, state.aircraft.aeroTable.get());
    }
    else
    {
        current_CL = calcCL(current_alpha, state.aircraft.CL_alpha);
        current_CD = calcCD(current_CL, state.aircraft.CD0, state.aircraft.k);
    }

    ImGui::Text("Current CL:   %.3f", current_CL);
    ImGui::Text("Current CD:   %.4f", current_CD);

    // Aircraft Info
    ImGui::Separator();
    ImGui::Text("Aircraft:");
    ImGui::Text("Mass:         %.0f kg", state.aircraft.mass);
    ImGui::Text("Wing Area:    %.1f m²", state.aircraft.S);
    ImGui::Text("Max Thrust:   %.0f N", state.aircraft.maxThrust);

    // Show which aerodynamic model is in use
    if (state.aircraft.hasAeroTable())
    {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "Aero Model:   Table-based");
        ImGui::Text("Data File:    %s", state.aircraft.aeroDataFile.c_str());
        ImGui::Text("CL (@ %.1f°):  %.3f", state.alpha_deg, current_CL);
        ImGui::Text("CD (@ %.1f°):  %.4f", state.alpha_deg, current_CD);
    }
    else
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Aero Model:   Legacy");
        ImGui::Text("CL_alpha:     %.2f", state.aircraft.CL_alpha);
        ImGui::Text("CD0:          %.3f", state.aircraft.CD0);
        ImGui::Text("k:            %.3f", state.aircraft.k);
    }

    // Aircraft Configuration Loader
    ImGui::Separator();
    ImGui::Text("Load Aircraft Configuration:");
    ImGui::SetNextItemWidth(150);
    if (ImGui::Combo("Aircraft Type", &ui_state.selected_aircraft, ui_state.aircraft_names.data(),
                     static_cast<int>(ui_state.aircraft_names.size())))
    {
    }

    ImGui::SameLine();
    if (ImGui::Button("Load Selected"))
    {
        try
        {
            if (ui_state.aircraft_configs[ui_state.selected_aircraft].filepath.empty())
            {
                state.aircraft = Aircraft();
                ui_state.load_message = std::string("Loaded: ") + ui_state.aircraft_configs[ui_state.selected_aircraft].name;
                ui_state.load_error = false;
            }
            else
            {
                state.aircraft = AircraftLoader::loadFromJSON(ui_state.aircraft_configs[ui_state.selected_aircraft].filepath);
                ui_state.load_message = std::string("Loaded: ") + ui_state.aircraft_configs[ui_state.selected_aircraft].name;
                ui_state.load_error = false;
            }
        }
        catch (const std::exception &e)
        {
            ui_state.load_message = std::string("Error: ") + e.what();
            ui_state.load_error = true;
        }
    }

    if (!ui_state.load_message.empty())
    {
        if (ui_state.load_error)
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%s", ui_state.load_message.c_str());
        else
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s", ui_state.load_message.c_str());
    }

    // Visualization and Performance
    ImGui::Separator();
    ImGui::Text("Performance:");
    ImGui::Text("FPS:          %.1f", ui_state.avg_fps);
    ImGui::Text("Frame Time:   %.2f ms", ui_state.avg_frame_time);
    ImGui::Text("Sim Step:     %.3f ms", state.dt * 1000.0);

#ifdef NDEBUG
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Build: Release");
#else
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Build: Debug");
#endif

    ImGui::Separator();
    ImGui::Checkbox("Show Demo Window", &ui_state.show_demo);
    ImGui::Checkbox("Show Metrics", &ui_state.show_metrics);

    ImGui::End();
}

// Render the instrumentation panel
inline void renderInstrumentationPanel(const SimulationState &state)
{
    ImGui::SetNextWindowPos(ImVec2(420, 520), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(850, 190), ImGuiCond_FirstUseEver);
    ImGui::Begin("Instrumentation");

    // Gauges
    ImGui::BeginGroup();
    ImGui::Text("Altitude");
    ImGui::ProgressBar(static_cast<float>(state.position.y / 1000.0), ImVec2(0.0f, 0.0f));
    ImGui::Text("%.0f m", state.position.y);
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Text("Airspeed");
    ImGui::ProgressBar(static_cast<float>(state.velocity.magnitude() / 100.0), ImVec2(0.0f, 0.0f));
    ImGui::Text("%.0f m/s", state.velocity.magnitude());
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Text("Throttle");
    ImGui::ProgressBar(state.throttle, ImVec2(0.0f, 0.0f));
    ImGui::Text("%.0f %%", state.throttle * 100.0f);
    ImGui::EndGroup();

    ImGui::Separator();

    // Atmospheric data
    double alt = std::max(0.0, state.position.y);
    ImGui::Text("Atmospheric Conditions:");
    ImGui::Text("Temperature: %.1f °C", getTemperature(alt) - 273.15);
    ImGui::Text("Pressure:    %.0f Pa", getPressure(alt));
    ImGui::Text("Density:     %.3f kg/m³", getDensity(alt));
    ImGui::Text("Sound Speed: %.1f m/s", getSpeedOfSound(alt));

    ImGui::End();
}
