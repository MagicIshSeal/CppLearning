// FlightDynamics GUI - Interactive 2D Flight Simulator with Dear ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <cmath>
#include <vector>
#include "atmosphere.hpp"
#include "aero.hpp"
#include "integrator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Aircraft parameters
struct Aircraft {
    double mass = 1200.0;      // kg
    double S = 16.0;           // wing area [m^2]
    double CL_alpha = 5.7;     // lift curve slope [1/rad]
    double CD0 = 0.025;        // parasitic drag coefficient
    double k = 0.04;           // induced drag factor
    double maxThrust = 5000.0; // maximum thrust [N]
};

// Flight history for visualization
struct FlightPoint {
    float x, z;
};

int main(int, char**)
{
    // Initialize SDL
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init Error: %s", SDL_GetError());
        return -1;
    }

    // Setup OpenGL context
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    // Create window
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    SDL_Window* window = SDL_CreateWindow("FlightDynamics - 2D Flight Simulator", 
                                          1280, 720, window_flags);
    if (window == NULL) {
        SDL_Log("SDL_CreateWindow Error: %s", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == NULL) {
        SDL_Log("SDL_GL_CreateContext Error: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync

    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    
    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Simulation state
    Aircraft aircraft;
    AircraftState state{0.0, 0.0, 30.0, 0.0};  // Start with 30 m/s forward speed
    
    double t = 0.0;
    double dt = 0.016;  // ~60 FPS
    
    // Control inputs
    float throttle = 0.3f;      // Start with 30% throttle
    float alpha_deg = 5.0f;     // Start with 5 degree angle of attack
    bool paused = false;
    bool reset_requested = false;
    
    // Flight path history
    std::vector<FlightPoint> flightPath;
    const int maxPathPoints = 1000;
    
    // Display options
    bool show_demo = false;
    bool show_metrics = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT)
                done = true;
            if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && 
                event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // Reset if requested
        if (reset_requested) {
            state = AircraftState{0.0, 0.0, 30.0, 0.0};  // Reset to 30 m/s
            throttle = 0.3f;
            alpha_deg = 5.0f;
            t = 0.0;
            flightPath.clear();
            reset_requested = false;
        }

        // Simulation step
        if (!paused) {
            double altitude = state.z;
            double alpha = alpha_deg * M_PI / 180.0;  // Convert to radians
            
            // Get atmospheric properties
            double rho = getDensity(std::max(0.0, altitude));
            
            // Calculate aerodynamic coefficients
            double CL = calcCL(alpha, aircraft.CL_alpha);
            double CD = calcCD(CL, aircraft.CD0, aircraft.k);
            
            // Calculate forces
            double L = calcLift(rho, state.V, aircraft.S, CL);
            double D = calcDrag(rho, state.V, aircraft.S, CD);
            double W = calcWeight(aircraft.mass, g);
            double T = calcThrust(throttle, aircraft.maxThrust);
            
            // Integrate equations of motion
            state = trapezoidalStep(state, L, D, W, T, aircraft.mass, dt);
            
            // Ground constraint
            if (state.z < 0.0) {
                state.z = 0.0;
                state.gamma = 0.0;
                if (state.V < 1.0) state.V = 0.0;
            }
            
            // Update flight path
            if (flightPath.size() < maxPathPoints) {
                flightPath.push_back({static_cast<float>(state.x), static_cast<float>(state.z)});
            } else {
                flightPath.erase(flightPath.begin());
                flightPath.push_back({static_cast<float>(state.x), static_cast<float>(state.z)});
            }
            
            t += dt;
        }

        // === Control Panel ===
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
        ImGui::Begin("Flight Controls");
        
        ImGui::Text("2D Flight Simulator");
        ImGui::Separator();
        
        // Pause/Resume button
        if (ImGui::Button(paused ? "Resume" : "Pause", ImVec2(120, 0))) {
            paused = !paused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(120, 0))) {
            reset_requested = true;
        }
        
        ImGui::Separator();
        ImGui::Text("Controls:");
        ImGui::SliderFloat("Throttle %%", &throttle, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Angle of Attack (deg)", &alpha_deg, -10.0f, 15.0f, "%.1f");
        
        ImGui::Separator();
        ImGui::Text("Flight Data:");
        ImGui::Text("Time:         %.1f s", t);
        ImGui::Text("Altitude:     %.1f m", state.z);
        ImGui::Text("Speed:        %.1f m/s (%.1f km/h)", state.V, state.V * 3.6);
        ImGui::Text("Distance:     %.1f m", state.x);
        ImGui::Text("Climb Angle:  %.2f deg", state.gamma * 180.0 / M_PI);
        ImGui::Text("Vertical Speed: %.1f m/s", state.V * sin(state.gamma));
        
        ImGui::Separator();
        ImGui::Text("Aircraft:");
        ImGui::Text("Mass:         %.0f kg", aircraft.mass);
        ImGui::Text("Wing Area:    %.1f m²", aircraft.S);
        ImGui::Text("Max Thrust:   %.0f N", aircraft.maxThrust);
        
        ImGui::Separator();
        ImGui::Checkbox("Show Demo Window", &show_demo);
        ImGui::Checkbox("Show Metrics", &show_metrics);
        
        ImGui::End();

        // === Flight Path Visualization ===
        ImGui::SetNextWindowPos(ImVec2(420, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(850, 500), ImGuiCond_FirstUseEver);
        ImGui::Begin("Flight Path Visualization");
        
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
        
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
        draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));
        
        // Draw ground
        float ground_y = canvas_p1.y;
        draw_list->AddLine(ImVec2(canvas_p0.x, ground_y), ImVec2(canvas_p1.x, ground_y), 
                          IM_COL32(100, 200, 100, 255), 2.0f);
        
        // Draw flight path
        if (flightPath.size() > 1) {
            // Calculate scale
            float max_x = 1000.0f;  // meters
            float max_z = 800.0f;    // meters
            
            for (size_t i = 0; i < flightPath.size() - 1; i++) {
                float x1 = canvas_p0.x + (flightPath[i].x / max_x) * canvas_sz.x;
                float y1 = canvas_p1.y - (flightPath[i].z / max_z) * canvas_sz.y;
                float x2 = canvas_p0.x + (flightPath[i+1].x / max_x) * canvas_sz.x;
                float y2 = canvas_p1.y - (flightPath[i+1].z / max_z) * canvas_sz.y;
                
                // Clamp to canvas
                x1 = std::max(canvas_p0.x, std::min(canvas_p1.x, x1));
                x2 = std::max(canvas_p0.x, std::min(canvas_p1.x, x2));
                y1 = std::max(canvas_p0.y, std::min(canvas_p1.y, y1));
                y2 = std::max(canvas_p0.y, std::min(canvas_p1.y, y2));
                
                draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), 
                                  IM_COL32(255, 255, 0, 255), 2.0f);
            }
            
            // Draw aircraft position
            if (!flightPath.empty()) {
                float curr_x = canvas_p0.x + (state.x / max_x) * canvas_sz.x;
                float curr_y = canvas_p1.y - (state.z / max_z) * canvas_sz.y;
                curr_x = std::max(canvas_p0.x, std::min(canvas_p1.x, curr_x));
                curr_y = std::max(canvas_p0.y, std::min(canvas_p1.y, curr_y));
                
                draw_list->AddCircleFilled(ImVec2(curr_x, curr_y), 5.0f, IM_COL32(255, 0, 0, 255));
            }
        }
        
        ImGui::Text("Scale: X=%.0fm, Z=%.0fm", 1000.0f, 800.0f);
        ImGui::End();

        // === Instrumentation Panel ===
        ImGui::SetNextWindowPos(ImVec2(420, 520), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(850, 190), ImGuiCond_FirstUseEver);
        ImGui::Begin("Instrumentation");
        
        // Altitude gauge
        ImGui::BeginGroup();
        ImGui::Text("Altitude");
        ImGui::ProgressBar(static_cast<float>(state.z / 1000.0), ImVec2(0.0f, 0.0f));
        ImGui::Text("%.0f m", state.z);
        ImGui::EndGroup();
        
        ImGui::SameLine();
        
        // Speed gauge
        ImGui::BeginGroup();
        ImGui::Text("Airspeed");
        ImGui::ProgressBar(static_cast<float>(state.V / 100.0), ImVec2(0.0f, 0.0f));
        ImGui::Text("%.0f m/s", state.V);
        ImGui::EndGroup();
        
        ImGui::SameLine();
        
        // Throttle gauge
        ImGui::BeginGroup();
        ImGui::Text("Throttle");
        ImGui::ProgressBar(throttle, ImVec2(0.0f, 0.0f));
        ImGui::Text("%.0f %%", throttle * 100.0f);
        ImGui::EndGroup();
        
        ImGui::Separator();
        
        // Atmospheric data
        double alt_for_atm = std::max(0.0, state.z);
        ImGui::Text("Atmospheric Conditions:");
        ImGui::Text("Temperature: %.1f °C", getTemperature(alt_for_atm) - 273.15);
        ImGui::Text("Pressure:    %.0f Pa", getPressure(alt_for_atm));
        ImGui::Text("Density:     %.3f kg/m³", getDensity(alt_for_atm));
        ImGui::Text("Sound Speed: %.1f m/s", getSpeedOfSound(alt_for_atm));
        
        ImGui::End();

        // Optional windows
        if (show_demo) ImGui::ShowDemoWindow(&show_demo);
        if (show_metrics) ImGui::ShowMetricsWindow(&show_metrics);

        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, 
                     clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DestroyContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
