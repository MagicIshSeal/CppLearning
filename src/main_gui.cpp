// FlightDynamics GUI - Vector-Based 2D Flight Simulator with Dear ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <cmath>
#include <vector>
#include "vec2.hpp"
#include "atmosphere.hpp"
#include "aero.hpp"
#include "integrator.hpp"
#include "pid.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Aircraft parameters
struct Aircraft
{
    double mass = 120.0;      // kg
    double S = 1.60;          // wing area [m^2]
    double CL_alpha = 5.7;    // lift curve slope [1/rad]
    double CD0 = 0.025;       // parasitic drag coefficient
    double k = 0.04;          // induced drag factor
    double maxThrust = 500.0; // maximum thrust [N]
};

// Flight history for visualization
struct FlightPoint
{
    float x, z;
};

int main(int, char **)
{
    // Initialize SDL
    if (!SDL_Init(SDL_INIT_VIDEO))
    {
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
    SDL_Window *window = SDL_CreateWindow("FlightDynamics - 2D Flight Simulator",
                                          1280, 720, window_flags);
    if (window == NULL)
    {
        SDL_Log("SDL_CreateWindow Error: %s", SDL_GetError());
        SDL_Quit();
        return -1;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == NULL)
    {
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
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Simulation state - using Vec2 vectors
    Aircraft aircraft;
    Vec2 position(0.0, 0.0); // (x, z) position in m
    Vec2 velocity(0.0, 0.0); // (x, z) velocity in m/s - start with horizontal velocity

    double t = 0.0;
    double dt = 0.016; // ~60 FPS

    // Control inputs
    float throttle = 0.0f;  // Start with 0% throttle
    float alpha_deg = 0.0f; // Start with 0 degree angle of attack
    bool paused = false;
    bool reset_requested = false;

    // Autopilot - Speed Control
    bool autopilot_speed = false;                                           // Autopilot on/off
    float speed_setpoint = 40.0f;                                           // Target speed in m/s
    float pid_kp = 0.02f;                                                   // Proportional gain
    float pid_ki = 0.001f;                                                  // Integral gain
    float pid_kd = 0.01f;                                                   // Derivative gain
    PIDController speed_pid(pid_kp, pid_ki, pid_kd, 0.0, 1.0);              // Output: throttle 0-1
    float prev_pid_kp = pid_kp, prev_pid_ki = pid_ki, prev_pid_kd = pid_kd; // Track gain changes

    // Autopilot - Altitude Control
    bool autopilot_altitude = false;                                                                // Autopilot on/off
    float altitude_setpoint = 100.0f;                                                               // Target altitude in m
    float alt_pid_kp = 0.1f;                                                                        // Proportional gain
    float alt_pid_ki = 0.001f;                                                                      // Integral gain
    float alt_pid_kd = 0.5f;                                                                        // Derivative gain
    PIDController altitude_pid(alt_pid_kp, alt_pid_ki, alt_pid_kd, -10.0, 15.0);                    // Output: alpha -10 to 15 deg
    float prev_alt_pid_kp = alt_pid_kp, prev_alt_pid_ki = alt_pid_ki, prev_alt_pid_kd = alt_pid_kd; // Track gain changes

    // Flight path history
    std::vector<FlightPoint> flightPath;
    const int maxPathPoints = 1000;

    // Display options
    bool show_demo = false;
    bool show_metrics = false;
    bool show_vectors = true; // Show force vectors by default
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Store force vectors for visualization
    Vec2 F_thrust_viz(0.0, 0.0);
    Vec2 F_drag_viz(0.0, 0.0);
    Vec2 F_lift_viz(0.0, 0.0);
    Vec2 F_weight_viz(0.0, 0.0);

    // Camera/view controls
    ImVec2 view_offset(0.0f, 0.0f); // Pan offset in pixels
    float view_scale = 1.0f;        // Zoom scale (pixels per meter)
    bool is_dragging = false;
    ImVec2 drag_start_pos(0.0f, 0.0f);
    ImVec2 drag_start_offset(0.0f, 0.0f);
    float vector_scale = 0.05f; // Scale for force vectors
    bool auto_follow = true;    // Auto-follow aircraft

    // Performance tracking
    Uint64 last_frame_time = SDL_GetPerformanceCounter();
    Uint64 perf_frequency = SDL_GetPerformanceFrequency();
    float frame_times[60] = {0}; // Track last 60 frame times
    int frame_time_index = 0;
    float avg_frame_time = 0.0f;
    float avg_fps = 0.0f;
    int frame_count = 0;

    // Main loop
    bool done = false;
    while (!done)
    {
        // Measure frame time
        Uint64 current_frame_time = SDL_GetPerformanceCounter();
        float delta_time = (float)(current_frame_time - last_frame_time) / perf_frequency;
        last_frame_time = current_frame_time;

        // Update frame time history
        frame_times[frame_time_index] = delta_time * 1000.0f; // Convert to milliseconds
        frame_time_index = (frame_time_index + 1) % 60;
        frame_count++;

        // Calculate average frame time and FPS every 10 frames
        if (frame_count % 10 == 0)
        {
            float sum = 0.0f;
            for (int i = 0; i < 60; i++)
            {
                sum += frame_times[i];
            }
            avg_frame_time = sum / 60.0f;
            avg_fps = avg_frame_time > 0.0f ? 1000.0f / avg_frame_time : 0.0f;
        }

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
        if (reset_requested)
        {
            position = Vec2(0.0, 0.0);
            velocity = Vec2(0.0, 0.0);
            throttle = 0.3f;
            alpha_deg = 5.0f;
            t = 0.0;
            flightPath.clear();
            speed_pid.reset();    // Reset PID state
            altitude_pid.reset(); // Reset PID state
            reset_requested = false;
        }

        // Simulation step
        if (!paused)
        {
            double altitude = position.y; // z-component is altitude
            double speed = velocity.magnitude();

            // Autopilot: Speed control with PID
            if (autopilot_speed)
            {
                // Only recreate PID controller if gains changed (preserves internal state)
                if (pid_kp != prev_pid_kp || pid_ki != prev_pid_ki || pid_kd != prev_pid_kd)
                {
                    speed_pid = PIDController(pid_kp, pid_ki, pid_kd, 0.0, 1.0);
                    prev_pid_kp = pid_kp;
                    prev_pid_ki = pid_ki;
                    prev_pid_kd = pid_kd;
                }

                // PID updates throttle to maintain target speed
                throttle = static_cast<float>(speed_pid.update(speed_setpoint, speed, dt));
            }

            // Autopilot: Altitude control with PID
            if (autopilot_altitude)
            {
                // Only recreate PID controller if gains changed (preserves internal state)
                if (alt_pid_kp != prev_alt_pid_kp || alt_pid_ki != prev_alt_pid_ki || alt_pid_kd != prev_alt_pid_kd)
                {
                    altitude_pid = PIDController(alt_pid_kp, alt_pid_ki, alt_pid_kd, -10.0, 15.0);
                    prev_alt_pid_kp = alt_pid_kp;
                    prev_alt_pid_ki = alt_pid_ki;
                    prev_alt_pid_kd = alt_pid_kd;
                }

                // PID updates angle of attack to maintain target altitude
                alpha_deg = static_cast<float>(altitude_pid.update(altitude_setpoint, altitude, dt));
            }

            Vec2 velocityDir = (speed > 1e-6) ? velocity.normalized() : Vec2(1.0, 0.0);
            double alpha = alpha_deg * M_PI / 180.0; // Convert to radians

            // Get atmospheric properties
            double rho = getDensity(std::max(0.0, altitude));

            // Calculate aerodynamic coefficients
            double CL = calcCL(alpha, aircraft.CL_alpha);
            double CD = calcCD(CL, aircraft.CD0, aircraft.k);

            // Calculate force magnitudes
            double L_mag = calcLift(rho, speed, aircraft.S, CL);
            double D_mag = calcDrag(rho, speed, aircraft.S, CD);
            double W_mag = calcWeight(aircraft.mass, g);
            double T_mag = calcThrust(throttle, aircraft.maxThrust);

            // === FORCE VECTORS ===
            // 1. THRUST: Rotate velocity direction by angle of attack
            Vec2 F_thrust = velocityDir.rotated(alpha) * T_mag;

            // 2. DRAG: Acts opposite to velocity
            Vec2 F_drag = (speed > 1e-6) ? velocityDir * (-D_mag) : Vec2(0.0, 0.0);

            // 3. LIFT: Acts perpendicular to velocity (rotate velocity by 90 degrees)
            Vec2 F_lift = velocityDir.rotated(M_PI / 2.0) * L_mag;

            // 4. WEIGHT: Always downward
            Vec2 F_weight(0.0, -W_mag);

            // Net force and acceleration
            Vec2 F_net = F_thrust + F_drag + F_lift + F_weight;
            Vec2 acceleration = F_net / aircraft.mass;

            // Store force vectors for visualization
            F_thrust_viz = F_thrust;
            F_drag_viz = F_drag;
            F_lift_viz = F_lift;
            F_weight_viz = F_weight;

            // Integrate using RK4
            integrateRK4(position, velocity, acceleration, dt);

            // Ground constraint
            if (position.y < 0.0)
            {
                position.y = 0.0;
                // Zero out vertical velocity if moving downward
                if (velocity.y < 0.0)
                    velocity.y = 0.0;
                // Apply simple ground friction when on ground and moving slowly
                if (velocity.magnitude() < 0.1 && throttle < 0.01)
                    velocity = Vec2(0.0, 0.0);
            }

            // Update flight path
            if (flightPath.size() < maxPathPoints)
            {
                flightPath.push_back({static_cast<float>(position.x), static_cast<float>(position.y)});
            }
            else
            {
                flightPath.erase(flightPath.begin());
                flightPath.push_back({static_cast<float>(position.x), static_cast<float>(position.y)});
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
        if (ImGui::Button(paused ? "Resume" : "Pause", ImVec2(120, 0)))
        {
            paused = !paused;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(120, 0)))
        {
            reset_requested = true;
        }

        ImGui::Separator();
        ImGui::Text("Controls:");
        ImGui::SliderFloat("Throttle %%", &throttle, 0.0f, 1.0f, "%.2f");
        if (autopilot_speed)
        {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[AUTO]");
        }
        ImGui::SliderFloat("Angle of Attack (deg)", &alpha_deg, -10.0f, 15.0f, "%.1f");
        if (autopilot_altitude)
        {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[AUTO]");
        }

        ImGui::Separator();
        ImGui::Text("Autopilot - Speed Control:");
        if (ImGui::Checkbox("Enable Speed Autopilot", &autopilot_speed))
        {
            if (autopilot_speed)
            {
                speed_pid.reset(); // Reset PID when enabling
            }
        }

        if (autopilot_speed)
        {
            ImGui::SliderFloat("Target Speed (m/s)", &speed_setpoint, 10.0f, 100.0f, "%.1f");
            ImGui::Text("PID Gains:");
            if (ImGui::SliderFloat("Kp (Proportional)", &pid_kp, 0.0f, 0.1f, "%.4f"))
            {
                speed_pid = PIDController(pid_kp, pid_ki, pid_kd, 0.0, 1.0);
            }
            if (ImGui::SliderFloat("Ki (Integral)", &pid_ki, 0.0f, 0.01f, "%.5f"))
            {
                speed_pid = PIDController(pid_kp, pid_ki, pid_kd, 0.0, 1.0);
            }
            if (ImGui::SliderFloat("Kd (Derivative)", &pid_kd, 0.0f, 0.05f, "%.4f"))
            {
                speed_pid = PIDController(pid_kp, pid_ki, pid_kd, 0.0, 1.0);
            }

            // Display PID debug info
            ImGui::Text("PID Terms:");
            ImGui::Text("  P: %.4f  I: %.4f  D: %.4f",
                        speed_pid.getProportionalTerm(),
                        speed_pid.getIntegralTerm(),
                        speed_pid.getDerivativeTerm());

            double speed_error = speed_setpoint - velocity.magnitude();
            ImGui::Text("Speed Error: %.2f m/s", speed_error);
        }

        ImGui::Separator();
        ImGui::Text("Autopilot - Altitude Control:");
        if (ImGui::Checkbox("Enable Altitude Autopilot", &autopilot_altitude))
        {
            if (autopilot_altitude)
            {
                altitude_pid.reset(); // Reset PID when enabling
            }
        }

        if (autopilot_altitude)
        {
            ImGui::SliderFloat("Target Altitude (m)", &altitude_setpoint, 0.0f, 1000.0f, "%.1f");
            ImGui::Text("PID Gains:");
            if (ImGui::SliderFloat("Kp (Proportional)##alt", &alt_pid_kp, 0.0f, 1.0f, "%.4f"))
            {
                altitude_pid = PIDController(alt_pid_kp, alt_pid_ki, alt_pid_kd, -10.0, 15.0);
            }
            if (ImGui::SliderFloat("Ki (Integral)##alt", &alt_pid_ki, 0.0f, 0.01f, "%.5f"))
            {
                altitude_pid = PIDController(alt_pid_kp, alt_pid_ki, alt_pid_kd, -10.0, 15.0);
            }
            if (ImGui::SliderFloat("Kd (Derivative)##alt", &alt_pid_kd, 0.0f, 2.0f, "%.4f"))
            {
                altitude_pid = PIDController(alt_pid_kp, alt_pid_ki, alt_pid_kd, -10.0, 15.0);
            }

            // Display PID debug info
            ImGui::Text("PID Terms:");
            ImGui::Text("  P: %.4f  I: %.4f  D: %.4f",
                        altitude_pid.getProportionalTerm(),
                        altitude_pid.getIntegralTerm(),
                        altitude_pid.getDerivativeTerm());

            double altitude_error = altitude_setpoint - position.y;
            ImGui::Text("Altitude Error: %.2f m", altitude_error);
        }

        ImGui::Separator();
        ImGui::Text("Flight Data:");
        ImGui::Text("Time:         %.1f s", t);
        ImGui::Text("Altitude:     %.1f m", position.y);
        ImGui::Text("Speed:        %.1f m/s (%.1f km/h)", velocity.magnitude(), velocity.magnitude() * 3.6);
        ImGui::Text("Distance:     %.1f m", position.x);
        ImGui::Text("Climb Angle:  %.2f deg", velocity.angle() * 180.0 / M_PI);
        ImGui::Text("Vertical Speed: %.1f m/s", velocity.y);

        ImGui::Separator();
        ImGui::Text("Aircraft:");
        ImGui::Text("Mass:         %.0f kg", aircraft.mass);
        ImGui::Text("Wing Area:    %.1f m²", aircraft.S);
        ImGui::Text("Max Thrust:   %.0f N", aircraft.maxThrust);

        ImGui::Separator();
        ImGui::Text("Visualization:");
        ImGui::Checkbox("Show Force Vectors", &show_vectors);
        if (show_vectors)
        {
            ImGui::SliderFloat("Vector Scale", &vector_scale, 0.001f, 0.2f, "%.3f", ImGuiSliderFlags_Logarithmic);
        }

        ImGui::Separator();
        ImGui::Text("Performance:");
        ImGui::Text("FPS:          %.1f", avg_fps);
        ImGui::Text("Frame Time:   %.2f ms", avg_frame_time);
        ImGui::Text("Sim Step:     %.3f ms", dt * 1000.0);

// Show build configuration
#ifdef NDEBUG
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Build: Release");
#else
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Build: Debug");
#endif

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
        if (canvas_sz.x < 50.0f)
            canvas_sz.x = 50.0f;
        if (canvas_sz.y < 50.0f)
            canvas_sz.y = 50.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        // Enable clipping for the canvas
        draw_list->PushClipRect(canvas_p0, canvas_p1, true);

        draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
        draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

        // Handle mouse interactions for pan and zoom
        ImGui::SetCursorScreenPos(canvas_p0);
        ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        bool is_hovered = ImGui::IsItemHovered();

        // Mouse dragging for panning
        if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            is_dragging = true;
            drag_start_pos = ImGui::GetMousePos();
            drag_start_offset = view_offset;
            auto_follow = false; // Disable auto-follow when user manually pans
        }
        if (is_dragging)
        {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
            {
                ImVec2 mouse_pos = ImGui::GetMousePos();
                view_offset.x = drag_start_offset.x + (mouse_pos.x - drag_start_pos.x);
                view_offset.y = drag_start_offset.y + (mouse_pos.y - drag_start_pos.y);
            }
            else
            {
                is_dragging = false;
            }
        }

        // Mouse wheel for zooming
        if (is_hovered)
        {
            float wheel = ImGui::GetIO().MouseWheel;
            if (wheel != 0.0f)
            {
                float zoom_factor = wheel > 0 ? 1.1f : 0.9f;
                view_scale *= zoom_factor;
                view_scale = std::max(0.1f, std::min(view_scale, 10.0f)); // Clamp zoom
            }
        }

        // Helper function to convert world coordinates to screen coordinates
        auto worldToScreen = [&](float world_x, float world_z) -> ImVec2
        {
            float screen_x = canvas_p0.x + view_offset.x + world_x * view_scale;
            float screen_y = canvas_p1.y + view_offset.y - world_z * view_scale; // Invert Y
            return ImVec2(screen_x, screen_y);
        };

        // Auto-follow aircraft if enabled
        if (auto_follow && !paused)
        {
            ImVec2 aircraft_screen = worldToScreen(static_cast<float>(position.x), static_cast<float>(position.y));

            // Define a margin from the edge (in pixels)
            float margin = 100.0f;

            // Check if aircraft is outside visible area with margin
            if (aircraft_screen.x < canvas_p0.x + margin)
            {
                view_offset.x += (canvas_p0.x + margin - aircraft_screen.x);
            }
            else if (aircraft_screen.x > canvas_p1.x - margin)
            {
                view_offset.x -= (aircraft_screen.x - (canvas_p1.x - margin));
            }

            if (aircraft_screen.y < canvas_p0.y + margin)
            {
                view_offset.y += (canvas_p0.y + margin - aircraft_screen.y);
            }
            else if (aircraft_screen.y > canvas_p1.y - margin)
            {
                view_offset.y -= (aircraft_screen.y - (canvas_p1.y - margin));
            }
        }

        // Draw ground line (extends infinitely in view)
        ImVec2 ground_p0 = worldToScreen(-10000.0f, 0.0f);
        ImVec2 ground_p1 = worldToScreen(10000.0f, 0.0f);
        draw_list->AddLine(ground_p0, ground_p1, IM_COL32(100, 200, 100, 255), 2.0f);

        // Draw grid lines for reference
        {
            int grid_spacing = 100; // meters
            ImU32 grid_color = IM_COL32(80, 80, 80, 255);

            // Vertical grid lines
            for (int x = -10000; x <= 10000; x += grid_spacing)
            {
                ImVec2 p0 = worldToScreen(static_cast<float>(x), -1000.0f);
                ImVec2 p1 = worldToScreen(static_cast<float>(x), 10000.0f);
                draw_list->AddLine(p0, p1, grid_color, 1.0f);
            }

            // Horizontal grid lines
            for (int z = 0; z <= 10000; z += grid_spacing)
            {
                ImVec2 p0 = worldToScreen(-10000.0f, static_cast<float>(z));
                ImVec2 p1 = worldToScreen(10000.0f, static_cast<float>(z));
                draw_list->AddLine(p0, p1, grid_color, 1.0f);
            }
        }

        // Draw flight path
        if (flightPath.size() > 1)
        {
            for (size_t i = 0; i < flightPath.size() - 1; i++)
            {
                ImVec2 p1 = worldToScreen(flightPath[i].x, flightPath[i].z);
                ImVec2 p2 = worldToScreen(flightPath[i + 1].x, flightPath[i + 1].z);
                draw_list->AddLine(p1, p2, IM_COL32(255, 255, 0, 255), 2.0f);
            }

            // Draw aircraft position
            if (!flightPath.empty())
            {
                ImVec2 aircraft_pos = worldToScreen(static_cast<float>(position.x), static_cast<float>(position.y));

                draw_list->AddCircleFilled(aircraft_pos, 5.0f, IM_COL32(255, 0, 0, 255));

                // Draw force vectors if enabled
                if (show_vectors)
                {
                    // Helper lambda to draw an arrow
                    auto drawArrow = [&](Vec2 force, ImU32 color, const char *label_text)
                    {
                        if (force.magnitude() > 0.1)
                        { // Only draw if force is significant
                            // Convert force to screen space vector
                            float force_screen_x = static_cast<float>(force.x) * vector_scale;
                            float force_screen_y = -static_cast<float>(force.y) * vector_scale; // Negative because screen Y is inverted

                            ImVec2 end_pos(aircraft_pos.x + force_screen_x, aircraft_pos.y + force_screen_y);

                            // Draw line
                            draw_list->AddLine(aircraft_pos, end_pos, color, 2.0f);

                            // Draw arrowhead
                            Vec2 dir = force.normalized();
                            Vec2 perp(-dir.y, dir.x);
                            float arrow_size = 8.0f;

                            ImVec2 tip = end_pos;
                            ImVec2 p1(end_pos.x - static_cast<float>(dir.x) * arrow_size + static_cast<float>(perp.x) * arrow_size * 0.5f,
                                      end_pos.y + static_cast<float>(dir.y) * arrow_size - static_cast<float>(perp.y) * arrow_size * 0.5f);
                            ImVec2 p2(end_pos.x - static_cast<float>(dir.x) * arrow_size - static_cast<float>(perp.x) * arrow_size * 0.5f,
                                      end_pos.y + static_cast<float>(dir.y) * arrow_size + static_cast<float>(perp.y) * arrow_size * 0.5f);

                            draw_list->AddTriangleFilled(tip, p1, p2, color);

                            // Draw label near the end of the arrow
                            draw_list->AddText(ImVec2(end_pos.x + 5, end_pos.y - 10), color, label_text);
                        }
                    };

                    // Draw each force vector with different colors
                    drawArrow(F_thrust_viz, IM_COL32(0, 255, 0, 255), "Thrust");   // Green
                    drawArrow(F_drag_viz, IM_COL32(255, 128, 0, 255), "Drag");     // Orange
                    drawArrow(F_lift_viz, IM_COL32(0, 255, 255, 255), "Lift");     // Cyan
                    drawArrow(F_weight_viz, IM_COL32(255, 0, 255, 255), "Weight"); // Magenta
                }
            }
        }

        draw_list->PopClipRect();

        ImGui::Text("Controls: Left-click drag to pan, Mouse wheel to zoom");
        ImGui::Text("Zoom: %.2fx | Position: (%.0f, %.0f) m", view_scale, position.x, position.y);
        ImGui::Checkbox("Auto-Follow Aircraft", &auto_follow);
        ImGui::SameLine();
        if (ImGui::Button("Reset View"))
        {
            view_offset = ImVec2(0.0f, 0.0f);
            view_scale = 1.0f;
        }
        ImGui::SameLine();
        if (ImGui::Button("Center on Aircraft"))
        {
            ImVec2 center_screen(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);
            view_offset.x = center_screen.x - canvas_p0.x - static_cast<float>(position.x) * view_scale;
            view_offset.y = center_screen.y - canvas_p1.y + static_cast<float>(position.y) * view_scale;
            auto_follow = true; // Re-enable auto-follow
        }

        ImGui::End();

        // === Instrumentation Panel ===
        ImGui::SetNextWindowPos(ImVec2(420, 520), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(850, 190), ImGuiCond_FirstUseEver);
        ImGui::Begin("Instrumentation");

        // Altitude gauge
        ImGui::BeginGroup();
        ImGui::Text("Altitude");
        ImGui::ProgressBar(static_cast<float>(position.y / 1000.0), ImVec2(0.0f, 0.0f));
        ImGui::Text("%.0f m", position.y);
        ImGui::EndGroup();

        ImGui::SameLine();

        // Speed gauge
        ImGui::BeginGroup();
        ImGui::Text("Airspeed");
        ImGui::ProgressBar(static_cast<float>(velocity.magnitude() / 100.0), ImVec2(0.0f, 0.0f));
        ImGui::Text("%.0f m/s", velocity.magnitude());
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
        double alt_for_atm = std::max(0.0, position.y);
        ImGui::Text("Atmospheric Conditions:");
        ImGui::Text("Temperature: %.1f °C", getTemperature(alt_for_atm) - 273.15);
        ImGui::Text("Pressure:    %.0f Pa", getPressure(alt_for_atm));
        ImGui::Text("Density:     %.3f kg/m³", getDensity(alt_for_atm));
        ImGui::Text("Sound Speed: %.1f m/s", getSpeedOfSound(alt_for_atm));

        ImGui::End();

        // Optional windows
        if (show_demo)
            ImGui::ShowDemoWindow(&show_demo);
        if (show_metrics)
            ImGui::ShowMetricsWindow(&show_metrics);

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
