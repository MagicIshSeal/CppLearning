# Setting Up Git Submodules for External Dependencies

## Why Use Git Submodules?

Git submodules allow you to keep external libraries as separate repositories while tracking specific versions. Benefits:
- Track exact versions of dependencies
- Easy updates with `git submodule update`
- Cleaner repository (no need to commit large external libraries)
- Easy to see what version of each library you're using

## Current Setup

The `external/` folder currently contains:
- `SDL2/` - Actually SDL3 prebuilt binaries
- `imgui/` - Dear ImGui source code

## How to Convert to Submodules

### 1. Remove current external directories (after backing up)

```powershell
# First, make sure your changes are committed
cd "C:\Users\mvane\Documents\Git Clone\CppLearning"
git add .
git commit -m "Save current work before switching to submodules"

# Back up the external folder just in case
Copy-Item -Recurse FlightDynamics/external FlightDynamics/external_backup

# Remove from git (but keep locally for now)
git rm -r FlightDynamics/external/imgui
git rm -r FlightDynamics/external/SDL2
```

### 2. Add as submodules

```powershell
cd FlightDynamics

# Add Dear ImGui (official repository)
git submodule add https://github.com/ocornut/imgui.git external/imgui
git submodule add https://github.com/libsdl-org/SDL.git external/SDL3

# Initialize and update
git submodule update --init --recursive
```

### 3. Check out specific versions

```powershell
# For ImGui - check out v1.91.5 (the version you're currently using)
cd external/imgui
git checkout v1.91.5
cd ../..

# For SDL3 - check out latest stable release
cd external/SDL3
git checkout release-3.2.0  # or whatever version you want
cd ../..
```

### 4. Update CMakeLists.txt

You'll need to update the CMakeLists.txt to:
- Build SDL3 from source (or download prebuilt binaries)
- Point to the correct ImGui source files

### 5. Commit the submodule setup

```powershell
git add .gitmodules
git add FlightDynamics/external/imgui
git add FlightDynamics/external/SDL3
git commit -m "Convert external dependencies to git submodules"
```

## Using Submodules

### Clone repository with submodules
```powershell
git clone --recursive https://github.com/MagicIshSeal/CppLearning.git
```

### Update submodules to latest
```powershell
git submodule update --remote --merge
```

### Update to specific version
```powershell
cd FlightDynamics/external/imgui
git checkout v1.92.0  # new version
cd ../../..
git add FlightDynamics/external/imgui
git commit -m "Update ImGui to v1.92.0"
```

## Alternative: Use Package Managers

Instead of submodules, you could also use:
- **vcpkg** (Microsoft's C++ package manager)
- **Conan** (C/C++ package manager)

### Example with vcpkg:
```powershell
# Install vcpkg
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Install dependencies
.\vcpkg install sdl3 imgui

# Integrate with Visual Studio/CMake
.\vcpkg integrate install
```

Then update CMakeLists.txt to use `find_package(SDL3 CONFIG REQUIRED)` and `find_package(imgui CONFIG REQUIRED)`.

## Recommendation

For this project, I'd recommend:
1. **Keep SDL3 as prebuilt binaries** (what you have now) - SDL is complex to build
2. **Use ImGui as a submodule** - it's header-only and easy to integrate
3. Or use **vcpkg** for both - easiest long-term maintenance

Let me know which approach you prefer!
