# Flight Dynamics Simulator

A C++ flight dynamics simulation with both command-line and GUI interfaces. Features atmospheric modeling, aerodynamics calculations, numerical integration, and PID control systems.

## Features

- **Atmospheric Modeling**: ISA (International Standard Atmosphere) calculations for temperature, pressure, and density at various altitudes
- **Aerodynamics**: Lift and drag calculations using standard aerodynamic equations
- **Numerical Integration**: Multiple integration methods (Euler, RK2, RK4) for solving differential equations
- **PID Controller**: Proportional-Integral-Derivative controller with anti-windup and output limiting
- **GUI Application**: Interactive interface built with Dear ImGui and SDL3
- **Comprehensive Testing**: Full test suite using Catch2 framework

## Project Structure

```
CppLearning/
├── src/                    # Source code
│   ├── atmosphere.*        # Atmospheric calculations
│   ├── aero.*              # Aerodynamics models
│   ├── integrator.*        # Numerical integration methods
│   ├── pid.*               # PID controller
│   ├── vec2.hpp            # 2D vector utilities
│   ├── main.cpp            # Command-line application
│   └── main_gui.cpp        # GUI application
├── tests/                  # Unit tests
│   ├── atmos_tests.cpp
│   ├── aero_tests.cpp
│   ├── integrator_tests.cpp
│   └── pid_tests.cpp
├── external/               # Git submodules (not committed)
│   ├── imgui/              # Dear ImGui library
│   └── SDL3/               # SDL3 library
└── .github/workflows/      # CI/CD workflows
```

## Requirements

- **Windows 10 or later**
- **CMake 3.15+**
- **Visual Studio 2022** (or MSBuild tools)
- **Git** (for cloning with submodules)

## Quick Start

### 1. Clone the Repository

```powershell
git clone --recursive https://github.com/MagicIshSeal/CppLearning.git
cd CppLearning
```

If you already cloned without `--recursive`, initialize submodules:

```powershell
git submodule update --init --recursive
```

### 2. Build the Project

```powershell
# Configure
cmake -B build

# Build (Debug)
cmake --build build

# Build (Release)
cmake --build build --config Release
```

### 3. Run the Applications

```powershell
# Command-line version
.\build\Debug\FlightDynamics.exe

# GUI version
.\build\Debug\FlightDynamicsGUI.exe
```

## Building & Testing

### Build Commands

```powershell
# Full build with tests
cmake --build build

# Build specific target
cmake --build build --target FlightDynamicsGUI

# Clean build
cmake --build build --target clean

# Rebuild from scratch
Remove-Item -Recurse -Force build
cmake -B build
cmake --build build
```

### Running Tests

All tests run automatically after building. To run manually:

```powershell
# Run all tests
cd build
ctest -C Debug --output-on-failure

# Or run individual test executables
.\Debug\atmos_tests.exe
.\Debug\aero_tests.exe
.\Debug\integrator_tests.exe
.\Debug\pid_tests.exe
```

**Test Coverage:**

- **Atmosphere Tests**: 5 assertions in 4 test cases
- **Aero Tests**: 6 assertions in 6 test cases
- **Integrator Tests**: 50 assertions in 12 test cases
- **PID Tests**: 243 assertions in 10 test cases

## Creating Releases

### Quick Local Release

Build and package for distribution:

```powershell
cd build
cmake --build . --config Release --target FlightDynamicsGUI

# Create release package
New-Item -ItemType Directory -Force -Path "FlightDynamicsGUI-Release"
Copy-Item "Release/FlightDynamicsGUI.exe" -Destination "FlightDynamicsGUI-Release/"
Copy-Item "Release/SDL3.dll" -Destination "FlightDynamicsGUI-Release/"
Compress-Archive -Path "FlightDynamicsGUI-Release/*" -DestinationPath "FlightDynamicsGUI-Windows-x64.zip" -Force
```

### GitHub Releases

#### Automated Release (Recommended)

Push a version tag to trigger automatic build and release:

```powershell
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0
```

The GitHub Actions workflow will automatically:

- Build the project in Release mode
- Package the executable and dependencies
- Create a GitHub release with the ZIP file

#### Manual Release

1. Go to https://github.com/MagicIshSeal/CppLearning/releases/new
2. Create a new tag (e.g., `v1.0.0`)
3. Upload `FlightDynamicsGUI-Windows-x64.zip`
4. Add release notes
5. Click "Publish release"

### Version Numbering

Follow semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR** (1.0.0): Breaking changes or major features
- **MINOR** (1.1.0): New features, backwards compatible
- **PATCH** (1.0.1): Bug fixes only

## Git Submodules

This project uses git submodules for external dependencies:

### Current Setup

- **ImGui** (https://github.com/ocornut/imgui.git) - Immediate mode GUI library
- **SDL3** (https://github.com/libsdl-org/SDL.git) - Cross-platform multimedia library

### Working with Submodules

```powershell
# Clone with submodules
git clone --recursive https://github.com/MagicIshSeal/CppLearning.git

# Initialize submodules (if not done during clone)
git submodule update --init --recursive

# Update submodules to latest
git submodule update --remote --merge

# Check submodule status
git submodule status
```

### If Submodules Are Missing

If you encounter errors about missing `external/` directories:

```powershell
# Remove any existing external directories
Remove-Item -Recurse -Force external -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force .git/modules/external -ErrorAction SilentlyContinue

# Re-add submodules
git submodule add https://github.com/ocornut/imgui.git external/imgui
git submodule add https://github.com/libsdl-org/SDL.git external/SDL3
git submodule update --init --recursive
```

## Development

### Code Organization

- **`atmosphere.*`**: Standard atmosphere calculations (ISA model)
- **`aero.*`**: Aerodynamic force calculations (lift, drag)
- **`integrator.*`**: Numerical integration (Euler, RK2, RK4)
- **`pid.*`**: PID controller with configurable gains and limits
- **`vec2.hpp`**: 2D vector math utilities

### Adding New Features

1. Implement functionality in appropriate module (`src/`)
2. Add unit tests (`tests/`)
3. Update CMakeLists.txt if adding new files
4. Run tests to verify: `cmake --build build`

### Testing

Tests use the Catch2 framework. Example test structure:

```cpp
#include "catch_amalgamated.hpp"
#include "your_module.hpp"

TEST_CASE("Feature description", "[module]") {
    // Arrange
    YourClass obj;

    // Act
    auto result = obj.method();

    // Assert
    REQUIRE(result == expected);
}
```

## Executables

After building, you'll find these in `build/Debug/` or `build/Release/`:

- **FlightDynamics.exe** - Command-line application
- **FlightDynamicsGUI.exe** - GUI application (requires SDL3.dll)
- **atmos_tests.exe** - Atmosphere tests
- **aero_tests.exe** - Aerodynamics tests
- **integrator_tests.exe** - Integration tests
- **pid_tests.exe** - PID controller tests

## Troubleshooting

### Build Issues

**Problem**: CMake can't find submodules

```powershell
git submodule update --init --recursive
```

**Problem**: Tests fail with "Not Run" error

```powershell
# Tests need configuration specified on Windows
ctest -C Debug --output-on-failure
```

**Problem**: SDL3.dll not found when running GUI

```powershell
# Make sure you're in the build/Debug or build/Release directory
cd build\Debug
.\FlightDynamicsGUI.exe
```

### Submodule Issues

**Problem**: `fatal: No url found for submodule path`

This occurs when switching branches. Fix:

```powershell
Remove-Item -Recurse -Force external -ErrorAction SilentlyContinue
Remove-Item -Recurse -Force .git/modules/external -ErrorAction SilentlyContinue
git submodule add https://github.com/ocornut/imgui.git external/imgui
git submodule add https://github.com/libsdl-org/SDL.git external/SDL3
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes and add tests
4. Ensure all tests pass (`ctest -C Debug`)
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## License

This project is available for educational purposes.

## Acknowledgments

- **Dear ImGui** - Omar Cornut and contributors
- **SDL3** - Sam Lantinga and the SDL team
- **Catch2** - Phil Nash and contributors

---

For more detailed information, see:

- Build instructions: This README covers all build steps
- Release process: See "Creating Releases" section above
- Submodule setup: See "Git Submodules" section above
