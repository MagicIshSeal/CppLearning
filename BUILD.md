# FlightDynamics Build Instructions

## CMake Build Commands

### Initial Configuration

```powershell
cmake -B build -G "NMake Makefiles"
```

### Build Everything

```powershell
cmake --build build
```

### Run All Tests

```powershell
cmake --build build --target run_tests
```

### Run Tests with CTest

```powershell
cd build
ctest --output-on-failure
```

### Clean Build

```powershell
cmake --build build --target clean
```

### Rebuild from Scratch

```powershell
Remove-Item -Recurse -Force build
cmake -B build -G "NMake Makefiles"
cmake --build build
```

## Executables

After building, you'll find the following executables in the `build/` directory:

- `FlightDynamics.exe` - Main application
- `atmos_tests.exe` - Atmosphere tests
- `aero_tests.exe` - Aerodynamics tests
- `integrator_tests.exe` - Integration tests
