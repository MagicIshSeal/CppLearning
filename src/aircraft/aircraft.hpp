#pragma once

#include <string>
#include <memory>

// Forward declaration
class AeroDataTable;

// Aircraft class representing a fixed-wing aircraft with its physical and aerodynamic properties
class Aircraft
{
public:
    // Physical properties
    double mass; // Mass in kg
    double S;    // Wing area in m²

    // Aerodynamic properties (legacy - used if no aero table)
    double CL_alpha; // Lift curve slope [1/rad]
    double CD0;      // Parasitic drag coefficient
    double k;        // Induced drag factor

    // Propulsion
    double maxThrust; // Maximum thrust in N

    // Aerodynamic table data (optional, overrides legacy params if present)
    std::shared_ptr<AeroDataTable> aeroTable;
    std::string aeroDataFile; // Path to CSV file

    // Default constructor with typical ultralight aircraft values
    Aircraft()
        : mass(120.0), S(1.60), CL_alpha(5.7), CD0(0.025), k(0.04), maxThrust(500.0),
          aeroTable(nullptr), aeroDataFile("")
    {
    }

    // Constructor with custom values
    Aircraft(double mass_, double S_, double CL_alpha_, double CD0_, double k_, double maxThrust_)
        : mass(mass_), S(S_), CL_alpha(CL_alpha_), CD0(CD0_), k(k_), maxThrust(maxThrust_),
          aeroTable(nullptr), aeroDataFile("")
    {
    }

    // Check if using table data
    bool hasAeroTable() const { return aeroTable != nullptr; }
};
