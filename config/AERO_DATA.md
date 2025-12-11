# Aerodynamic Data CSV Format

The simulation supports loading aerodynamic coefficients from CSV files instead of using simplified linear/parabolic models.

## CSV Format

The CSV file should contain three columns with a header row:

```csv
alpha,CL,CD
-10,-0.45,0.080
-8,-0.30,0.065
-6,-0.15,0.055
-4,0.00,0.048
-2,0.20,0.045
0,0.40,0.044
2,0.60,0.045
4,0.80,0.048
6,1.00,0.053
8,1.18,0.060
10,1.34,0.070
12,1.46,0.083
14,1.52,0.100
16,1.50,0.125
18,1.42,0.160
20,1.28,0.210
```

### Columns

- **alpha**: Angle of attack in degrees
- **CL**: Lift coefficient (dimensionless)
- **CD**: Drag coefficient component (dimensionless)

### Important Notes

1. **Alpha is automatically converted** from degrees to radians internally
2. **Data is sorted** by alpha for interpolation
3. **CD values are components** - the total drag is `CD_total = CD0 + CD_table(alpha)` where CD0 is the parasitic drag from the aircraft config
4. **Linear interpolation** is used between data points
5. **Linear extrapolation** is used beyond the data range:
   - Below minimum alpha: extends using slope from first two points
   - Above maximum alpha: extends using slope from last two points
6. **CL is clamped** to a minimum of 0 when extrapolating beyond known data
7. **CD is clamped** to edge values when extrapolating (no extrapolation for drag)

## Using CSV Data in Aircraft Configs

Add an `aeroDataFile` field to your JSON configuration:

```json
{
  "mass": 120.0,
  "S": 1.6,
  "CL_alpha": 5.7,
  "CD0": 0.025,
  "k": 0.04,
  "maxThrust": 500.0,
  "aeroDataFile": "aero_default.csv"
}
```

- **aeroDataFile**: Path to CSV file (relative to config directory)
- **CD0**: Parasitic drag coefficient (added to table CD values)
- **CL_alpha, k**: Legacy parameters (used if CSV loading fails)

## Behavior

### With CSV Data Loaded

- **CL**: Read directly from table with interpolation/extrapolation
- **CD**: `CD = CD0 + CD_table(alpha)`
- Realistic post-stall behavior from table data

### Fallback (No CSV or Load Failure)

- **CL**: Linear model `CL = CL_alpha * alpha`
- **CD**: Parabolic polar `CD = CD0 + k * CL²`
- Simplified analytical model

## Example Data Sources

Real airfoil data can be obtained from:

- **XFOIL**: Generate polar data for any airfoil geometry
- **Airfoil Tools** (airfoiltools.com): Pre-computed polar databases
- **UIUC Airfoil Database**: Experimental wind tunnel data
- **CFD Results**: Computational fluid dynamics simulations
- **Flight Test Data**: Measured data from actual aircraft

## Sample Data

The included `aero_default.csv` contains typical data for a small general aviation airfoil:

- **Operating Range**: -10° to +20°
- **Stall**: ~14° (CL_max ≈ 1.52)
- **Post-Stall**: Gradual lift reduction with increased drag
- **Zero-Lift Angle**: ~-4°
- **Minimum Drag**: Around 0° AoA

This data represents a cambered airfoil suitable for general aviation aircraft, with realistic stall characteristics and drag polar.

## Flight Physics Integration

The elevator control system works as follows:

1. **Elevator Input** (-1 to +1): Pilot/autopilot command
2. **Pitch Rate**: Elevator → pitch angular velocity (°/s)
3. **Pitch Angle**: Integrated from pitch rate
4. **Flight Path Angle**: Direction of velocity vector
5. **Angle of Attack**: `AoA = pitch - flight_path_angle`
6. **Aerodynamic Coefficients**: CL and CD looked up from table at AoA
7. **Forces**: Lift and drag calculated from coefficients and dynamic pressure
8. **Acceleration**: Net forces / mass → velocity change

This provides realistic flight dynamics where control inputs affect pitch rate, which naturally changes angle of attack based on the flight path.
