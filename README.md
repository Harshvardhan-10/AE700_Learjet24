# AE 700 — Guidance and Control of Unmanned Autonomous Vehicles
### Project: 6-DOF Simulation and Autopilot Design — Learjet 24
**Indian Institute of Technology Bombay | Department of Aerospace Engineering**

---

## Team

| Name | Roll Number |
|---|---|
| Harshvardhan Shrivastav | 23B1525 |
| Ayush Lonakadi | 23B2253 |
| Manyaman Naik | 23B2145 |
| Sanat Jain | 23B2463 |

**Instructor:** Prof. Shashi Ranjan Kumar

---

## Overview

This project implements a complete fixed-wing UAV simulation pipeline for the **Learjet 24** aircraft, following the framework of Beard & McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012.

The pipeline covers five sections:

1. **3-D Animation** — Coordinate frames, vertices-and-faces aircraft body, rotation/translation verification
2. **6-DOF Equations of Motion** — Euler-angle S-function, gyroscopic coupling test
3. **Forces and Moments** — Aerodynamic, gravitational, and propulsion forces in body frame
4. **Linear Design Models** — Trim computation, state-space linearisation, transfer functions
5. **Autopilot Design** — Successive loop closure: roll, course, pitch, altitude, airspeed controllers

---

## Requirements

- MATLAB R2020b or later
- Simulink
- Control System Toolbox (for `tf`, `feedback`, `lsim` in Chapters 5–6)

---

## Directory Structure

```
├── parameters/          # Aircraft, simulation, wind, sensor parameters
├── tools/               # Rotation matrix utilities (Euler, Quaternion)
├── chap2/               # Section 1: 3-D animation and coordinate frames
├── chap3/               # Section 2: 6-DOF equations of motion
├── chap4/               # Section 3: Forces and moments
├── chap5/               # Section 4: Trim, state-space, transfer functions
└── chap6/               # Section 5: Autopilot design and simulation
```

---

## How to Run

### Step 0 — Set MATLAB path

Open MATLAB, navigate to the project root, and run:

```matlab
addpath('parameters')
addpath('tools')
```

---

### Section 1: 3-D Animation (`chap2/`)

Open and run the Simulink model:

```
chap2/mavsim_chap2.slx
```

The aircraft body is defined in `drawAircraft.m`. The model accepts position `(pn, pe, pd)` and attitude `(phi, theta, psi)` as inputs via slider gains.

**Rotation vs Translation order test:**
- Run with default settings → correct behaviour (rotate about CG, then translate)
- Swap `rotate()` and `translate()` calls in `drawAircraft.m` → incorrect behaviour (aircraft orbits world origin)

Reference images: `Figure_1_normal.png` (correct), `Figure_1_rev.png` (incorrect).

---

### Section 2: Kinematics and Dynamics (`chap3/`)

Open the Simulink model:

```
chap3/mavsim_chap3.slx
```

The equations of motion are in `mav_dynamics.m` (Level-1 S-function, 12 Euler states).

**EOM verification script:**

```matlab
cd chap3
verify_EOM.m          % applies one force/moment per axis, saves eom_verification.png
```

**Jxz gyroscopic coupling test:**

```matlab
cd chap3
verify_Jxz_coupling.m % compares Jxz=0 vs Jxz≠0, saves jxz_coupling_verification.png
```

---

### Section 3: Forces and Moments (`chap4/`)

Open the Simulink model:

```
chap4/mavsim_chap4.slx
```

Aerodynamic, gravitational, and propulsion forces are computed in `forces_moments.m`.

**Verification script:**

```matlab
cd chap4
fm_verification.m     % applies delta_e, delta_a, delta_r individually, saves fm_verification.png
```

---

### Section 4: Linear Design Models (`chap5/`)

Run the following scripts **in order** from `chap5/`:

```matlab
cd chap5

compute_trim.m          % requires mavsim_trim.slx open or on path
                        % saves: trim_results.mat, trim_results_turn.mat

compute_ss_model.m      % requires mavsim_trim.slx
                        % saves: ss_models.mat

compute_tf_model.m      % saves: transfer_function_coef.mat
```

**What each produces:**
- `trim_results.mat` — trim state and controls at Va=80 m/s, γ=0 (alpha ≈ 5.5°)
- `trim_results_turn.mat` — coordinated turn trim at n=1.2 (phi ≈ 33.9°, CL ≈ 0.83)
- `ss_models.mat` — lateral A,B and longitudinal A,B matrices
- `transfer_function_coef.mat` — a_phi1/2, a_theta1/2/3, a_V1/2/3, etc.

> **Note:** `mavsim_trim.slx` must be on the MATLAB path when running `compute_trim.m` and `compute_ss_model.m` as both call `trim()` and `linmod()` internally.

---

### Section 5: Autopilot Design (`chap6/`)

Run the following scripts **in order** from `chap6/`:

```matlab
cd chap6

% Step 1: Compute autopilot gains (requires transfer_function_coef.mat)
compute_autopilot_gains.m    % saves: autopilot_gains.mat

% Step 2: Run nonlinear closed-loop simulation
% Calls autopilot.m -> forces_moments.m -> mav_dynamics.m at each timestep
run_autopilot_sim.m          % saves: autopilot_sim_results.mat

% Step 3: Generate all 10 report figures
generate_report_plots.m      % saves: fig1 through fig10 as PNG
```

**To run the full Simulink autopilot model:**

```
chap6/mavsim_chap6.slx
```

Set wind to zero by setting `WIND.sigma_u = WIND.sigma_v = WIND.sigma_w = 0` in `parameters/wind_parameters.m` before running.

---

## Full Pipeline (complete run from scratch)

```matlab
% From project root:
addpath('parameters'); addpath('tools')

cd chap3;  verify_EOM;           cd ..
cd chap3;  verify_Jxz_coupling;  cd ..
cd chap4;  fm_verification;      cd ..
cd chap5;  compute_trim;         cd ..
cd chap5;  compute_ss_model;     cd ..
cd chap5;  compute_tf_model;     cd ..
cd chap6;  compute_autopilot_gains;  cd ..
cd chap6;  run_autopilot_sim;        cd ..
cd chap6;  generate_report_plots;    cd ..
```

---

## Key Design Parameters

| Parameter | Value | Notes |
|---|---|---|
| Trim airspeed | 80 m/s | Gives α ≈ 5.5° (required 5–7°) |
| Surface deflection limit | ±40° | Per project specification |
| Roll bandwidth ωn,φ | 2.85 rad/s | Analytical |
| Pitch bandwidth ωn,θ | 4.17 rad/s | Analytical |
| Course bandwidth ωn,χ | 0.078 rad/s | Manually tuned |
| Altitude/airspeed bandwidth | 0.417 rad/s | ωn,θ / 10 |

---

## Notes

- The course loop gains (`kp=0.5`, `ki=0.05`) are manually tuned. The analytically derived B&M values produce actuator saturation at the speed we chose for Learjet24.
- The yaw damper gain `kp = -0.5` is negative because the formula in `autopilot.m` applies an additional negative sign: `delta_r = sat(-kp * r_washed, ...)`. The net effect is positive damping.
- All autopilot plots are generated from the pipeline (`run_autopilot_sim.m`), the same variations can be seen via running chap6 simulink model too.

---

## References

1. R. W. Beard & T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012.
2. J. Roskam, *Airplane Flight Dynamics and Automatic Flight Controls*, Part I, DARcorporation, 1995, pp. 522–524.
