# Geometric Controller on SE(3) — MATLAB Reproduction

MATLAB reproduction of:

> **Geometric Tracking Control of a Quadrotor UAV on SE(3)**  
> Taeyoung Lee, Melvin Leok, N. Harris McClamroch  
> IEEE Conference on Decision and Control, 2010.

MATLAB R2026b (graphical). Figures are displayed on screen and then saved as PNG/PDF; animations are recorded as MP4.

---

## 1. Research Paper (Summary)

The paper develops a **nonlinear geometric tracking controller** for a quadrotor UAV directly on the special Euclidean group **SE(3)**, using rotation matrices (coordinate-free). This avoids the singularities of Euler-angle representations and the ambiguities/unwinding of quaternions.

Key contributions:
- A global rigid-body dynamics model of the quadrotor on SE(3).
- A tracking controller following a 3D position command and a 1D heading direction using 4 thrust inputs (underactuated: 4 inputs, 6 DOF).
- Almost global exponential attractiveness of the tracking-error zero equilibrium.
- Two numerical examples: (I) elliptical-helix tracking, (II) recovery from an initially upside-down attitude.

---

## 2. Quadrotor Dynamical Model

Coordinates:
- Inertial frame `{i1,i2,i3}`, body-fixed frame `{b1,b2,b3}` (b3 points downward, opposite thrust).
- `R` : body-to-inertial rotation matrix, `R ∈ SO(3)`.
- Total thrust acts along `-b3`, i.e. inertial force = `-f R e3`.

Physical parameters (Section IV):

| Parameter | Value | Unit |
|---|---|---|
| `m` | 4.34 | kg |
| `J` | `diag([0.0820, 0.0845, 0.1377])` | kg·m² |
| `d` | 0.315 | m |
| `c_tau_f` | 8.004e-4 | m |
| `g` | 9.81 | m/s² |

Equations of motion (Eqs. (2)–(5)):

```
x_dot  = v                                            (2)
m v_dot = m g e3 - f R e3                            (3)
R_dot  = R hat(Omega)                                 (4)
J Om_dot + Om x (J Om) = M                            (5)
```

Hat/vee maps: `hat(x) y = x × y`, `vee` is its inverse.

Rotor allocation (Eq. (1)):

```
[f; M1; M2; M3] = [1  1   1   1;
                   0 -d   0   d;
                   d  0  -d   0;
                  -c  c  -c   c] * [f1; f2; f3; f4]
```

Individual rotor thrusts recovered via the inverse allocation matrix (directly controlled thrusts, no rotor dynamics).

---

## 3. Geometric Tracking Controller (Eqs. (6)–(16))

Tracking errors:
```
ex   = x - xd                                          (6)
ev   = v - xd_dot                                      (7)
Psi  = 0.5 * trace(I - Rd' R)                          (8)
eR   = 0.5 * vee(Rd' R - R' Rd)                        (10)
eOm  = Omega - R' Rd Omega_d                           (11)
```

Desired attitude (Eq. (12)):
```
a   = -kx ex - kv ev - m g e3 + m xdd
b3d = -a / ||a||
b2d = cross(b3d,b1d) / ||cross(b3d,b1d)||
Rd  = [cross(b2d,b3d), b2d, b3d]
```

Control inputs (Eqs. (15)–(16)):
```
f  = -a' R e3
M  = -kR eR - kOmega eOm + Omega x (J Omega)
     - J ( hat(Omega) R' Rd Omega_d - R' Rd Omega_dot_d )
```

`Omega_d` and `Omega_dot_d` are obtained analytically from `Rd`, `Rd_dot`, `Rd_ddot`:
```
hat(Omega_d)     = Rd' Rd_dot
hat(Omega_dot_d) = Rd' Rd_ddot - hat(Omega_d)^2
```

Controller gains (Section IV):
```
kx      = 16 m
kv      = 5.6 m
kR      = 8.81
kOmega  = 2.54
```

---

## 4. Numerical Experiments (Section IV)

### Case I — Elliptical helix (0–10 s)

| | Value |
|---|---|
| `x(0)` | [0; 0; 0] |
| `v(0)` | [0; 0; 0] |
| `R(0)` | `eye(3)` |
| `Omega(0)` | [0; 0; 0] |
| `xd(t)` | [0.4 t; 0.4 sin(πt); 0.6 cos(πt)] |
| `b1d(t)` | [cos(πt); sin(πt); 0] |

### Case II — Upside-down recovery (0–6 s)

| | Value |
|---|---|
| `x(0)` | [0; 0; 0] |
| `v(0)` | [0; 0; 0] |
| `Omega(0)` | [0; 0; 0] |
| `R(0)` | [[1,0,0];[0,-0.9995,-0.0314];[0,0.0314,-0.9995]] |
| `xd(t)` | [0; 0; 0] |
| `b1d(t)` | [1; 0; 0] |

Paper-reported observations (Case II): initial attitude error ≈ 178°, initial `Psi` ≈ 1.995, `Psi < 1` at t ≈ 0.88 s, then position and angular velocity errors converge. The simulation checks these without imposing them.

---

## 5. Project Structure

```
GeometricController_MATLAB/
├── main.m
├── README.md
│
├── parameters/
│   ├── quadrotor_parameters.m   % physical parameters (m, J, d, c_tau_f, g)
│   └── controller_parameters.m  % gains kx, kv, kR, kOmega
│
├── dynamics/
│   ├── quadrotor_dynamics.m     % Eqs. (2)-(5) + controller call
│   ├── hat_map.m                % hat : R^3 -> so(3)
│   ├── vee_map.m                % vee : so(3) -> R^3
│   └── rotor_allocation.m       % Eq. (1) + inverse allocation
│
├── controller/
│   ├── geometric_controller.m   % Eqs. (6)-(16)
│   ├── tracking_errors.m        % Eqs. (6)-(8), (10)-(11)
│   ├── desired_attitude.m       % Eq. (12)
│   └── desired_angular_velocity.m % Omega_d, Omega_dot_d
│
├── trajectories/
│   ├── case1_trajectory.m       % elliptical helix
│   └── case2_trajectory.m       % upside-down recovery
│
├── simulations/
│   ├── run_case1.m             % Case I run + data save + validation
│   └── run_case2.m             % Case II run + data save + validation
│
├── visualization/
│   ├── draw_quadrotor.m         % 3D quadrotor graphic primitive
│   ├── plot_case1_results.m     % Figure 4 reproduction (PNG+PDF)
│   ├── plot_case2_results.m     % Figure 5 reproduction (PNG+PDF)
│   ├── animate_case1.m          % Case I MP4 animation
│   └── animate_case2.m          % Case II MP4 animation
│
└── results/
    ├── case1_helix/{plots,videos,data}
    └── case2_inverted_recovery/{plots,videos/data}
```

---

## 6. How to Run

Open MATLAB R2026b, set the current folder to `GeometricController_MATLAB/`, then:

```matlab
main
```

This runs both cases in order, displays each figure graphically, saves PNG/PDF plots, records MP4 animations, and saves MAT data files.

To run a single case:

```matlab
run_case1   % elliptical helix
run_case2   % upside-down recovery
```

---

## 7. Regenerating Plots and Videos

Plots and videos are regenerated automatically when `run_case1` / `run_case2` (or `main`) are executed. Outputs go to:

- `results/case1_helix/plots/`  — PNG + PDF of Figure 4 content
- `results/case1_helix/videos/` — `case1_helix.mp4`
- `results/case1_helix/data/`   — `case1_data.mat`
- `results/case2_inverted_recovery/plots/` — PNG + PDF of Figure 5 content
- `results/case2_inverted_recovery/videos/` — `case2_inverted_recovery.mp4`
- `results/case2_inverted_recovery/data/` — `case2_data.mat`

---

## 8. Numerical Results

**Case I:** The quadrotor tracks the elliptical helix while rotating its heading. Tracking errors converge exponentially to zero (consistent with Proposition 2 since the initial attitude error is small).

**Case II:** Starting upside down (`R(0)` ≈ 178° error, `Psi(0)` ≈ 1.995), the attitude error function decreases and drops below 1 (≈ 0.88 s per the paper), after which position and angular velocity errors converge to zero — demonstrating almost global exponential attractiveness (Proposition 3).

The simulation prints these diagnostics to the MATLAB command window so you can verify them against the paper without imposing the values.

---

## 9. Scope

This project implements **only** the mathematical model, controller, assumptions, parameters, and the two numerical examples presented in the paper. No PID/LQR, no motor dynamics, no sensor models/disturbances, no additional trajectories, no gain optimization, no robustness analysis, and no extensions beyond the paper.
