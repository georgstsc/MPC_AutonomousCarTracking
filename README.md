# MPC Autonomous Car Tracking

**Model Predictive Control for Highway Driving**

A comprehensive implementation of Model Predictive Control (MPC) systems for autonomous vehicle control in highway scenarios, developed as part of the ME-425 Model Predictive Control course at EPFL.

## 👥 Team AW

- **Georg Schwabedal** (328434)
- **Gautier Demierre** (340523)
- **Benjamin Bahurel** (326888)

*December 2024*

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Implementation Details](#implementation-details)
- [Usage](#usage)
- [Results](#results)
- [Requirements](#requirements)
- [Project Structure](#project-structure)
- [Acknowledgments](#acknowledgments)

---

## 🎯 Overview

This project implements an advanced Model Predictive Control system for autonomous vehicles to handle real-world highway driving scenarios. The system ensures smooth and safe driving through:

1. **System Linearization**: Simplifying nonlinear dynamics into longitudinal (speed) and lateral (steering) subsystems
2. **Offset-Free Tracking**: Precise control accounting for external disturbances like drag
3. **Robust Tube-MPC**: Safe distance maintenance with unpredictable lead vehicle behavior
4. **Nonlinear MPC**: Full-system dynamics for advanced maneuvers (lane changes, overtaking)

All implementations are developed in **MATLAB**.

---

## ✨ Features

### 1. **Linearized MPC Controllers** (Part 2-3)
- Decoupled longitudinal and lateral subsystem control
- Terminal invariant set computation for stability
- Velocity tracking: 80 km/h → 120 km/h in < 10s
- Lane change execution in < 3s

### 2. **Offset-Free Tracking** (Part 4)
- Disturbance estimator using pole placement
- Augmented state-space model with constant disturbance
- Eliminates steady-state error from linearization

### 3. **Robust Tube-MPC** (Part 5)
- Adaptive cruise control with safety guarantees
- Minimum Robust Invariant Set (mRIS) computation
- Tightened constraints using Pontryagin Difference
- Safe distance maintenance: 8m nominal, 6m minimum

### 4. **Nonlinear MPC** (Part 6)
- Full nonlinear system dynamics (RK4 integration)
- Lane change and overtaking maneuvers
- Ellipsoidal collision avoidance constraints
- Real-time trajectory optimization

---

## 🏗️ System Architecture

### Linearization

The system is linearized around steady-state operating point `xs = [0, 0, 0, Vs]'` with input `us = [0, uT,s]'`:

```
ẋ = f(x,u) ≈ f(xs,us) + A(x-xs) + B(u-us)
```

**Decoupled Subsystems:**
- **Longitudinal**: `[x, V]` - position and velocity
- **Lateral**: `[y, θ]` - lateral position and heading angle

### State Vector
```matlab
x = [x, y, θ, V]'  % [position_x, position_y, heading, velocity]
```

### Input Vector
```matlab
u = [δ, uT]'       % [steering_angle, throttle]
```

---

## 🔧 Implementation Details

### Part 3: MPC Controller Design

#### Longitudinal Controller (`MpcControl_lon.m`)
- **Objective**: Velocity tracking without steady-state error
- **Cost Matrices**: `Q = diag([0, 25])`, `R = 0.2`
- **Prediction Horizon**: Variable (tuned for performance)
- **Constraints**: Input saturation only

#### Lateral Controller (`MpcControl_lat.m`)
- **Objective**: Lane change in < 3s
- **Cost Matrices**: `Q = diag([3, 3])`, `R = 4`
- **Prediction Horizon**: `H = 2s`
- **Terminal Set**: Computed via iterative LQR pre-sets
- **Constraints**: State and input limits for passenger comfort

### Part 4: Offset-Free Tracking

**Augmented System:**
```matlab
A_aug = [Ad(2,2), Bd(2); 0, 1]
B_aug = [Bd(2); 0]
C_aug = [Cd(2,2), 0]
```

**Estimator Design:**
- Pole placement at `[0.6, 0.7]` for fast convergence
- Delta-form state estimation: `Δz_next = A*Δz + B*Δu + L*(Δy - C*Δz)`

### Part 5: Robust Tube-MPC

**Key Components:**

1. **Minimum Robust Invariant Set (ϵ)**
   - LQR controller: `Q = diag([20, 54.5])`, `R = 1.5`
   - Disturbance: `±0.5` on throttle input
   - Computed via iterative algorithm with `.minHRep()` optimization

2. **Tightened Constraints**
   ```matlab
   X_tilde = X ⊖ ϵ  % Pontryagin Difference
   U_tilde = U ⊖ K*ϵ
   ```

3. **Terminal Components**
   - Terminal controller: LQR with `Qf = Q/2`, `Rf = R*2`
   - Terminal set: Maximal invariant set via preset iteration

4. **Safety Parameters**
   - Safe distance: 8m (nominal), 6m (minimum)
   - Relative state tracking: `Δ = x̃_long - x_long - x_safe`

### Part 6: Nonlinear MPC

**Cost Function:**
```matlab
cost = 10*(error_speed)^2 + 10*(error_y)^2 + 10*(steering)^2
```

**Integration**: RK4 (Runge-Kutta 4th order) for exact dynamics

**Overtaking Constraint** (Deliverable 6.2):
- Ellipsoidal safety region: `(p - p_L)^T * H * (p - p_L) ≥ 1`
- Safety matrix: `H = diag([1/64, 1/9])` → 8m longitudinal, 3m lateral clearance
- Prediction horizon: `H = 2s`

---

## 🚀 Usage

### Running Deliverables

Each deliverable can be executed independently:

```matlab
% Deliverable 3: Basic MPC Controllers
Deliverable5.m

% Deliverable 4: Offset-Free Tracking
% (Run modified version with estimator)

% Deliverable 5.1: Robust Tube-MPC
% (Configure lead car scenarios)

% Deliverable 6: Nonlinear MPC
% (Lane change and overtaking)
```

### Visualizing Results

Plots are automatically generated and saved in the `plots/` directory:

```matlab
% Display plots
plot_display.m
```

### Computing Tube-MPC Sets

```matlab
% Generate minimal invariant and terminal sets
tube_mpc_sets.m
```

---

## 📊 Results

### Deliverable 3: Linear MPC
- ✅ Velocity: 80 → 120 km/h in ~8s
- ✅ Lane change: Completed in ~2.5s
- ✅ Smooth trajectories with constraint satisfaction

### Deliverable 4: Offset-Free Tracking
- ✅ Zero steady-state error at all velocities
- ✅ Accurate disturbance estimation and compensation

### Deliverable 5: Robust Tube-MPC
- ✅ Safe distance maintained: 6-8m range
- ✅ Robust to ±0.5 throttle disturbances
- ✅ Smooth velocity adaptation to lead car

### Deliverable 6: Nonlinear MPC
- ✅ Precise 100 km/h tracking with RK4 integration
- ✅ Safe overtaking with 3m lateral, 8m longitudinal clearance
- ✅ Computational efficiency with H=2s horizon

---

## 📦 Requirements

### Software
- **MATLAB** R2020a or later
- **Optimization Toolbox**
- **Control System Toolbox**
- **MPT3 Toolbox** (Multi-Parametric Toolbox) for polytope operations

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/georgstsc/MPC_AutonomousCarTracking.git
   cd MPC_AutonomousCarTracking
   ```

2. Install MPT3:
   ```matlab
   % In MATLAB:
   % Download from: https://www.mpt3.org/
   % Follow installation instructions
   ```

3. Run main deliverable files:
   ```matlab
   Deliverable5.m
   ```

---

## 📁 Project Structure

```
.
├── Deliverable5.m              # Main simulation script
├── MpcControl_lon.m            # Longitudinal MPC controller
├── MpcControl_lat.m            # Lateral MPC controller
├── plot_display.m              # Visualization utilities
├── tube_mpc_sets.m             # Robust set computations
├── tube_mpc_variables.mat      # Pre-computed sets and parameters
├── Minimal.fig                 # MATLAB figure file
├── plots/                      # Generated result plots
│   ├── Minimal_set5.1.png
│   ├── Terminal_set5.1.png
│   ├── position_heading.png
│   ├── velocity.png
│   ├── throttle.png
│   ├── steering.png
│   └── ...
└── README.md                   # This file
```

---

## 🎓 Key Concepts Demonstrated

1. **System Linearization**: Analytical derivation of state-space matrices
2. **LQR Control**: Optimal feedback gain computation
3. **Terminal Sets**: Invariant set computation for stability
4. **State Estimation**: Luenberger observer with pole placement
5. **Robust Control**: Tube-MPC with disturbance rejection
6. **Nonlinear Optimization**: Direct multiple shooting with CasADi/YALMIP
7. **Collision Avoidance**: Ellipsoidal constraint formulation

---

## 🏆 Achievements

- ✅ **Full system decomposition** into manageable subsystems
- ✅ **Offset-free tracking** with disturbance estimation
- ✅ **Guaranteed safety** via tube-MPC with formal verification
- ✅ **Real-time capable** nonlinear MPC for complex maneuvers
- ✅ **Comprehensive testing** across multiple scenarios

---

## 🙏 Acknowledgments

Special thanks to:
- **Professor Colin Jones** for course guidance and foundational materials
- **ME-425 Teaching Assistants** for invaluable support throughout the project
- EPFL Model Predictive Control course materials and resources

---

## 📚 References

- Course: ME-425 Model Predictive Control, EPFL
- Textbook: *Model Predictive Control* - Rawlings, Mayne, Diehl
- MPT3: Multi-Parametric Toolbox 3.0
- Exercise series and lecture slides (available on Moodle)

---

## 📄 License

This project is part of academic coursework at EPFL. All rights reserved by the authors.

---

## 📧 Contact

For questions or collaboration:
- Georg Schwabedal: [GitHub Profile](https://github.com/georgstsc)

---

**Course**: ME-425 Model Predictive Control  
**Institution**: École Polytechnique Fédérale de Lausanne (EPFL)  
**Semester**: Fall 2024
