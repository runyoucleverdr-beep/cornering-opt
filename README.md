# cornering-opt

Single-level minimum-time cornering optimization with a three-point parameterized racing line.

## Overview

This project studies how the optimal racing line changes under different entry speeds.

Instead of treating the racing line as fixed, the cornering problem is formulated as a **single-level constrained nonlinear optimization problem** in which both:

- the path geometry, and
- the discretized speed profile

are optimized simultaneously.

The current implementation uses:

- a **three-point racing-line parameterization**
  - `turn-in point`
  - `apex point`
  - `exit-opening point`
- **squared speed variables** `q_i = v_i^2`
- a **minimum-time objective**
- nonlinear constraints for:
  - path ordering
  - track width
  - curvature-based friction
  - acceleration
  - braking
  - entry-speed equality
  - variable bounds

The solver is based on **SciPy `trust-constr`**, using an interior-point / trust-region style constrained optimization method.

---

## Current Project Status

The current baseline uses a **Baku Turn 15-inspired segment** and now converges to a **strictly feasible solution**.

### Baseline status

- `Success: True`
- termination by `gtol`
- zero violation in:
  - ordering
  - track width
  - friction
  - acceleration
  - braking
  - entry speed
  - variable bounds

This means the optimization pipeline is working end-to-end and produces a numerically converged feasible baseline solution.

---

## Current Main Finding

Under the current model and current Baku Turn 15-inspired approximation:

- the model is reliably feasible for entry speeds roughly in the range `17-23 m/s`
- infeasibility starts to appear at higher entry speeds such as `24 m/s` and above
- within the successful range, the optimal geometric racing line changes very little
- the main response to higher entry speed is **braking adaptation**, not major geometric line restructuring

In plain English:

> the line stays almost the same, while the braking profile changes.

---

## Problem Formulation

### Decision Variables

The optimization variable is:

`x = (s_turn, s_apex, s_exit, q_1, ..., q_N)`

where:

- `s_turn`: turn-in location
- `s_apex`: apex location
- `s_exit`: exit-opening location
- `q_i = v_i^2`: squared speed at discretization point `i`

### Objective

Minimize total traversal time through the corner segment:

`T(x) ~= sum_i [ Delta_s / sqrt(q_i) ]`

where `Delta_s` is the mean path discretization step.

### Constraints

The current model includes the following constraints.

#### 1. Ordering constraint

The three geometric points must appear in the correct order:

- `s_turn + eps <= s_apex`
- `s_apex + eps <= s_exit`

#### 2. Track-width constraint

The vehicle center must stay within the simplified track-width limits:

- `-w - W_car/2 <= n_i(z) <= w + W_car/2`

where:

- `w` is the track half-width
- `W_car` is the vehicle width
- `n_i(z)` is the lateral offset at discretization point `i`

#### 3. Curvature-based friction constraint

- `q_i^2 * kappa_i(z)^2 <= (mu * g)^2`

where:

- `kappa_i(z)` is the path curvature at point `i`
- `mu` is the friction coefficient
- `g` is gravitational acceleration

#### 4. Acceleration constraint

- `q_(i+1) - q_i <= 2 * a_acc_max * Delta_s`

#### 5. Braking constraint

- `q_i - q_(i+1) <= 2 * a_brake_max * Delta_s`

#### 6. Entry-speed equality

- `q_1 = v_in^2`

#### 7. Variable bounds

Box bounds are imposed on:

- `s_turn`
- `s_apex`
- `s_exit`
- `q_i`

---

## Geometry Definition

The geometric racing line is not fully free.

It is constructed from five control points:

1. `entry_point`
2. `turn-in point`
3. `apex point`
4. `exit-opening point`
5. `exit_point`

The actual optimized geometric variables are only:

- `s_turn`
- `s_apex`
- `s_exit`

These are arc-length locations along the centerline.  
At each of these locations, the code places a control point on the appropriate inside/outside side of the track, then connects all control points with a **cubic spline**.

So the current line family is:

- relatively low-dimensional
- easy to interpret
- numerically stable

but also limited in geometric flexibility.

---

## Geometry Cases

### 1. Toy corner

The project was first validated on a toy corner to verify:

- spline path generation
- curvature computation
- lateral offset computation
- single-level constrained optimization

### 2. Baku Turn 15-inspired segment

The current baseline uses a **Baku Turn 15-inspired approximation**.

Important note:

> This is **not** an official FIA-grade measured geometry of Baku Turn 15.  
> It is a practical approximation constructed from a hand-crafted curvature profile and qualitative corner characteristics.

The current approximation is designed to capture:

- a fast approach
- a short rightward attitude under braking
- a tighter left-hand braking corner
- a short unwind on exit

---

## Current Numerical Strategy

The current implementation uses:

- SciPy `trust-constr`
- warm-started initial guess
- curvature-based initialization of speed variables
- second-stage projected warm start
- boundary inset (`boundary_margin`) to reduce spline overshoot near track edges

This combination was needed to obtain a fully feasible baseline solution.

---

## Repository Structure

```text
cornering-opt/
├─ README.md
├─ pyproject.toml
├─ requirements.txt
├─ .gitignore
│
├─ configs/
│  └─ base.yaml
│
├─ data/
│  ├─ raw/
│  └─ processed/
│
├─ docs/
├─ notebooks/
│
├─ results/
│  ├─ figures/
│  ├─ logs/
│  └─ runs/
│
├─ scripts/
│  ├─ run_toy_case.py
│  ├─ run_baku_t15_speed_sweep.py
│  ├─ analyze_baku_t15_speed_sweep.py
│  └─ run_experiment.py
│
├─ src/
│  └─ cornering/
│     ├─ __init__.py
│     ├─ geometry/
│     │  ├─ centerline.py
│     │  ├─ path_builder.py
│     │  └─ curvature.py
│     ├─ models/
│     │  └─ config.py
│     ├─ optimization/
│     │  ├─ constraints.py
│     │  ├─ objective.py
│     │  └─ solver.py
│     └─ utils/
│        ├─ baku_t15.py
│        ├─ plotting.py
│        └─ toy_track.py
│
└─ tests/
   └─ test_smoke.py
````

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/cornering-opt.git
cd cornering-opt
```

### 2. Create a virtual environment

#### PowerShell

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

If script execution is blocked:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

### 3. Install dependencies

```powershell
pip install -r requirements.txt
```

---

## How to Run

### Run the current Baku Turn 15 baseline

From the repository root:

```powershell
$env:PYTHONPATH = ".\src"
python .\scripts\run_toy_case.py
```

Note: `run_toy_case.py` currently runs the **Baku Turn 15-inspired case**, even though the filename still reflects the earlier toy-corner stage.

This baseline run outputs:

* optimizer termination status
* objective value
* optimized geometric parameters
* constraint diagnostics
* optimized racing-line plot
* curvature profile
* speed profile

---

### Run the entry-speed sweep

```powershell
$env:PYTHONPATH = ".\src"
python .\scripts\run_baku_t15_speed_sweep.py
```

This sweep saves:

* per-case summary data
* per-case path and speed profile data
* figures for each case
* a summary CSV in `results/runs/`

---

### Analyze the sweep results

```powershell
$env:PYTHONPATH = ".\src"
python .\scripts\analyze_baku_t15_speed_sweep.py
```

This analysis script generates:

* objective vs entry speed
* geometric line parameters vs entry speed
* minimum speed vs entry speed
* constraint violations vs entry speed
* path overlay across successful runs
* speed profile overlay across successful runs

---

## Main Code Components

### `src/cornering/models/config.py`

Defines the configuration dataclass for:

* geometry
* discretization
* physical parameters
* variable bounds

### `src/cornering/geometry/path_builder.py`

Responsible for:

* centerline interpolation
* control-point generation
* spline path construction
* curvature computation
* lateral offset computation
* path feasibility checks

### `src/cornering/optimization/objective.py`

Defines:

* decision-vector unpacking
* minimum-time objective evaluation

### `src/cornering/optimization/constraints.py`

Defines:

* nonlinear constraint functions
* bounds
* diagnostics for each constraint type

### `src/cornering/optimization/solver.py`

Defines:

* initial guess generation
* warm-start logic
* second-stage projected warm start
* `trust-constr` solve routine

### `src/cornering/utils/baku_t15.py`

Defines the current Baku Turn 15-inspired segment approximation.

---

## Current Results Summary

### Feasible range

At the moment, the model is reliably feasible for entry speeds approximately in the range:

* `17-23 m/s`

### Infeasible range

For higher entry speeds such as:

* `24 m/s`
* `26 m/s`

the current setup becomes infeasible, mainly due to:

* braking constraint violation
* friction constraint violation
* some bounds violation

This suggests that, under the current geometry and physical assumptions, the model reaches the feasible operating limit of the corner.

### Interpretation

Within the successful entry-speed range:

* the optimal geometric line is almost unchanged
* the speed profiles differ mainly in the approach phase
* the curves then converge to a very similar minimum-speed region through the main corner

This suggests that the current model responds to higher entry speeds mainly through:

* **different braking behavior**

rather than through:

* **substantial geometric restructuring of the racing line**

---

## Limitations

The current implementation is intentionally simplified.

Main limitations include:

* the Baku Turn 15 geometry is only an approximation
* the racing line is parameterized by only three geometric variables
* the line family may therefore be too rigid to show richer geometric adaptation
* current results are based on a reduced-order path-and-speed model, not a full vehicle dynamics simulator

---

## Next Steps

The next development stage can go in either of two directions.

### Direction 1: Result consolidation

Use the current baseline and sweep results to write up:

* modeling setup
* numerical method
* feasible speed range
* geometric stability of the optimal line
* braking-dominated adaptation behavior

### Direction 2: Model extension

If stronger geometric line variation is desired, possible extensions include:

* wider geometric bounds
* more spline control points
* direct optimization of lateral offset profiles
* improved corner geometry approximation
* richer vehicle dynamics

---

## Research Goal

The broader goal of this project is to study:

> how the optimal cornering line changes with entry speed,

using a tractable but physically meaningful constrained optimization formulation.

---

## Notes

* The Baku Turn 15 geometry used here is an approximation, not official track-survey data.
* The current implementation prioritizes interpretability and solver stability over full vehicle-dynamics fidelity.
* This is a reduced-order research prototype rather than a full motorsport simulation package.

````

