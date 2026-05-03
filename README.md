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

The current baseline uses a **Baku Turn 15-inspired segment** and successfully converges to a strictly feasible solution.

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

This means the current optimization pipeline is working end-to-end and produces a numerically converged feasible baseline solution.

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

## Geometry

### Toy corner

The project was first validated on a toy corner to verify:

- spline path generation
- curvature computation
- lateral offset computation
- single-level constrained optimization

### Baku Turn 15-inspired segment

The current baseline uses a **Baku Turn 15-inspired approximation**.

Important note:

> This is **not** an official FIA-grade measured geometry of Baku Turn 15.  
> It is a practical approximation constructed from a hand-crafted curvature profile and public qualitative understanding of the corner.

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

This combination was necessary to obtain a fully feasible baseline solution.

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

This currently runs the **Baku Turn 15-inspired case** and outputs:

* optimizer termination status
* objective value
* optimized geometric parameters
* constraint diagnostics
* optimized racing-line plot
* curvature profile
* speed profile

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

## Current Baseline Result

A successful baseline run currently gives a feasible solution with:

* early turn-in near the lower bound
* mid-to-late apex
* late exit-opening
* significant speed drop through the corner
* zero violation in all modeled constraints

This baseline should now be treated as the stable reference version for future experiments.

---

## Next Step: Entry-Speed Sweep

The next development stage is to run an **entry-speed sweep**, for example:

* `v_in in {18, 20, 22, 24, 26}`

and compare how the optimal:

* turn-in location
* apex location
* exit-opening location
* speed profile
* traversal time

change as the entry speed varies.

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