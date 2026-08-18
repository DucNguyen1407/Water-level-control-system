# Water Level Control System

A MIMO process-control study of an **interacting two-tank water level system**, covering mathematical modeling, decoupling network design, feedforward disturbance compensation, and PI controller design via **Direct Synthesis** and **Root Locus** methods — implemented and simulated in **MATLAB/Simulink**.

---

## Overview

```
┌─────────────┐   F1 (disturbance)
│   Tank 1    │◄──────────────────
│    h1       │
└──────┬──────┘
       │ F2 (valve, controlled)
       ▼
┌─────────────┐
│   Tank 2    │
│    h2       │
└──────┬──────┘
       │ F3 (valve, controlled)
       ▼
    Outflow
```

Two interacting tanks are connected in series. Inflow `F1` acts as a load disturbance on Tank 1, while outflow valves `F2` (between tanks) and `F3` (tank 2 outlet) are the manipulated variables used to regulate levels `h1` and `h2`.

---

## Features

- Nonlinear dynamic modeling of a two-tank interacting system based on valve flow equation `F = Cv·l·√(ΔP/gs)`
- Degrees-of-freedom analysis (5 variables − 2 equations = 3 DOF)
- Linearization around a nominal operating point (h = 1.5 m)
- Conversion of the linearized ODE model into a transfer-function (Laplace domain) representation
- MIMO transfer matrix `Gp(s)` (process) and `Gd(s)` (disturbance)
- Ideal decoupling network `D(s)` to eliminate cross-channel interaction and reduce the MIMO system to two independent SISO loops
- Two independent PI controller design methods:
  - **Direct Synthesis** combined with an ideal decoupler and a disturbance feedforward compensator
  - **Root Locus** with pole-zero cancellation, validated via Relative Gain Array (RGA) analysis
- P&ID flowsheet design for feedback + feedforward control strategy
- Simulink simulation and comparison of closed-loop responses with/without decoupler and with/without disturbance compensator

---

## System Model

### Process Variables

| Variable | Role |
|----------|------|
| `F1` | Disturbance (inflow to Tank 1) |
| `F2` | Manipulated variable (valve between Tank 1 and Tank 2) |
| `F3` | Manipulated variable (outlet valve on Tank 2) |
| `h1` | Controlled variable (Tank 1 level) |
| `h2` | Controlled variable (Tank 2 level) |

### Governing Equations

Each tank's level is described by a mass balance where the rate of change of stored volume equals inflow minus outflow, with outflow through each valve following a nonlinear square-root flow relationship.

Degrees-of-freedom analysis confirms the model is consistent, with the number of independent equations matching the number of manipulated/disturbance inputs.

### Linearized Transfer-Function Model

Linearizing the nonlinear balance equations around the operating point and applying the Laplace transform produces a MIMO transfer-function model relating the two manipulated valve positions and the disturbance flow to the two tank levels:

`y = Gp(s)·u(s) + Gd(s)·d(s)`

where `Gp(s)` is the process transfer matrix (valves → levels), `Gd(s)` is the disturbance transfer matrix (inflow → levels), `u` is the vector of valve inputs, and `y` is the vector of tank levels. Off-diagonal terms in `Gp(s)` represent the cross-coupling between the two tanks.

---

## Simulation Parameters

The model is simulated using representative parameters for water as the working fluid, a fixed nominal valve opening, a chosen operating level, and fixed tank/valve sizing — used throughout as the basis for linearization and controller tuning.

---

## Control Design

### P&ID Strategy

A **feedback + feedforward** control scheme is used to reject the `F1` load disturbance while regulating `h1` and `h2` via level transmitters (LT) and level controllers (LC) acting on valves `F2` and `F3`.

### (a) Direct Synthesis + Ideal Decoupling

- An ideal decoupler `D(s)` is placed ahead of the process so that the combined system becomes diagonal, splitting the MIMO plant into two independent SISO loops by canceling the off-diagonal (cross-coupling) transfer function terms.
- Each decoupled loop is then treated as an independent SISO system and tuned using Direct Synthesis, specifying a desired closed-loop response and solving for the corresponding PI controller.
- A feedforward disturbance compensator is derived from the steady-state mass balance, computing the valve adjustments needed to counteract changes in the inflow disturbance before they affect tank levels.

### (b) Root Locus with Pole-Zero Cancellation

- RGA (Relative Gain Array) analysis at steady state confirms negligible cross-coupling between the two channels at equilibrium, allowing the loops to be paired and tuned independently.
- Each PI controller's zero is placed to cancel the dominant slow pole of its associated process channel, shaping the closed-loop response via the resulting root locus as gain is varied.
- The controller gain is chosen within the range that avoids valve saturation while maximizing response speed in the linear operating region.

---

## Simulation Results

Simulink models were built for both control strategies, each including:
- The two-tank interacting process block
- The controller block (PI + decoupler / PI + feedforward compensator)
- Step disturbance and setpoint inputs

Results compare closed-loop level responses **with and without** the decoupling network and **with and without** the disturbance feedforward compensator, showing significantly reduced cross-tank interaction and disturbance sensitivity when both are included.

---

## Controller Method Comparison

| Aspect | Root Locus | Direct Synthesis + Decoupling |
|--------|-----------|-------------------------------|
| Intuition | Visual, tracks closed-loop pole movement with gain `K` | Algebraic; decouples MIMO into SISO first |
| Best suited for | SISO or weakly-coupled systems | Strongly-coupled MIMO systems |
| Model sensitivity | Performance degrades if model is inaccurate | Very sensitive — decoupler relies on exact model |
| Disturbance rejection | Not inherently strong | Improved via explicit feedforward compensation |
| Implementation complexity | Straightforward, standard MATLAB/Simulink tools | More complex (matrix derivation, decoupler design) |
| Industrial deployment | Simple to implement on PLC | Harder — dynamic decoupler blocks are resource-intensive |

---

## Other Control Approaches Considered

- **PID Control** — Ziegler-Nichols or relay feedback tuning around a fixed operating point
- **Sliding Mode Control (SMC)** — robust nonlinear control using a sliding surface
- **Optimal Control** — state-space based, minimizing a cost function balancing level error and valve energy
- **Intelligent Control**:
  - *Fuzzy Logic* — rule-based control without requiring an exact math model
  - *Artificial Neural Networks* — learns and approximates the system's nonlinear dynamics from operating data
  - *Adaptive Control* — automatically retunes parameters as system characteristics (e.g., valve wear affecting `Cv`) change over time

---

## Conclusion

This project models, linearizes, and designs feedback/feedforward controllers for an interacting two-tank MIMO water level system. Two PI tuning methods — Direct Synthesis with ideal decoupling, and Root Locus with pole-zero cancellation — were implemented and compared in Simulink, both showing improved performance when combined with decoupling and disturbance-compensation techniques.

## Contribution

Contributions are welcome! Please feel free to submit issues or pull requests. 


