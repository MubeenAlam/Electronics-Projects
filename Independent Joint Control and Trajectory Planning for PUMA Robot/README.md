# Independent Joint Control and Trajectory Planning for PUMA Robot

## Aim of the Project

Robots are widely used in modern industrial and commercial applications. As robots are assigned increasingly complex tasks, efficient and stable control mechanisms become essential.

The aim of this project is to:
- Implement an **Independent Joint Controller** for a **PUMA robot**
- Plan a **joint-space trajectory** between two Cartesian points
- Analyze the **controller performance** using simulation results

---

## Methodology

This project uses an **Independent Joint Control** scheme where each robot joint is controlled separately without coupling effects.

### Control Architecture
1. **User Input**  
   - Initial and final Cartesian coordinates

2. **Inverse Kinematics**  
   - Converts Cartesian coordinates into joint angles

3. **Joint Space Trajectory Planner**
   - Generates smooth joint trajectories between start and end positions

4. **Independent Joint Controllers**
   - Each joint has its own controller with:
     - Proportional feedback (position)
     - Velocity feedback (damping)

5. **Forward Kinematics**
   - Used to compute end-effector position for trajectory visualization

---

## Independent Joint Controller

The controller used is a **Proportional + Velocity (PV) Controller**.

### Previous Controller
- Pure **Proportional (P) Controller**
- Caused oscillations and overshoot

### Improved Controller
- Added **velocity feedback**
- Benefits:
  - Reduced overshoot
  - Improved damping
  - Better stability

---

## Controller Calculations

The motor and controller system is modeled using transfer functions.

### Key Parameters
- Natural frequency:  
  \[
  \omega_n
  \]

- Damping ratio:  
  \[
  \zeta
  \]

Motor constants:
- Armature resistance
- Torque constant
- Back-EMF constant
- Effective moment of inertia

Controller gains:
- Proportional gain \( K_p \)
- Velocity gain \( K_v \)

---

## Simulation Setup

### Files Used

- **Simulink**
  - `my_puma.mdl` – Main simulation model

- **MATLAB Scripts**
  - `vars.m`
    - Calculates controller gains
    - Performs inverse kinematics
    - Generates joint trajectories
  - `End_point_graph.m`
    - Computes forward kinematics
    - Plots end-effector trajectory in 3D

---

## Results and Plots

### End Effector Trajectory
- 3D plot showing motion from initial to final position
- Start point marked
- Reference frame origin shown

### Position Error
- Small steady-state error observed
- Error < **0.025 rad** for joints 2 and 3

### Joint Velocity
- Joint 3 shows under-damped behavior
- Minor oscillations observed

---

## Discussion

- Independent joint control performs well for **large motions**
- Small, precise movements show:
  - Steady-state error
  - Slight oscillations
- Velocity feedback improves stability but does not fully eliminate error

---

## Conclusion

- Independent Joint Controllers are:
  - Simple
  - Effective for large-scale tasks such as:
    - Pick and place
    - Welding
    - Cutting

- They are **not ideal for high-precision micro-movements**
- More advanced controllers are required for higher accuracy tasks

---

## Code

### MATLAB Files
- `vars.m` – Parameter calculation, inverse kinematics, trajectory planning
- `End_point_graph.m` – Forward kinematics and 3D plotting

---

## Tools Used
- MATLAB
- Simulink
- GitHub

---


