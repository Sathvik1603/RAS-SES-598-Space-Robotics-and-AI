# Cart-Pole Optimal Control Assignment

**Name:** Sathvik Merugu  
**ASU ID:** 1235373752

---

## Introduction
This project focuses on designing and analyzing an **LQR (Linear Quadratic Regulator)** controller for a cart-pole system subjected to **earthquake-like disturbances**. The main goal is to:
- Keep the **cart within ±2.5 m** of its track,
- Maintain the **pole in the upright position**, despite disturbances.

---

## System Description

### Physical Configuration
- Inverted pendulum on a cart  
- **Track limits:** ±2.5 m (total range: 5 m)  
- **Pole length:** 1 m  
- **Cart mass:** 1.0 kg  
- **Pole mass:** 1.0 kg  

### Disturbance Profile
- Superimposed sine wave signals (frequency: 0.5–4.0 Hz)
- Amplitude: 15.0 N (with random fluctuations)
- Includes Gaussian noise to mimic realistic earthquake dynamics

---

## Objectives
- **Limit cart motion** within ±2.5 m
- **Minimize pole oscillations**
- **Reduce control effort** without compromising performance

---

## LQR Controller

### What is LQR?
LQR optimally stabilizes dynamic systems using state feedback, by minimizing a cost function that balances performance and control effort.

### LQR Equations
System state-space model:
```
dx/dt = Ax + Bu
```

Cost function:
```
J = ∫ (xᵀQx + uᵀRu) dt
```

Optimal control law:
```
u = -Kx
```
Where:
- \( Q \): State penalty matrix  
- \( R \): Control effort penalty  
- \( K \): Gain matrix computed from solving CARE (Continuous Algebraic Riccati Equation)

---

### Parameter Tuning

**Initial LQR matrices:**
```python
Q = diag([1.0, 1.0, 10.0, 10.0])
R = [[0.1]]
```

**Tuned LQR matrices:**
```python
Q = diag([1.0, 3.0, 1.0, 3.0])
R = [[0.01]]
```

**Tuning Rationale:**
- Increased velocity weight → faster stabilization
- Balanced pole angle vs. cart position
- Reduced R → allows quicker responses with acceptable control usage

---

## Performance Analysis

### Short-Term (first 20ms)
- Immediate reaction to disturbance
- Quick stabilization of both pole and cart

### Long-Term (10s)
- Slight oscillations quickly damped
- Efficient return to equilibrium
- Low overshoot and steady-state error

---

## Simulation Results

### Cart Position & Velocity
- Cart stays well within ±2.5m
- RMS cart travel reduced from **0.48 m → 0.38 m**

### Pole Stability
- Angular deviation reduced from **7.5° → 5.2°**
- System recovers from disturbances faster (30% improvement)

### Control Force
- Smooth actuation with no high-frequency spikes
- Slight increase in peak force, but energy efficient

---

## Improved Performance Summary

| Metric | Before | After |
|--------|--------|-------|
| RMS Cart Position | 0.48 m | 0.38 m |
| Max Pole Angle Deviation | 7.5° | 5.2° |
| Recovery Time | ~5s | ~3.5s |
| Control Force Spikes | Moderate | Lower |

---

## Simulation Info
- **Simulation Time:** 120 s  
- **Observed Recovery:** 3–5 s post disturbance

---

## Visual Results




https://github.com/user-attachments/assets/36c7035b-e494-4c26-bb69-eaf569d51218





### Cart Position:
![Screenshot 2025-05-10 012723](https://github.com/user-attachments/assets/d0d425ad-886f-44bd-9586-cc0da719a5e3)


### Pole Angle:
![Screenshot 2025-05-10 014500](https://github.com/user-attachments/assets/85634a2f-3000-40d3-be18-7b5900eb0e0f)


---

## Conclusion
The tuned LQR controller **effectively stabilized** the cart-pole system under dynamic seismic-like forces. It:
- Reduced angle deviation,
- Limited cart travel,
- Increased recovery speed,
- Maintained energy-efficient control input.
