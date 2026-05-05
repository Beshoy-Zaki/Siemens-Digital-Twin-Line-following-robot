# Line-Following Robot Simulation (Digital Twin)

**Author:** Beshoy Zaki Farouk  
**Email:** beshoy21102005@gmail.com  
**Organization:** Siemens (Digital Twin Training)  
**Simulation Framework:** Innexis Virtual System Interconnect (VSI) 2025.2  
**Date:** March 17, 2026

---

## 1. Overview

This project simulates a line-following robot using a three-client Digital Twin architecture powered by the Innexis VSI 2025.2 framework. It demonstrates closed-loop PID control of a differential-drive robot tracking a reference path on a 2D plane. The entire simulation and its components operate over a real-time CAN protocol backplane.

**Core Features:**
- Three separate simulation clients for plant, controller, and visualization/KPIs
- Communication via VSI Fabric Server and CAN protocol
- Fully parameterized PID control (P, PI, PD, PID)
- Supports straight and sinusoidal (curved) reference paths
- Robust to sensor noise and initial misalignment

---

## 2. Architecture & Components

The system is composed of three Python-based clients managed by the VSI Simulator:

| Component   | Client ID | Port   | Role                                      |
|-------------|-----------|--------|-------------------------------------------|
| plant.py    | 0         | 50101  | Robot unicycle kinematics + sensors       |
| controller.py | 1       | 50102  | PID steering controller                   |
| visualizer.py | 2       | 50103  | Logger + KPI reporter                     |

All clients synchronize via the VSI Fabric Server through CAN packets, with each signal assigned a unique CAN ID.

**Key CAN IDs:**
- `x`:         0x14 (20),  Plant → All, Robot x-position (m)
- `y`:         0x15 (21),  Plant → All, Lateral error from path (m)
- `theta`:     0x16 (22),  Plant → All, Heading angle (rad)
- `omega`:     0x17 (23),  Controller → Plant, Steering command (rad/s)

---

## 3. Robot Kinematic Model

- **State:** x, y, theta (position and heading)
- **Dynamics:** Discretized forward Euler integration, time step = 1 ms
- **Control:** Angular velocity (omega), with constant forward velocity (v = 1.0 m/s)
- **Reference Paths:**  
  - Straight: y_ref(x) = 0  
  - Curved:  y_ref(x) = 0.5 * sin(0.2 * x)

---

## 4. PID Controller

- Standard discrete PID acting on lateral error:
  - omega[k] = Kp*e[k] + Ki*sum(e[i]*DT) + Kd*(e[k]-e[k-1])/DT
- Output clamped to [-3.0, 3.0] rad/s
- Typical baseline gains: **Kp=2, Ki=0.1, Kd=0.5** (others tested)
- Integral term for bias & steady-state error; Derivative for noise and overshoot damping

---

## 5. Experimental Results (Key Highlights)

- **Gain Sweep:** Higher Kp/Kd improves convergence and error; integral term needed for bias rejection
- **Curved Path:** High-gain controllers track sinusoids more tightly
- **Noise:** Controller robust to significant Gaussian noise disturbances
- **PD vs PID:** Derivative dominates under noise/curve; integral provides marginal benefit in this context

---

## 6. File Structure

```
LineFollower_Success/
|-- LineFollowingRobot.dt        # Digital Twin configuration (component mapping)
|-- pref.cvsi                    # Simulation run parameters
|-- FabricServer                 # VSI Fabric Server binary (pre-compiled)
|-- pythonGateways/              # VSI gateway libraries (pre-compiled .so)
|-- src/
|    |-- plant/plant.py          # Plant client
|    |-- controller/controller.py# Controller client
|    |-- visualizer/visualizer.py# Visualizer client
|-- results/                     # CSV log files, KPI reports, plots
```

---

## 7. Usage and Execution

> **Note:** Requires Innexis VSI 2025.2 installation and access to FabricServer & pythonGateways.

**Typical Run Steps:**
1. **Start FabricServer**  
   ```
   ./FabricServer --config pref.cvsi
   ```
2. **Launch Each Client** in separate terminals:
   - Plant: `python src/plant/plant.py`
   - Controller: `python src/controller/controller.py`
   - Visualizer: `python src/visualizer/visualizer.py`

   All clients will connect to the FabricServer, synchronize, and begin simulation.
3. **Simulation Output:**  
   Results, logs, and KPI plots will be saved in the `results/` directory.

---

## 8. Key API Calls (Python Clients)

- `vsiCommonPythonApi.connectToServer(url, domain, port, id)` — Connects to server
- `vsiCanPythonGateway.initialize(session, id)` — Initialize CAN gateway
- `vsiCommonPythonApi.waitForReset()` — Wait for simulation reset
- `vsiCanPythonGateway.setCanId(id)` — Select CAN frame ID
- `vsiCanPythonGateway.setCanPayloadBits(data, 0, 64)` — Pack double payload
- `vsiCanPythonGateway.sendCanPacket()` — Send CAN frame
- `vsiCanPythonGateway.recvVariableFromCanPacket(8,0,64,id)` — Receive CAN frame
- `vsiCommonPythonApi.advanceSimulation(dt_ns)` — Advance clock

---

## 9. Contact

For questions or support, contact:  
Beshoy Zaki Farouk  
[beshoy21102005@gmail.com](mailto:beshoy21102005@gmail.com)
