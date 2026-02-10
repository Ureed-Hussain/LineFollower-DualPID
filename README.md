# LineFollower-DualPID
Dual-PID controlled line follower with five IR sensors for stable and accurate tracking.
---
A robust Arduino UNO line-following robot designed for **complex tracks** (sharp turns, curves, intersections/parallel segments) using **5 IR sensors** and a **dual-PID control strategy**:
- **Forward PID** for stable straight motion (reduced oscillation)
- **Turn/Tracking PID** for continuous line tracking and corrections

The code also includes **high-priority pattern handling** (override rules) for special cases like sharp turns, line loss, and tricky track patterns.

---

## ✨ Features

- ✅ **5x IR sensors** (L2, L1, C, R1, R2) for better track perception  
- ✅ **Dual PID control**
  - Forward PID (stable straight driving)
  - Main PID (weighted error from 5 sensors)
- ✅ **Complex path support**
  - sharp turns / pivots
  - search behavior when line is lost
  - special sensor-pattern overrides for tricky scenarios
- ✅ **Signed motor control**
  - allows reverse pivot turns (left motor backward / right forward, etc.)

---
## Path

![WhatsApp Image 2026-02-10 at 10 33 33 PM](https://github.com/user-attachments/assets/ba59f131-2579-4030-aee2-84d14ea2a30b)


The robot is designed to follow complex line paths, including:

* Sharp left and right turns
* Curves and continuous bends
* Parallel line segments
* Ambiguous patterns (partial line loss, edge-only detection)

To handle these scenarios, the algorithm does not rely only on continuous PID correction.
Instead, explicit sensor-pattern detection is used to override the PID when required, ensuring stability and preventing oscillations or line loss.


## 🔌 Hardware Required

- Arduino UNO  
- 5x IR line sensors (digital output recommended)  
- Motor driver (L298N / similar H-bridge)  
- 2 DC motors + chassis  
- Battery pack (sufficient current for motors)

---

## 🧷 Pin Mapping

<img width="813" height="385" alt="image" src="https://github.com/user-attachments/assets/58928aaa-3496-4d0d-946b-5b032f7e8b85" />

### Sensors
| Sensor | Arduino Pin |
|-------:|------------:|
| LEFT2 (far left) | A4 |
| LEFT1 | A3 |
| CENTER | A2 |
| RIGHT1 | A1 |
| RIGHT2 (far right) | A0 |

### Motor Driver
| Motor | Pin | Type |
|------:|----:|------|
| Left ENA | 9  | PWM |
| Left IN1 | 4  | DIR |
| Left IN2 | 5  | DIR |
| Right ENB | 10 | PWM |
| Right IN3 | 6  | DIR |
| Right IN4 | 7  | DIR |

---

## 🧠 Control Logic Overview

<img width="1411" height="826" alt="image" src="https://github.com/user-attachments/assets/0be2de93-11f8-4459-90a4-98cf13f62d5c" />


### 1) Priority Overrides (Pattern-Based)
Before running the main PID loop, the robot checks **explicit sensor patterns** and triggers dedicated actions like:
- forward stable drive
- straight crossing/parallel segments
- pivot search (left/right)
- hard turn patterns

These overrides are useful for “complex paths” where a pure PID may fail.

### 2) Forward PID (Straight Stability)
When the robot detects a stable centered line:
- Pattern: `L1=0, C=1, R1=0`
it uses a separate PID tuned for straight motion to avoid oscillation.

### 3) Main PID (Weighted 5-Sensor Tracking)
If no override matched, the robot computes a weighted position:

Weights:
- L2=-2, L1=-1, C=0, R1=+1, R2=+2

Then it applies PID correction to adjust motor speeds:
- `leftSpeed  = BASE_SPEED + correction`
- `rightSpeed = BASE_SPEED - correction`

### 4) Line Lost Handling
If all sensors read `0` for too long, the robot pivots in the **last known direction** to re-acquire the line.

---


## ⚙️ Key Parameters

### Speeds
- `BASE_SPEED` = base PWM during tracking  
- `MAX_SPEED` = absolute PWM limit (keeps headroom)  
- `turnSpeed`, `sharpTurnSpeed`, `searchSpeed` = behavior speeds  

### Main PID (tracking)
```cpp
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

// ====== Forward PID (to keep straight without oscillation) ======
float f_Kp = 6.0;     // small proportional gain 6 best put 4 value when base_speed is 70
float f_Kd = 5.0;     // small derivative gain 5
float f_lastError = 0.0;
unsigned long f_lastTime = 0;
```

There are many line-following codes available online, but most of them are not stable on complex paths.
In this project, PID control is applied separately for forward motion and for turning, which significantly improves stability.
As a result, the robot moves smoothly, consistently, and remains stable even on sharp turns and complex tracks.

