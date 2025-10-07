# 🤖 Robot-Control

This module implements the **low-level velocity control** for the Omniwheel robot.  
The system receives motion commands via a **UDP server**, processes them, and controls each wheel independently using **PI controllers** designed from identified motor models.

---

## 🧩 Overview

The robot can execute several types of movements commanded from the host computer:
- **Linear motion** – Forward or backward with specified velocity and distance.  
- **Circular motion** – Rotation around a defined radius and angular speed.  
- **In-place rotation** – Spin around its own axis by a given angle.

Each command is sent through the UDP communication interface located in [`udp_server`](../udp_server).

---

## ⚙️ Control Architecture

- **Sensors used:** Wheel encoders only.  
- **Estimation:** Encoder-derived velocities are filtered using a **Kalman filter** to reduce noise.  
- **Model identification:** Each DC motor was modeled as a **first-order system** to obtain parameters for controller tuning.  
- **Controller:** A **PI (Proportional–Integral)** controller was implemented for precise wheel speed tracking.  
- **Kinematics:** Inverse kinematics are applied to convert desired robot body velocities into individual wheel targets.

<!-- START: Robot Control Flow Diagram -->

```mermaid
graph LR
    subgraph Command and Kinematics Level
        direction LR
        A[Motion Command (UDP)] 
        A -->|v, \alpha, \text{radius}| B[Command Processing]
        B -->|Body Velocity $v_x, v_y, \omega_z$| C[Inverse Kinematics]
        C --> |$\omega_{target}$| I{Σ Summing Junction}
        
        style A fill:#e0f7fa,stroke:#00796b,stroke-width:2px
        style B fill:#b3e5fc,stroke:#01579b,stroke-width:2px
        style C fill:#81d4fa,stroke:#01579b,stroke-width:2px
    end

    subgraph Velocity Control Loop (Per Wheel)
        direction LR
        I --> E[PI Controller]
        E --> F[DC Motor / Wheel (Plant)]
        F --> G(Encoder)
        G --> H[Kalman Filter]
        H -->|$-\omega_{filtered}$| I
        
        style I fill:#fff8e1,stroke:#ffd740,stroke-width:2px
        style E fill:#ffe0b2,stroke:#ffb74d,stroke-width:2px
        style F fill:#ffccbc,stroke:#ff8a65,stroke-width:2px
        style G fill:#f8bbd0,stroke:#e91e63,stroke-width:2px
        style H fill:#f48fb1,stroke:#e91e63,stroke-width:2px
    end
```

---

## 🧠 Relation to Higher-Level Modules

This module serves as the **foundation for [Robot-Navigation](../Robot-Navigation)**,  
where higher-level behaviors (trajectory tracking, obstacle avoidance, and sensor fusion) are built on top of this control layer.

---

## 🚀 How to Run

```bash
# Open project directory
cd Robot-Control

# Build and deploy to the microcontroller
make all
# or compile from your preferred IDE

# Start UDP server on PC
cd ../udp_server
python3 udp_server.py
```

Commands can then be sent to control the robot’s motion.

---

## 🧪 Validation

- Step response tests were used to validate the PI controller performance.
- Encoder velocity filtering was verified against noisy raw signals.
- Command tracking was analyzed for linear, circular, and rotational motions.

---

## License

This project inherits its license from the root repository. See the LICENSE file in the root of the repo.

---

© 2025 Striker2 
