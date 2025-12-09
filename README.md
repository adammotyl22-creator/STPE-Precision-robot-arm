# 🦾 STPE-Precision-Robot-Arm

An open-source **5-DOF** robot arm (5 positional axes + gripper) that I plan to create for precise **SMD teleoperation** (Surface-Mount Device). The **Closed-Loop system** (NEMA 17 + AS5048A) managed by **Teensy 4.1** & **TMC2209** is **intended to achieve $\mathbf{0.5\text{ mm}}$ accuracy** by actively correcting gear backlash.

### 🧠 AI / Project Augmentation
I **plan to use** Artificial Intelligence (AI) tools to support critical development phases, such as optimizing algorithms (e.g., generating and refining **Inverse Kinematics (IK)** equations) and precise tuning of the **PID control loop** **to achieve $\mathbf{0.5\text{ mm}}$ stability and analyze the impact of ML on these control functions.**

---

## 📐 Mechanical Specification & Kinematics

* **Degrees of Freedom (DOF):** $\mathbf{5\text{ Positional DOF}}$ (5 controlled axes for position/orientation).
    * **Axes 1-3 (Main Position):** Steppers with Closed-Loop control (Base Yaw, Lower Arm Pitch, Upper Arm Pitch).
    * **Axes 4-5 (Wrist Orientation):** Will be controlled by Servos (providing two axes of wrist movement: Roll, Pitch, or Yaw).
    * **Gripper Axis:** Will be controlled by a Servo (Open/Close).
* **Inverse Kinematics (IK):** The IK algorithm **will be implemented** on the Teensy 4.1. It will map target $\text{X}, \text{Y}, \text{Z}$ coordinates and end-effector orientation to precise angles for **all 5 positional axes**.
* **Inspiration:** The design **is inspired by** [PCrnjak/PAROL6](https://github.com/PCrnjak/PAROL6-Desktop-robot-arm/tree/main) (IK complexity) and the [Compact Robot Arm](https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed) (form factor).

---

## 🔌 Electronics Architecture and Wiring

* **MCU:** **Teensy 4.1** ($600\text{ MHz}$) – **Will be utilized** for real-time PID/IK calculations.
* **Closed-Loop Core:** **AS5048A** encoders ($14\text{-bit}$) **will communicate** with the Teensy via the **SPI** bus. **TMC2209** drivers **will receive** **STEP/DIR** signals.
* **Power:** Stable $24\text{V}$ (LRS-150-24V) for the stepper motors. Isolation from the $3.3\text{V}/\text{5V}$ logic **will be ensured** using **LM2596** converters and Level Shifters.

---

## 💻 Software and Firmware

* **Development:** I **will use** **PlatformIO** (or Arduino IDE).
* **Functionality:** I **will implement** a **Trajectory Generator** for smooth accelerations and a **Teleoperation Module** for handling input data from the control device.

---

## 🚧 Status / Planned Progress

* [x] Repository creation and documentation added
* [x] Key component selection and purchase
* [ ] Mechanical assembly (structure)
* [ ] Electronics soldering and wiring (Wiring)
* [ ] Implementation of the **5-DOF IK Solver**
* [ ] Implementation and tuning of the **PID loop**
* [ ] Precision testing ($\le 0.5\text{ mm}$)
