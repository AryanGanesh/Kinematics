# 🤖 Kinematics

![Header Banner](https://capsule-render.vercel.app/api?type=rect\&color=0:0D47A1,100:1976D2\&height=90\&section=header\&text=Kinematics\&fontSize=30\&fontColor=ffffff\&fontAlignY=55)

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8%2B-3776AB?style=flat-square&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/MATLAB-R2023a+-orange?style=flat-square&logo=mathworks&logoColor=white"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square"/>
  <img src="https://img.shields.io/github/stars/AryanGanesh/Kinematics?style=flat-square&color=yellow"/>
  <img src="https://img.shields.io/github/last-commit/AryanGanesh/Kinematics?style=flat-square&color=blue"/>
</p>

---

## 🧠 Overview

**Kinematics** is a comprehensive learning and visualization toolkit for **robotic manipulator kinematics** — from simple planar arms to generalized multi-DOF systems.
It blends **Python** (for analytical planar inverse kinematics and visualization) with **MATLAB** (for symbolic DH parameter derivation, 3D coordinate frame visualization, and real-time animation).

🎯 Designed for students, researchers, and robotics enthusiasts to **build deep intuition for how robotic arms move and orient themselves in space**.

---

## 📁 Repository Structure

| File                            | Language | Description                           | Key Functionality                                                                                    |
| ------------------------------- | -------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `dof2.py`                       | Python   | 2-DOF Planar Arm Kinematics           | Analytical IK for a 2R arm. Plots reachable workspace and both elbow-up/elbow-down poses.            |
| `dof3.py`                       | Python   | 3-DOF Planar Arm Kinematics           | Position + orientation IK using DH convention. Visualizes workspace and target configuration.        |
| `DH_params_visualisation.m`     | MATLAB   | Generalized DH Visualization          | Accepts numeric/symbolic DH parameters for N-DOF arms. Computes HTMs and plots 3D coordinate frames. |
| `Link_Manipulator_simulation.m` | MATLAB   | 2-Link Manipulator Workspace Animation | Real-time animation of 2R robot exploring linear trajectories with dual elbow-up/down visualization. |

---

## 🚀 Getting Started

### 🧩 Prerequisites

| Tool                      | Purpose                              | Status                                      |
| ------------------------- | ------------------------------------ | ------------------------------------------- |
| **Python 3.x**            | For 2R and 3R IK scripts             | ✅ Required                                  |
| **NumPy**                 | Matrix and trigonometric operations  | ✅ Required                                  |
| **Matplotlib**            | Visualization and workspace plotting | ✅ Required                                  |
| **MATLAB**                | For DH visualization & animations    | ✅ Required                                  |
| **Symbolic Math Toolbox** | Symbolic computation in MATLAB       | ⚙️ Required for `DH_params_visualisation.m` |

---

### ⚙️ Installation

Clone the repository:

```bash
git clone https://github.com/AryanGanesh/Kinematics.git
cd Kinematics
```

Install Python dependencies:

```bash
pip install numpy matplotlib
```

Ensure MATLAB's **Symbolic Math Toolbox** is available to run the DH visualization script.

---

## 💡 Usage

### 1️⃣ 2R & 3R Planar Arm Inverse Kinematics (Python)

```bash
# For 2-DOF arm
python dof2.py

# For 3-DOF arm (position + orientation)
python dof3.py
```

**Output:**

* Interactive plots showing the **reachable workspace**
* Visualization of **arm configurations** reaching the target

---

### 2️⃣ Generalized DH Visualization (MATLAB)

Run in MATLAB:

```matlab
>> DH_params_visualisation
```

**Features:**

* Accepts **numeric or symbolic** DH parameters
* Computes all **Homogeneous Transformation Matrices (HTMs)**
* Visualizes coordinate frames (**Frame 0 → Frame N**) in **3D**
* Displays final transformation ( T_{0}^{N} ) both symbolically and numerically

---

### 3️⃣ 2-Link Manipulator Workspace Animation (MATLAB)

Run in MATLAB:

```matlab
>> Link_Manipulator_simulation
```

**Features:**

* **Real-time animation** of a 2-link planar manipulator (35.35 units per link)
* Explores **linear trajectory** along positive X-axis with 5-unit spacing
* **Dual visualization**: Shows both **elbow-up** and **elbow-down** configurations simultaneously
* **Smooth interpolation** between target points for high-quality video recording
* Displays:
  * Workspace boundary (100×100 area: -50 to +50 on both axes)
  * Reachable workspace circle
  * Target points with current target highlighted
  * Real-time joint angles (θ₁ and θ₂) in degrees
  * Color-coded links and joints

**Perfect for:**
* Creating assignment videos and presentations
* Understanding inverse kinematics solutions
* Visualizing reachable workspace limits
* Comparing elbow-up vs elbow-down configurations

---

## ✨ Features

✅ Analytical IK solutions for 2R & 3R planar manipulators  
✅ Generalized DH parameter visualization for N-DOF systems  
✅ Real-time animation with dual configuration display  
✅ Symbolic computation for HTM derivation  
✅ Interactive plots for geometric understanding  
✅ Smooth trajectory interpolation for video recording  
✅ Educational focus — clear and modifiable for learning

---

## 📸 Demo Snapshots

> * <img width="711" height="716" alt="image" src="https://github.com/user-attachments/assets/5dc8eeb3-dacd-45bc-9952-247ec0a69894" />  
**2 DOF planar arm workspace**

> * <img width="1911" height="1019" alt="image" src="https://github.com/user-attachments/assets/b3a83ca4-ba17-4a00-8b0c-69a7acb4c374" />  
**3 DOF Planar arm solutions visualised**

> * <img width="825" height="839" alt="image" src="https://github.com/user-attachments/assets/4063ee4f-2450-4a49-bcf5-d452aa69be2e" />  
**N-DOF DH parameters Visualised**

> * **2-Link Manipulator Animation** *![WhatsApp Image 2025-11-03 at 8 48 42 AM](https://github.com/user-attachments/assets/471e1d49-3899-4ef5-b457-a569ab9709d7)
*  
**Real-time dual configuration animation exploring positive X-axis**

---

## 🎥 Recording Videos

For the 2-link manipulator animation, you can record high-quality videos using:

**Option 1: MATLAB Built-in Video Writer**
```matlab
% Add at the beginning of the script
v = VideoWriter('robot_animation.mp4', 'MPEG-4');
v.FrameRate = 20;
open(v);

% Inside animation loop, after drawnow
frame = getframe(gcf);
writeVideo(v, frame);

% After animation completes
close(v);
```

**Option 2: Screen Recording Software**
* **Windows**: Xbox Game Bar (Win + G)
* **Mac**: QuickTime Player or Screenshot toolbar (Cmd + Shift + 5)
* **Linux**: SimpleScreenRecorder or Kazam

---

## 🧑‍💻 Author

**Aryan Ganesh K.**  
Robotics & Mechatronics Engineer

<p align="left">
  <a href="https://www.linkedin.com/in/aryanganesh-kavuri-405684286" target="_blank">
    <img src="https://img.shields.io/badge/LinkedIn-Connect-blue?style=flat-square&logo=linkedin"/>
  </a>
  <a href="https://github.com/AryanGanesh" target="_blank">
    <img src="https://img.shields.io/badge/GitHub-Profile-black?style=flat-square&logo=github"/>
  </a>
</p>

⭐ **If you find this repository helpful, give it a star — it helps others discover and supports future development!**

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/AryanGanesh/Kinematics/issues).

---

## 📧 Contact

For questions or collaborations, reach out via LinkedIn or open an issue on GitHub.
