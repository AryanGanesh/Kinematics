🤖 Robotics Kinematics & Visualization Suite
This repository contains essential scripts for understanding, simulating, and visualizing the kinematics of robotic manipulators, covering both Denavit-Hartenberg (DH) parameter conventions and analytical Inverse Kinematics (IK) solutions for common planar arm configurations.

The code is split between Python (utilizing matplotlib and numpy) for hands-on, specific planar arm analysis, and MATLAB (utilizing the Symbolic Toolbox) for generalized DH transformation matrix derivation and 3D frame visualization.

📁 Repository Structure
File Name

Language

Description

Key Functionality

dof2.py

Python

2-DOF Planar Arm Kinematics

Calculates Inverse Kinematics (IK) for a 2R arm. Plots the reachable workspace and the final arm configuration for the two solutions (elbow-up/elbow-down).

dof3.py

Python

3-DOF Planar Arm Kinematics

Calculates Inverse Kinematics (IK) for a 3R arm (position and orientation). Uses DH convention and plots the workspace and final configuration.

DH_params_visualisation.m

MATLAB

Generalized DH Visualization

Takes user-defined DH parameters (numeric or symbolic) for an N-DOF robot, calculates the Homogeneous Transformation Matrices (HTM), and visualizes the coordinate frames in 3D space.

🚀 Getting Started
Prerequisites
Tool

Purpose

Status

Python 3.x

Used for the dof2.py and dof3.py scripts.

Required

numpy

Mathematical operations in Python.

Required

matplotlib

Plotting arm configurations and workspaces in Python.

Required

MATLAB

Used for the DH_params_visualisation.m script.

Required

MATLAB Symbolic Math Toolbox

Essential for symbolic DH matrix derivation.

Required for .m file

Installation
After cloning the repository, follow these steps to set up your environment:

Install Python Packages:
The Python scripts (dof2.py and dof3.py) rely on two primary libraries:

numpy for efficient array and matrix operations required for kinematics calculations.

matplotlib for generating the interactive arm plots and workspace visualizations.

Run the following command in your terminal:

pip install numpy matplotlib

Verify MATLAB Symbolic Toolbox:
The DH_params_visualisation.m file requires the Symbolic Math Toolbox within your MATLAB installation. If you encounter an error when running the script, ensure this toolbox is installed and licensed.


## 💡 Usage

### 1. 2R and 3R Planar Arm Inverse Kinematics (`dof2.py` & `dof3.py`)

Run the Python scripts directly from your terminal. They are interactive and will prompt you for link lengths and the target end-effector coordinates.

```bash
# For 2-DOF planar arm analysis
python dof2.py

# For 3-DOF planar arm analysis (position + orientation)
python dof3.py

Output: Each script will first plot the full reachable workspace and then display one or two separate plots showing the arm in the calculated configuration reaching the specified target point.

2. Generalized DH Visualization (DH_params_visualisation.m)
Open the .m file in MATLAB and run it. The script will guide you through entering the Denavit-Hartenberg parameters for your specific robot (N-DOF).

>> DH_params_visualisation


Key Features:

Accepts both numeric (for immediate plotting) and symbolic (for HTM derivation) DH parameters.

Plots all coordinate frames (Frame 0 to Frame N) in a 3D figure, allowing you to visualize the link geometry.

Calculates and prints the final symbolic and numerical Homogeneous Transformation Matrices (T 
0
N
​
 )
