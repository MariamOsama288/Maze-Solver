# Autonomous Maze Solver Robot (Webots Simulation) 🤖🗺️

An autonomous maze-solving robot developed in Python using the **Webots simulation platform**. The robot utilizes proximity distance sensors to implement wall-following behaviors and leverages a Supervisor node pipeline to locate and navigate toward a specific red target pipe.

## 📝 Project Overview
The core objective of this project is to guide an e-puck (or custom differential drive) robot through a complex, walled environment. The control architecture uses a **Finite State Machine (FSM)** combined with basic kinematics to systematically search for walls, follow them closely, make controlled turns, and pivot toward the target once detected.

## ⚙️ Finite State Machine (FSM) Architecture
The robot logic continuously transitions through specific states based on distance sensor feedbacks and target proximity:
- `search_wall`: Moves straight forward until a structural boundary is encountered.
- `follow_wall`: Adjusts wheel velocities to maintain a fixed distance relative to the left/right walls.
- `turn_right` / `turn_left`: Triggers crisp, in-place rotations when forward obstacles are detected.
- `curve_+90`: Executes curved turns to handle sharp layout corners smoothly.
- `move_to_pipe`: Overrides general wall-following to steer directly toward the red target.
- `reached_pipe`: Stops both motors completely once the target distance drops below the set threshold (`0.1m`).

## 🛠️ Hardware & Devices Setup
The controller interacts with the following hardware components:
- **8 Distance Sensors (`ps0` to `ps7`)**: Read obstacle proximity inputs to calculate directional alignment.
- **Wheel Encoders**: Track positional updates from the left and right wheels to measure speed commands.
- **Differential Drive Motors**: Actuated continuously using inverted kinematics commands constrained under `MAX_SPEED = 6.28`.
- **Supervisor Node**: Accesses global world matrices to read the robot's real-time position/orientation and evaluate the target node `pipe(1)` values.

## 📊 Mathematical & Kinematic Models
The controller converts linear velocity ($u$) and angular velocity ($w$) targets into actual commands using standard differential kinematics equations:
- **Wheel Radius ($R$)**: $0.0205$ meters.
- **Axle Track ($D$)**: $0.0520$ meters.
- **Wall Following Controller**: Uses proportional error evaluation ($kd = 0.003$) combined with angle error compensation ($kd2 = 0.003$) to minimize trajectory offsets.

## 🚀 How to Run the Simulation
1. Install **Webots Simulation Software**.
2. Clone this repository into your local directory.
3. Open the corresponding world file (`.wbt`) inside Webots.
4. Associate this python controller script with your robot node.
5. Click **Play** to watch the robot solve the maze and reach the red target pipe!
