# 🤖 Autonomous Navigation with PID & LIDAR in PyBullet

This project simulates a differential drive robot navigating autonomously in a dynamic environment using a **PID controller** and **LIDAR-style ray scanning**, implemented in Python with PyBullet.

## 📌 Key Features

- Simulates a racecar robot navigating toward a target while avoiding obstacles
- Uses simulated LIDAR via `rayTestBatch()` for perception
- Implements real-time control using steering angle computation
- Randomized obstacle generation to test navigation in dynamic settings
- Custom URDF loading of racecar and obstacle objects

## 🛠 Tech Stack

- Python
- [PyBullet](https://github.com/bulletphysics/bullet3)
- NumPy
- Math / Random

## 🧠 How It Works

- A racecar URDF (`racecar_differential.urdf`) is loaded onto a simulated plane.
- 10 random obstacles are spawned using various colored cube models.
- LIDAR-like perception is implemented using `rayTestBatch()` to detect surrounding objects.
- A steering angle is dynamically calculated toward a goal position using relative orientation.
- Wheels and steering are controlled via PyBullet joint APIs.

## 📂 Project Structure

finalproject_robotics/
├── main.py # Full navigation + LIDAR code (this file)
├── data/
│ ├── plane/
│ │ └── plane.urdf
│ ├── cube_black/
│ ├── cube_green/
│ └── cube/ # All contain URDF files for cubes
└── README.md 


