# Delivery Robot — Setup and Run

This project is intended for **Ubuntu 24.04** and **ROS 2 Jazzy**.

Follow the steps in order. The first build may take a few minutes.

## 1. Install the required packages

Open a terminal and run:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rqt-image-view \
  python3-colcon-common-extensions \
  python3-rosdep
```

Set up rosdep (needed for dependency installation):

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` says it has already been initialized, just run `rosdep update`.

## 2. Download the project

Clone the repository using the Git URL shared by the project owner. Replace `<REPOSITORY_URL>` with the real URL:

```bash
cd ~
git clone <REPOSITORY_URL>
```

Enter the workspace directory. For example, if the downloaded repository is named `Autonomous-Delivery-Robot-` and contains `ros2_ws`:

```bash
cd ~/Autonomous-Delivery-Robot-/ros2_ws
```

If your folder name or location is different, adjust the path accordingly. You should see the `src` directory when you run `ls`.

## 3. Install dependencies and build

Run these commands from inside the `ros2_ws` directory:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to delivery_robot_gazebo
source install/setup.bash
```

Wait for the build to finish successfully before continuing.

## 4. Launch the simulation

In Terminal 1, from the `ros2_ws` directory, run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch delivery_robot_gazebo delivery_robot.launch.py
```

Keep this terminal open while the simulation is running.

## 5. Drive the robot with the keyboard

Open Terminal 2 and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Autonomous-Delivery-Robot-/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

If the project is stored in a different location, update the second `source` path. Keep the terminal focused to send keyboard commands. Use the key guide printed by the teleoperation program; commonly, `i` moves forward, `,` moves backward, `j` turns left, `l` turns right, and `k` stops.

## 6. View the camera (optional)

Open Terminal 3 and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Autonomous-Delivery-Robot-/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

If needed, update the workspace path. In the image viewer, select `/camera/image_raw`.

## Troubleshooting

- **`Package 'delivery_robot_gazebo' not found`:** Make sure the build succeeded and you ran `source install/setup.bash` from the correct workspace.
- **A dependency is missing:** Run `rosdep install --from-paths src --ignore-src -r -y` from the workspace after sourcing ROS 2 Jazzy.
- **Gazebo is already running or fails to start:** Close any old Gazebo simulation terminals, then try launching again.
- **The robot does not move:** Make sure the simulation is running and the keyboard teleoperation terminal is open. Check that the command remaps `cmd_vel` to `/cmd_vel` exactly as shown above.

The `build/`, `install/`, and `log/` folders are generated locally during the build. They do not need to be included in the shared repository.
