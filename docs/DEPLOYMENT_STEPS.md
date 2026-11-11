# Step-by-step setup, run and GitHub publication

These steps apply to the assembled ContactlessCaging directory created by `assemble_repository.py`. Perform acoustic and robot operations under laboratory supervision with the physical emergency-stop system available.

## Part A — Prepare the full repository on Windows

1. Install **Git for Windows** and **Python 3.10+**.
2. Extract `ContactlessCaging_Repository_Revision.zip` to a convenient folder, such as `C:\Research\ContactlessCaging_Repository_Revision`.
3. Open PowerShell in that folder.
4. Execute:

   ```powershell
   python assemble_repository.py --clone --output ContactlessCaging_Revised --zip
   ```

   This retrieves all source files, images and videos from the original public GitHub repository, applies the C++ project corrections and copies the ROS 2 and analysis resources.
5. Check the generated directory `ContactlessCaging_Revised` and inspect `README.md`.
6. The assembler also creates `ContactlessCaging_Revised_Complete_Source.zip`; generated Visual Studio build artefacts are removed from the revision.

## Part B — Build the Windows acoustic project

7. Install Visual Studio 2022 with *Desktop development with C++*, x64 MSVC v143 and the Windows SDK.
8. Install OpenCL development libraries/runtime and clBLAS that match your GPU/driver.
9. Open `ContactlessCaging_Revised\ContactlessCaging_Code\WirelessBoard\WirelessBoard.sln`.
10. Select x64. Configure library include/link paths to local installations, then build the solver, UDP library and main project.
11. Ensure `board_mini.pat`, `ip_addresses.pat` and the OpenCL kernel are accessible from the application's working directory.
12. Confirm board IDs, 10 cm nominal opposing-PAT separation, transforms and 128-byte board-packet mapping against the actual apparatus.

## Part C — Install ROS 2 on Ubuntu 22.04

13. Install ROS 2 Humble and the TurtleBot3 ROS 2 packages according to the official distributions applicable to your computer.
14. From the aligned repository:

   ```bash
   source /opt/ros/humble/setup.bash
   sudo apt update
   sudo apt install -y python3-colcon-common-extensions python3-yaml \
      ros-humble-rclpy ros-humble-geometry-msgs ros-humble-std-srvs ros-humble-launch-ros
   cd ros2_ws
   colcon build --symlink-install --packages-select contactlesscaging_control
   source install/setup.bash
   ```
15. Edit `ros2_ws/src/contactlesscaging_control/config/formation.yaml` with the measured global formation offsets and calibrated tracker units.
16. Launch PhaseSpace pose publishing. For the historic binary UDP packet format, run:

   ```bash
   ros2 run contactlesscaging_control phasespace_udp_bridge --ros-args -p udp_port:=7777
   ```
17. Confirm the pose topics with `ros2 topic echo /phasespace/robot1/pose` and repeat for each robot. Confirm data freshness and units before enabling robot movement.
18. On each robot's Raspberry Pi, run its independent controller:

   ```bash
   ros2 launch contactlesscaging_control one_robot.launch.py robot_name:=robot1
   ```

   Change `robot1` to the corresponding robot on each computer.
19. On the leader or a coordination computer, start the reference publisher:

   ```bash
   ros2 run contactlesscaging_control transport_reference
   ```
20. Inspect controller outputs with ultrasound disabled. Confirm backward leader and forward follower behaviour, speed limits, and emergency-stop operation.

## Part D — Execute a controlled trial

21. Configure the original acoustic FPGA/ESP32 system and the correct four-board phase input. Confirm the 40 kHz common timing reference and acoustic alignment using the proper measurement instruments.
22. Enable each robot's controller:

   ```bash
   ros2 param set /robot1/formation_controller enabled true
   ros2 param set /robot2/formation_controller enabled true
   ros2 param set /robot3/formation_controller enabled true
   ros2 param set /robot4/formation_controller enabled true
   ```
23. Start the commanded leader reference:

   ```bash
   ros2 service call /formation/start std_srvs/srv/Trigger '{}'
   ```
24. Record PhaseSpace pose/robot odometry with `ros2 bag record`, plus the apparatus's acoustic command, synchronisation and trial-outcome channels.
25. Stop by disabling the controllers and switching off acoustic outputs. Store experiment identifiers with the data.

## Part E — Regenerate metrics from collected CSV files

```bash
python analysis/transport_metrics.py measured_trajectory.csv --reference-y-cm 40
python -m pip install matplotlib
python analysis/pressure_profiles.py pressure_measurements.csv --output acoustic_pressure.png
```

## Part F — Commit the assembled repository

26. On your computer with your GitHub account authenticated:

   ```bash
   cd ContactlessCaging_Revised
   git switch -c revision/contactlesscaging-code-cleanup
   git status
   git diff --check
   git add -A
   git diff --cached --check
   git commit -m "Revise ContactlessCaging code and remove generated build files"
   git push -u origin revision/contactlesscaging-code-cleanup
   ```
27. Open your GitHub repository and create a pull request from `revision/contactlesscaging-code-cleanup` into `main`. Review the source diff and merge using your GitHub account.
28. Create a tagged release from the selected commit, archive it with Zenodo and add the assigned code DOI to the paper's Code Availability statement.

## Troubleshooting

- `ModuleNotFoundError: rclpy`: source `/opt/ros/humble/setup.bash` and the workspace `install/setup.bash` on Ubuntu 22.04.
- ROS 2 controller has no motion: check `enabled`, `/phasespace/robotN/pose`, global frames, topic namespaces and target presence.
- The Windows C++ project cannot link: correct the OpenCL, clBLAS and dependent `.lib` paths and ensure all projects use the same x64 toolchain.
- Acoustic PAT appears at incorrect focal positions: check transform convention, real 5 cm PAT offsets, board mapping, phase-delay calibration and UDP packet assignment.
- PhaseSpace messages have wrong units: configure the `position_scale` and `heading_unit` parameters to match the tracking stream.
