# ContactlessCaging
### Collective Contactless Transport via Distributed Acoustic Caging in a Physical Robot Swarm

![System overview](ContactlessCaging_Images/BlockDiagram.png)

**Research code, documentation, and integration tools for ContactlessCaging, a four-robot acoustic transport project.** ContactlessCaging uses four mobile ultrasonic phased-array robots to generate and transport a shared acoustic trapping region without contact with the payload [1]. The present study extends the *AcoustoBots* platform [2] and the two-robot IROS 2025 transport study [3].

## 1. Overview

The experimental ContactlessCaging platform consists of four TurtleBot3 Burger robots. Each robot carries an 8 × 8 ultrasonic phased array (PAT) with 64 transducers operating at 40 kHz. The boards face a common trapping region, giving 256 elements in the four-array configuration. The architecture combines:

- **Acoustic actuation:** element-wise joint GS-PAT phase control with calibrated board geometry.
- **Robot coordination:** ROS 2 Humble, independent local PID controllers, a designated leader and three followers.
- **Pose sensing:** external PhaseSpace tracking of PAT-board positions.
- **Timing:** a 40 kHz reference distributed from leader to followers via FPGA–IR electronics.
- **Evaluation:** microphone pressure-amplitude profiles, PhaseSpace-based trajectories, full-path transport trials and synchronisation measurements.

The control architecture is **distributed at robot level but uses global external pose sensing and a leader reference**; it does not represent a leader-independent, fully onboard autonomous swarm.

### Experimental results documented in the research manuscript

| Metric | Reported value |
| --- | --- |
| Mobile robots | 4 × TurtleBot3 Burger |
| Phased arrays | 4 × (8 × 8), 256 transducer elements |
| Acoustic carrier | 40 kHz |
| Payload | 2 mm-diameter EPS particle |
| Commanded path / nominal speed | 100 cm straight line / approximately 5 cm/s |
| Successful four-robot trials | 12 of 15 (80%) |
| Successful single-robot reference trials | 14 of 15 (approximately 93%) |
| Four-array measured peak pressure amplitude | Approximately 8026.4 Pa |
| Four-robot formation-centre lateral RMSE | 1.386 cm (11 sampled trajectory positions) |
| Maximum observed inter-robot timing offset | Approximately 1.9 µs |

The nominal 20 s traversal duration is calculated from commanded distance and speed rather than measured trial timestamps. The tracked PAT-board reference trajectories are different from the trajectory of the suspended particle.

## 2. Repository contents

```text
ContactlessCaging/
├── ContactlessCaging_Code/
│   └── WirelessBoard/
│       ├── WirelessBoard.sln                          # Visual Studio solution
│       ├── WirelessBoard/
│       │   ├── AcoustoBots_GSPAT_FocalPoint_Hinge_Bead_Cooperative_Four.cpp
│       │   ├── WirelessBoard.vcxproj
│       │   ├── board_mini.pat
│       │   └── ip_addresses.pat
│       ├── AsierInhoUDP/                             # PAT UDP driver
│       └── GSPAT_SolverV2/                           # C++/OpenCL GS-PAT solver
├── ContactlessCaging_Images/
├── ContactlessCaging_Videos/
├── ros2_ws/
│   └── src/contactlesscaging_control/                # ROS 2 support package
│       ├── contactlesscaging_control/
│       │   ├── control_math.py
│       │   ├── formation_controller.py
│       │   ├── phasespace_udp_bridge.py
│       │   └── transport_reference.py
│       ├── launch/
│       └── config/formation.yaml
├── analysis/
│   ├── transport_metrics.py
│   └── pressure_profiles.py
├── docs/
│   ├── ARCHITECTURE.md
│   ├── ACQUISITION_AND_DATA.md
│   ├── DEPLOYMENT_STEPS.md
│   └── SOURCE_PROVENANCE.md
├── tests/
├── .gitignore
└── README.md
```

The existing acoustic C++ source is retained, including historical AcoustoBots programmes, while temporary IDE and build outputs are removed from the active tree. The included Python ROS 2 package supplies additional formation-control interfaces and a parser for the tracking UDP packet used in the earlier C++ source. The actual low-level FPGA–IR signal generation remains a separate hardware/firmware subsystem.

## 3. Hardware requirements

| Hardware | Role |
| --- | --- |
| 4 × TurtleBot3 Burger, Raspberry Pi 4B | Differential-drive locomotion and ROS 2 coordination |
| 4 × 8 × 8 PAT boards, Murata MA40S4S | 40 kHz acoustic-field emission |
| Adafruit ESP32 Feather V2 | Communication and acoustic driving interface |
| Altera Cyclone IV FPGA and IR TX/RX | Phase-timing reference distribution |
| PhaseSpace tracking system | Global PAT-board pose feedback |
| Laboratory control computer with OpenCL-compatible GPU | Windows C++ GS-PAT acoustic software |
| Brüel & Kjær microphone and XYZ positioning stage | Pressure profiles |
| Oscilloscope | Inter-robot timing measurements |
| 2 mm EPS sphere | Contactless-transport specimen |

**Physical configuration:** The four PAT normals point horizontally inward, towards the common target. The study describes a nominal 5 cm PAT-surface-to-target distance. Check the actual transducer-to-world transforms, mounting geometries, board IDs and local network addresses against the apparatus before energising the PATs.

## 4. Software dependencies

### Acoustic C++/OpenCL project — Windows development computer

- Windows 10/11 x64 and **Visual Studio 2022** with **MSVC v143** and the Windows SDK (from the checked-in `.vcxproj`).
- Compatible **OpenCL** runtime/headers/libraries and **clBLAS** for the GS-PAT solver.
- WinSock2, Visual C++ runtime, `AsierInhoUDP` library and `GSPAT_SolverV2` library.
- `board_mini.pat`, `ip_addresses.pat` and `hologramSolver_V2.cl` in the configured runtime working directory.

Existing Visual Studio project properties refer to laboratory-specific paths. Change these to paths on the target computer. Library versions depend on the selected OpenCL SDK and driver.

### Robot coordination — Ubuntu and ROS 2

- Ubuntu 22.04 LTS and **ROS 2 Humble** on the Raspberry Pi 4B / ROS workstation.
- `rclpy`, `geometry_msgs`, `std_srvs`, `launch_ros`, `ament_index_python`, `python3-yaml` and `colcon`.
- PhaseSpace pose stream: `geometry_msgs/PoseStamped` in metres and a common `world` frame, or the supplied bridge for the historical tracker UDP packet format.
- TurtleBot3 ROS 2 bring-up and accessible `/<robot>/cmd_vel` control topics.

The supplied ROS 2 controller gains and relative positions are starting configuration examples; replace them with laboratory-calibrated values before commanding motion.

## 5. Build the acoustic software (Windows)

1. Clone the repository (or download the assembled repository archive) and open `ContactlessCaging_Code\WirelessBoard\WirelessBoard.sln`.
2. In Visual Studio, select **x64** and the desired configuration (`Debug` or `Release`).
3. Set the OpenCL and clBLAS header/library paths, and build the `GSPAT_SolverV2` and `AsierInhoUDP` library projects.
4. Build the `WirelessBoard` application. The corrected project selects `AcoustoBots_GSPAT_FocalPoint_Hinge_Bead_Cooperative_Four.cpp`, which configures four PAT boards and 256 transducers.
5. Configure the working directory so the `.pat` files and OpenCL kernel are found. Confirm the real IP addresses, IDs, calibration and inward-facing board transforms.
6. Operate the acoustic programme only after checking the four individual phase packets with the FPGA/ESP32 hardware protocol.

The four-array C++ application also contains a keyboard-operated focal-point demonstration (`W/A/S/D` and `Q/E`). Its legacy `F/B/L/R` network commands target an earlier robot interface; the TurtleBot3 motion workflow uses the ROS 2 controllers described below.

## 6. Build the ROS 2 package (Ubuntu 22.04)

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-yaml \
    ros-humble-rclpy ros-humble-geometry-msgs ros-humble-std-srvs \
    ros-humble-launch-ros
cd ros2_ws
colcon build --symlink-install --packages-select contactlesscaging_control
source install/setup.bash
```

Check the available interfaces:

```bash
ros2 pkg executables contactlesscaging_control
ros2 topic list
```

For one controller on each Raspberry Pi:

```bash
# Robot 1 computer (repeat for robot2, robot3, robot4 on their own computers)
ros2 launch contactlesscaging_control one_robot.launch.py robot_name:=robot1
```

For a single-machine bench session with four controller instances:

```bash
ros2 launch contactlesscaging_control four_robots.launch.py
```

If the tracking host streams the historical PhaseSpace UDP packet used by the older C++ programme, start the UDP bridge:

```bash
ros2 run contactlesscaging_control phasespace_udp_bridge --ros-args \
    -p udp_port:=7777 -p frame_id:=world -p position_scale:=1.0
```

Adjust the position scale, heading units and tracker-name mapping using `config/formation.yaml`. Consult `docs/DEPLOYMENT_STEPS.md` for the complete execution sequence.

## 7. Contactless transport execution sequence

1. Set the four robot names, PAT-board calibrations, geometry, PhaseSpace trackable names and ROS 2 network configuration.
2. Start the PhaseSpace stream and inspect all four `/phasespace/robotN/pose` topics in the same global frame.
3. Bring up the TurtleBot3 ROS 2 nodes with one robot namespace per robot.
4. Launch one formation-controller process per robot, using the measured global formation offsets. Controllers initialise in the **disabled** state.
5. Inspect a zero-command condition and apply small safe commands to confirm motion direction and `cmd_vel` routing. Enable the individual controllers as appropriate.
6. Start the leader's 100 cm commanded reference at approximately 5 cm/s using the `/formation/start` service.
7. Configure the PAT phase generation, common FPGA–IR 40 kHz timing reference and acoustic target according to the experimental apparatus, then perform safe acoustic and transport tests.
8. Record robot pose, odometry, acoustic commands, timing traces and trial outcomes; stop controllers and disable transducer power at the end of the run.

Example motion enable/start commands:

```bash
ros2 param set /robot1/formation_controller enabled true
ros2 param set /robot2/formation_controller enabled true
ros2 param set /robot3/formation_controller enabled true
ros2 param set /robot4/formation_controller enabled true
ros2 service call /formation/start std_srvs/srv/Trigger '{}'
```

Emergency software stop for each node: `ros2 param set /robotN/formation_controller enabled false`. Use the hardware emergency stop where required.

## 8. Data analysis and figures

- `analysis/transport_metrics.py` calculates mean lateral deviation, maximum lateral deviation and lateral RMSE from measured PAT-board positions (`x_cm,y_cm` CSV).
- `analysis/pressure_profiles.py` plots measured one-dimensional microphone pressure profiles from CSV and identifies extrapolated samples separately.
- In the research manuscript, the acoustic measurement procedure considered a nominal **60 × 60 mm** region, while Figure 7a contains available 1D profiles along the **0–40 mm** scan coordinate; the graph does not constitute a complete 2D map.
- The four-robot trajectory is the geometric mean of PAT-board reference positions (from the PhaseSpace system), and must not be treated as direct EPS-particle position tracking.

Input schemas and interpretation details are documented in `analysis/README.md` and `docs/ACQUISITION_AND_DATA.md`. Underlying experimental measurements should retain their original acquisition provenance and sample annotations.

## 9. Limitations and scope

The paper concerns a laboratory proof-of-concept with a single 2 mm EPS payload, fixed four-robot geometry and approximately 5 cm/s commanded transport along a straight 100 cm path. The reported acoustic pressure profiles do not determine three-dimensional radiation forces or trapping stiffness. Particle displacement relative to the acoustic trap centre was not recorded independently. Other payloads, turning paths and speed-dependent reliability remain subjects for further experiments.

The repository preserves the existing Windows C++ acoustic code and supplies ROS 2 support code and analysis utilities. Hardware-specific FPGA/ESP32 firmware, PhaseSpace stream calibration, acoustic-to-robot streaming integration and original experiment logs should be maintained alongside the experimental apparatus and recorded configuration. Source changes require bench-level testing prior to physical transport runs.

## 10. Code availability and release

Original research sources: [github.com/narsimlukemsaram/ContactlessCaging](https://github.com/narsimlukemsaram/ContactlessCaging).

For a manuscript release, tag the selected GitHub commit and create a DOI-assigning archive (e.g. Zenodo). Cite the resulting version DOI in the paper's Code Availability statement when assigned. The code and datasets should be archived with appropriate third-party attributions and permissions.

## References and how to cite

[1] N. Kemsaram and S. Subramanian, "ContactlessCaging: Collective Contactless Transport via Distributed Acoustic Caging in a Physical Robot Swarm". 

[2] N. Kemsaram, A. Delibasi, J. Hardwick, B. Gautam, D. M. Plasencia and S. Subramanian, "AcoustoBots: A swarm of robots for acoustophoretic multimodal interactions," Frontiers in Robotics and AI, 12, 1537101. https://doi.org/10.3389/frobt.2025.1537101.

[3] N. Kemsaram, A. Delibasi, J. Hardwick, B. Gautam, D. M. Plasencia and S. Subramanian, "A Cooperative Contactless Object Transport with Acoustic Robots," 2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hangzhou, China, 2025, pp. 18043-18050, doi: 10.1109/IROS60139.2025.11245907. 

[4] D. M. Plasencia, Hirayama, R., Montano-Murillo, R. and S. Subramanian, "GS-PAT: High-speed multi-point sound-fields for phased arrays of transducers," ACM Transactions on Graphics, 39(4), Article 138.
