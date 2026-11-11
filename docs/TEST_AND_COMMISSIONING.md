# Source checks and physical commissioning

## Source package test report

The builder, source changes, pure PID mathematics, legacy PhaseSpace packet parser and CSV metrics have been exercised with a local Python test suite. The included tests cover the coordinate transformation handedness, per-board packet indexing edit, manuscript references and controller arithmetic.

Run after assembling the complete repository:

```bash
python -m pytest -q tests/
```

The ROS 2 package has additional tests under `ros2_ws/src/contactlesscaging_control/test/`, executable after installing ROS 2 Humble and the package:

```bash
cd ros2_ws
colcon test --packages-select contactlesscaging_control
colcon test-result --verbose
```

## Commissioning procedure

1. Build the C++ solution with the selected Windows OpenCL/clBLAS development environment.
2. Confirm each board uses a unique ID and its corresponding UDP address, hardware packet and calibration array.
3. Use calibration markers to record four board centres and normals, and check transforms in the declared world frame.
4. Inspect the individual 128-byte phase packets and FPGA timing on lab equipment with acoustic emissions inhibited.
5. Inspect PhaseSpace messages to confirm units, heading convention and stable tracker-to-robot identity mapping.
6. Run the ROS 2 controller in disabled state; confirm `cmd_vel` zero and inspect emergency-stop controls.
7. Apply low-speed individual robot movements first, checking leader reversed motion and follower heading.
8. Check the relative formation offsets using real physical dimensions to avoid collisions.
9. Turn on the acoustic subsystem only after the robot motion, PAT geometry and IR timing are prepared.
10. Record code revisions, controller gains, board IDs, environment, firmware, data recordings and test notes for the research release.

Do not treat the repository's historical keyboard-controlled acoustic example as the complete automated four-TurtleBot3 experiment. The physical experiment requires the apparatus-specific FPGA/ESP32 firmware and integration settings described in the manuscript.
