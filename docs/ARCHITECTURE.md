# ContactlessCaging system architecture

## Acoustic actuation

The Windows-side four-array example uses `numPatBoards = 4` and `GSPAT_V2::createSolver(256)`. The transducer locations are read from the 64-element PAT configuration, mapped into world coordinates by per-board 4 × 4 transforms and supplied to the common GS-PAT solver. The patch selects a separate 128-byte UDP packet for each PAT and uses consistent rigid transforms (rotation matrix determinant +1 and fourth-column translation) with the existing row-major driver.

The physical FPGA–IR network distributes a 40 kHz timing reference from the leader. The corresponding firmware and calibrated transducer positions belong to the laboratory's acoustic-hardware configuration.

## Four robot motion

The included `contactlesscaging_control` ROS 2 package provides:

- `formation_controller`: a separate ROS process per robot, with Cartesian position PID, signed forward/backward differential-drive conversion, speed limits, stale-pose gating and a disabled-by-default motor gate.
- `transport_reference`: publishes a world-frame 100 cm straight-line leader target at 0.05 m/s once `/formation/start` is called.
- `phasespace_udp_bridge`: parses the existing 32-byte-name / 112-byte-state binary PhaseSpace UDP records and publishes world-frame `PoseStamped` samples.

The leader executes backward motion; the followers execute forward motion relative to its current measured position plus global formation offsets. All four controllers run independently, and can be deployed on their respective Raspberry Pi 4B computers.

## ROS topics

```text
PhaseSpace (legacy UDP tracking: port 7777)
           |
           v
  phasespace_udp_bridge (or native PoseStamped pose publisher)
           |
           +-- /phasespace/robot1/pose ---> robot1/formation_controller ---> /robot1/cmd_vel
           +-- /phasespace/robot2/pose ---> robot2/formation_controller ---> /robot2/cmd_vel
           +-- /phasespace/robot3/pose ---> robot3/formation_controller ---> /robot3/cmd_vel
           +-- /phasespace/robot4/pose ---> robot4/formation_controller ---> /robot4/cmd_vel
           |
transport_reference -- /formation/leader_target --> leader controller
```

**Reference-frame assumptions:** positions in metres, headings in radians unless configured otherwise; all robot pose messages share the calibrated `world` frame. Determine the exact world-frame offsets from the original PAT-board geometry and robot poses.

## Acoustic and motion coordination

The provided C++ acoustic keyboard programme and the ROS 2 motion controllers are distinct executables. For simultaneous experimental control, the apparatus requires its original acoustic-phase target-distribution logic (including actual PAT-board poses and phase-update scheduling), plus the FPGA–IR timing and ESP32 driver firmware. Keep those interfaces and update rates consistent with the original experiment.

## Safety controls

The motion controllers generate a zero `Twist` when disabled or when required pose input is stale. The default configuration sets `enabled: false`, and the motor enable step should occur only after checking commands against the physical robot orientation and the site emergency-stop procedure.
