# Source provenance and correspondence to the manuscript

The source files under `ContactlessCaging_Code/WirelessBoard` originate from the GitHub `narsimlukemsaram/ContactlessCaging` repository. The original C++ OpenCL solver, calibration interfaces, UDP transport and AcoustoBots examples are retained. The four-array source and Visual Studio project receive focused source corrections via the accompanying builder.

The `ros2_ws/src/contactlesscaging_control` support package and `analysis/` tools are newly prepared supplemental integration and processing code; they are not described as recovered copies of the exact original deployment or measurement scripts. Retain provenance of the original ROS 2 source, FPGA–IR and ESP32 firmware when depositing a manuscript-specific release.

## Source-level changes

1. The four-array C++ programme selects 4 PATs and 256 transducers, using an indexed board-message offset rather than repeating the first board's packet.
2. The four 4 × 4 board-to-world transforms use fourth-column translations and right-handed rotations aligned with the C++ driver's row-major multiplication.
3. The Visual Studio `WirelessBoard` target selects the `_Four.cpp` programme; historic `_Two.cpp` and other sources remain as supporting work.
4. The UDP driver's `addClient()` accepts constant IP address strings provided by the four-array source.
5. The README describes the distributed robot-level architecture accurately and cites related work.

## Integration boundaries

The existing Windows acoustic programme exposes keyboard target updates and legacy robot UDP commands. The ROS 2 support package exposes PhaseSpace pose inputs and independent TurtleBot3 motion commands. The experiment's apparatus-specific acoustic target streaming and FPGA timing/ESP32 firmware should be supplied from the original research system for a combined deployment.

## Experiment-traceability items

Record the code commit, software libraries, PAT-board IDs, calibration transforms, ROS 2 parameter values, PhaseSpace coordinate frame, FPGA/ESP32 firmware references, raw measurements, and dataset/plot-processing scripts with the archive.
