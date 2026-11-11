# Acoustic and robotic experiment acquisition

## Pressure profiles

The current ContactlessCaging research manuscript reports a nominal microphone measurement region of 60 × 60 mm around the acoustic trapping location. Figure 7a shows one-dimensional tabulated pressure-amplitude profiles along the available 0–40 mm scan coordinate. These profiles comprise individual PATs and the simultaneous four-array configuration. Figure 7a marks linearly extrapolated endpoint values for Robots 1 and 2 at 39.5 and 40 mm; those points are different from direct samples.

A usable CSV for `analysis/pressure_profiles.py` has columns:

```csv
scan_position_mm,robot1_pa,robot2_pa,robot3_pa,robot4_pa,four_robots_pa,sample_origin
```

Enter the original microphone readings and mark each row `measured` or `extrapolated`. Keep the acquisition spacing, probe orientation, calibration details and exact scan axis with the source data. Do not infer a full two-dimensional pressure map from the plotted one-dimensional curves.

## Robot trajectories

The paper uses PhaseSpace markers on the PAT boards. For four robots, the plotted formation trajectory is the geometric centre of four board-reference positions:

\[c(t) = \frac{1}{4}\sum_{i=1}^{4}p_i(t)\]

The single-robot comparison uses one tracked PAT-board reference. Each plotted trajectory is represented by 11 samples, while transport success rates are based on 15 independent trials for each configuration. The reported figures distinguish the commanded 100 cm reference path and individual robot/formation trajectories from the unmeasured instantaneous EPS particle position.

Create a CSV with measured data:

```csv
x_cm,y_cm
```

Compute deviations from the paper's straight reference y = 40 cm with:

```bash
python analysis/transport_metrics.py path_to_measured_trajectory.csv --reference-y-cm 40
```

For the four-robot centre, first align timestamped PAT-board positions and compute their mean x and mean y. The output script computes mean absolute deviation, maximum absolute deviation and RMS lateral error.

## Trial reporting

A full-path transport is a success only when the 2 mm EPS particle remains levitated throughout the prescribed path and reaches the destination without falling. The revised paper reports 12 successful four-robot trials out of 15 (80%) and 14 successful single-robot trials out of 15 (approximately 93%). The nominal 20 s reference-duration estimate is calculated as 100 cm / (5 cm/s).

Retain separate files for observed trial outcomes, PAT-board trajectory samples, pressure measurements, acquisition timestamps and oscilloscope offset measurements. Record the files' acquisition dates and apparatus configurations.
