# Analysis scripts

`transport_metrics.py` reads a measured PAT-board trajectory CSV with `x_cm,y_cm` and calculates mean absolute lateral deviation, maximum deviation and lateral RMSE against the reference `y = 40 cm` by default.

`pressure_profiles.py` plots measured 1D pressure-amplitude profiles from a supplied microphone CSV. It keeps measured and extrapolated samples visually distinguishable.

These scripts contain no hard-coded experimental measurements. Supply the original recorded files in the specified CSV schema and document acquisition conditions. For the four-robot centre, calculate the average of the four PAT-board reference positions at each timestamp before running the trajectory script.

```bash
python analysis/transport_metrics.py data/robot_trajectory.csv --reference-y-cm 40
python analysis/pressure_profiles.py data/pressure_profiles.csv --output figure7a.png
```
