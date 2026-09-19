# Extended Kalman Filter — C Implementation

Generic EKF engine in C applied to drone attitude and position estimation via IMU/GPS sensor fusion.

## Results

The EKF estimates the drone's position (blue) by fusing noisy IMU predictions and GPS measurements (red), tracking the true trajectory (black) with sub-meter accuracy.

> Run the simulation and generate the plot : `make run && python3 plot_results.py`

## Architecture

The engine is fully decoupled from the application — swapping the drone model for another system (robot, satellite, vehicle) only requires replacing `imu_model.c` and `sensor_models.c`.

```
src/
├── matrix.c          # Fixed-size matrix algebra (add, mul, transpose, Cholesky inverse)
├── ekf.c             # Generic EKF engine — predict() + update()
├── imu_model.c       # Drone dynamics f(x,u) and numerical Jacobian F
├── sensor_models.c   # GPS / barometer / magnetometer observation models
└── main.c            # Simulation loop — synthetic trajectory + noisy sensors

include/              # Headers
tests/                # Unit tests (matrix operations)
plot_results.py   # 3D trajectory, position error, top-down view
```

## State vector (9×1)

| Index | Variable | Unit |
|-------|----------|------|
| 0–2   | Position x, y, z | m |
| 3–5   | Velocity vx, vy, vz | m/s |
| 6–8   | Attitude φ (roll), θ (pitch), ψ (yaw) | rad |

## EKF cycle

```
IMU  (~200 Hz) ──▶  ekf_predict()   x̂⁻ = f(x̂, u)      P⁻ = F·P·Fᵀ + Q
GPS  (~ 10 Hz) ──▶  ekf_update()    K  = P⁻·Hᵀ·S⁻¹     x̂  = x̂⁻ + K·y
```

## Build & run

```bash
make ekf_drone                     # build simulation
./ekf_drone or make run            # run — outputs results.csv
python3 scripts/plot_results.py    # plot results
make test                          # run matrix unit tests
make clean                         # remove build artifacts
```

**Dependencies** : gcc, make, python3 (matplotlib, pandas)

## Key design choices

- **No dynamic allocation** — fixed-size matrices (`MAX_N = 10`), stack-allocated. Suitable for embedded targets.
- **Cholesky inversion** — exploits the SPD structure of S and P for numerical stability.
- **Numerical Jacobian** — F via finite differences; analytical version can replace it in `imu_model.c`.
- **Sensor-agnostic engine** — `ekf.c` uses function pointers for `f`, `h` and their Jacobians.

## Sensors

| Sensor | Measurement | Frequency | Noise σ |
|--------|------------|-----------|---------|
| IMU (accel + gyro) | Acceleration, angular rate | 200 Hz | 0.1 m/s², 0.01 rad/s |
| GPS | Position x, y, z | 10 Hz | 2.5 m |
| Barometer | Altitude z | 10 Hz | 0.5 m |
| Magnetometer | Yaw ψ | 10 Hz | 0.05 rad |
