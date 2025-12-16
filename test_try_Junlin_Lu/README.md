Junlin Lu
## LiDAR Integration and Obstacle-Aware Navigation (Engineering Attempt)

This project includes an explicit attempt to integrate a 2D LiDAR sensor into a
grid-based global path planning pipeline in Webots.

### LiDAR Initialisation
A forward-facing 2D LiDAR is enabled at controller startup using the Webots API.
Basic sensor parameters (horizontal resolution, field of view, minimum and maximum
range) are printed at runtime to verify correct configuration and data availability.

The LiDAR provides range measurements in polar form, which are sampled at each
control step.

### Sector-Based Distance Processing
To obtain a lightweight and interpretable representation of the environment,
the LiDAR scan is divided into three angular sectors:
- Left
- Centre
- Right

For each sector, the minimum valid range is extracted while filtering out
non-finite values and very short readings caused by self-reflections or noise.
This produces a compact (left, centre, right) distance triplet that can be
logged and analysed in real time.

### Integration with Global Path Planning
The global planner generates a sequence of waypoints based on a predefined grid map.
During navigation, the planner outputs ideal wheel velocities that drive the robot
towards the next waypoint.

These velocities are then passed through a local obstacle avoidance layer that:
- Reduces forward speed when obstacles are within a slowing distance,
- Introduces a steering bias towards the freer side when the front sector is close,
- Temporarily overrides planner commands in emergency proximity situations.

In addition, an experimental feedback loop is implemented where LiDAR measurements
below a blocking threshold are used to estimate the position of newly observed
obstacles and insert them into the grid map. This mechanism is coupled with a
re-planning cooldown to avoid continuous re-planning at every control step.

### Scope and Intent
The purpose of this implementation is not to provide a complete industrial-grade
navigation stack, but to explore the practical challenges of fusing reactive
obstacle avoidance with global path planning in a structured environment.

The resulting behaviour and limitations (e.g. oscillation in narrow passages,
difficulty rejoining the planned route) are reproducible and form part of the
engineering analysis presented in this repository.
