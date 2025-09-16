# Quick Reference Guide

## Essential Launch Command Understanding

When you run:
```bash
roslaunch guidance_planner ros1_example.launch
```

### What Starts Up:
1. **Main Planning Node** (`/example`) - Core algorithm execution
2. **RViz** - Visualization of planning results  
3. **rqt_reconfigure** - Real-time parameter tuning GUI
4. **Parameter Server** - Configuration management

### Key Files Involved:
- `launch/ros1_example.launch` - Launch configuration
- `config/params.yaml` - Algorithm parameters  
- `src/ros1_example.cpp` - Main executable
- `rviz/ros1_clean.rviz` - Visualization setup

## Critical Configuration Parameters

### Planning Performance
```yaml
sampling/n_samples: 50     # More = better coverage, slower
homotopy/n_paths: 4        # Number of distinct trajectories
T: 4.0                     # Planning horizon (seconds)
N: 20                      # Time discretization steps
```

### Algorithm Selection
```yaml
homotopy/comparison_function: "Homology"  # Options: Homology, Winding, UVD
dynamics/connections: "Straight"          # Options: Straight, Dubins
spline_optimization/enable: true          # Enable trajectory smoothing
```

### Debug Options
```yaml
debug/output: false        # Console debug information
debug/visuals: false       # Detailed RViz visualizations
```

## System Architecture at a Glance

```
ros1_example.cpp (Main Loop)
    ↓
GlobalGuidance.Update()
    ↓
┌─────────────┬─────────────┬─────────────┬─────────────┐
│     PRM     │ GraphSearch │  Homotopy   │ CubicSpline │
│   Sampling  │   A* Paths  │ Comparison  │ Smoothing   │
└─────────────┴─────────────┴─────────────┴─────────────┘
    ↓
Output: Multiple topologically distinct trajectories
```

## Key Classes and Methods

### GlobalGuidance (Main Interface)
```cpp
// Setup
guidance.SetStart(position, orientation, velocity);
guidance.LoadObstacles(obstacles, static_obstacles);
guidance.SetGoals(goals);

// Planning
bool success = guidance.Update();

// Results
int count = guidance.NumberOfGuidanceTrajectories();
auto& trajectory = guidance.GetGuidanceTrajectory(0);
```

### Essential Data Types
```cpp
// Input
Obstacle(id, position, velocity, dt, N, radius);
Goal(position, cost);

// Output  
OutputTrajectory {
    topology_class,      // Unique ID
    path,               // Waypoint sequence
    spline             // Smooth trajectory
};
```

## ROS Communication

### Topics Published
- `/visualization_marker*` - Planning visualization for RViz
- Debug topics (when enabled)

### Services Available  
- `guidances` - External planning requests
- `select_guidance` - Trajectory selection
- `guidances_Hcost` - Homotopy cost queries

### Dynamic Reconfigure
- Real-time parameter updates through rqt_reconfigure
- Changes take effect immediately

## Common Issues & Solutions

### No Trajectories Found
- **Cause**: Obstacles block all paths
- **Solution**: Increase `n_samples`, extend `T`, or check obstacle predictions

### Planning Too Slow
- **Cause**: Too many samples or complex homotopy
- **Solution**: Reduce `n_samples`, use "Winding" instead of "Homology"

### Jerky Trajectories
- **Cause**: Spline optimization disabled or poor
- **Solution**: Enable `spline_optimization`, adjust smoothness weights

### RViz Shows Nothing
- **Cause**: Visualization not publishing
- **Solution**: Check topic names, enable `debug/visuals`

## Integration Checklist

To use in your robot system:

- [ ] Replace `RandomizeObstacles()` with real sensor data
- [ ] Subscribe to robot state (`/odom`, `/pose`)
- [ ] Subscribe to obstacle information (`/obstacles`, `/scan`)
- [ ] Publish command velocities from computed trajectories
- [ ] Handle planning failures gracefully
- [ ] Tune parameters for your environment/vehicle

## Performance Expectations

**Typical timing** (Intel i7, 50 samples, 4 obstacles, 4 trajectories):
- Total planning time: 60-120ms
- Breakdown: Sampling (25%), Homotopy (40%), Optimization (30%), Other (5%)

**Memory usage**: ~300-500 KB for typical scenarios

**Update rate**: Practical 5-10 Hz, demonstration at 1 Hz

## Quick Parameter Tuning Guide

| Scenario | n_samples | n_paths | comparison_function | Notes |
|----------|-----------|---------|-------------------|-------|
| Real-time | 25 | 2 | Winding | Fastest |
| Balanced | 50 | 4 | Homology | Default |
| High-quality | 100 | 6 | Homology | Best results |
| Debugging | 200 | 8 | Homology | Maximum detail |

## Visualization Legend

**In RViz**:
- **Blue dots**: PRM sample points
- **Black lines**: Graph connectivity
- **Green cylinders**: Obstacle predictions (height = time)
- **Colored 3D curves**: Computed trajectories (different colors = different topology)
- **Grid**: World coordinate reference

---

For complete details, see the comprehensive documentation in the `docs/` folder.