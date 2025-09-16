# Comprehensive Guidance Planner System Guide

## Table of Contents
1. [System Overview](#system-overview)
2. [Launch Sequence Breakdown](#launch-sequence-breakdown) 
3. [Configuration Parameters](#configuration-parameters)
4. [Architecture and Components](#architecture-and-components)
5. [Algorithm Workflow](#algorithm-workflow)
6. [Homotopy Comparison Functions](#homotopy-comparison-functions)
7. [Graph Building and Propagation](#graph-building-and-propagation)
8. [ROS Communication](#ros-communication)
9. [Data Structures](#data-structures)
10. [Code Flow Examples](#code-flow-examples)
11. [Beginner's Guide](#beginners-guide)

---

## System Overview

The Guidance Planner is a **sampling-based global motion planner** that computes **topology-distinct trajectories** in 2D dynamic environments. When you run `roslaunch guidance_planner ros1_example.launch`, you're starting a complete motion planning pipeline that:

1. **Samples points** in 3D space-time (x, y, t)
2. **Builds a visibility graph** (Visibility-PRM) connecting collision-free points
3. **Searches for multiple paths** through the graph to reach goals
4. **Classifies paths by topology** using homotopy comparison functions
5. **Optimizes trajectories** using cubic splines
6. **Visualizes results** in RViz for analysis

### Key Innovation
Unlike traditional planners that find a single "best" path, this planner finds **multiple topologically distinct paths** - meaning each path navigates around obstacles in fundamentally different ways (e.g., passing left vs right of an obstacle).

---

## Launch Sequence Breakdown

### What Happens When You Run the Launch File

```bash
roslaunch guidance_planner ros1_example.launch
```

#### Step 1: Parameter Loading
```xml
<rosparam command="load" file="$(find guidance_planner)/config/params.yaml"/>
<param name="clock_frequency" value="20"/>
```
- Loads all configuration parameters from `config/params.yaml`
- Sets the main control loop frequency to 20 Hz

#### Step 2: Main Node Launch
```xml
<node pkg="guidance_planner" type="example" name="example" respawn="false" output="screen"/>
```
- Starts the main executable built from `src/ros1_example.cpp`
- Node name: `/example`
- Output goes to screen for debugging

#### Step 3: Configuration GUI
```xml
<node pkg="rqt_reconfigure" type="rqt_reconfigure" name="rqt_reconfigure" respawn="false"/>
```
- Launches dynamic reconfigure GUI for real-time parameter tuning

#### Step 4: Visualization
```xml
<node name="rviz" pkg="rviz" type="rviz" args="-d $(find guidance_planner)/rviz/ros1_clean.rviz" output="log"/>
```
- Starts RViz with pre-configured visualization settings
- Shows the planning graph, obstacles, and computed trajectories

---

## Configuration Parameters

### Core Planning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `T` | 4.0 | **Time horizon in seconds** - How far into the future to plan |
| `N` | 20 | **Discrete time steps** - Number of time slices in the planning horizon |
| `seed` | 1 | **Random seed** for reproducible results (-1 = random) |

### Homotopy Classification

| Parameter | Default | Description |
|-----------|---------|-------------|
| `homotopy/n_paths` | 4 | **Number of distinct trajectories** to compute |
| `homotopy/comparison_function` | "Homology" | **Which topology comparison to use**: Homology, Winding, or UVD |
| `homotopy/use_learning` | false | Enable machine learning for path selection |

### Sampling Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sampling/n_samples` | 50 | **Maximum samples per iteration** - More samples = better coverage but slower |
| `sampling/timeout` | 10 | **Sampling timeout in milliseconds** |
| `sampling/margin` | 5.0 | **Safety margin around goals** for sampling |

### Motion Constraints

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_velocity` | 3.0 | **Maximum allowed velocity** (m/s) |
| `max_acceleration` | 3.0 | **Maximum allowed acceleration** (m/s²) |
| `dynamics/connections` | "Straight" | **Path type**: "Straight" or "Dubins" (for car-like robots) |
| `dynamics/turning_radius` | 0.305 | **Minimum turning radius** for Dubins paths (m) |

### Spline Optimization

| Parameter | Default | Description |
|-----------|---------|-------------|
| `spline_optimization/enable` | true | **Enable trajectory smoothing** |
| `spline_optimization/geometric` | 25.0 | **Weight for geometric smoothness** |
| `spline_optimization/smoothness` | 10.0 | **Weight for velocity/acceleration smoothness** |
| `spline_optimization/collision` | 0.5 | **Weight for collision avoidance** |

### Visualization Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug/visuals` | false | **Show detailed debug visualizations** |
| `visuals/transparency` | 0.97 | **Obstacle transparency** (0=opaque, 1=invisible) |
| `visuals/visualize_all_samples` | false | **Show all PRM samples** (can be overwhelming) |

---

## Architecture and Components

### Core Classes and Their Roles

#### 1. GlobalGuidance (Main Orchestrator)
- **File**: `include/guidance_planner/global_guidance.h`
- **Purpose**: High-level interface that coordinates all planning components
- **Key Methods**:
  - `SetStart()`: Set robot's starting position and velocity
  - `LoadObstacles()`: Provide obstacle predictions
  - `SetGoals()` / `LoadReferencePath()`: Define where to plan to
  - `Update()`: Execute one planning iteration
  - `GetGuidanceTrajectory()`: Retrieve computed trajectories

#### 2. PRM (Probabilistic Roadmap)
- **File**: `include/guidance_planner/prm.h`
- **Purpose**: Builds the visibility graph in space-time
- **Key Responsibilities**:
  - Sample collision-free points in (x, y, t) space
  - Connect visible points to form graph edges
  - Maintain graph consistency across time iterations
  - Propagate nodes forward in time

#### 3. Environment (Collision Detection)
- **File**: `include/guidance_planner/environment.h`
- **Purpose**: Handles all collision checking and obstacle management
- **Features**:
  - Collision checking for points and line segments
  - Support for dynamic obstacles with predictions
  - Static obstacle handling (walls, boundaries)
  - Optimized collision checking with spatial grids

#### 4. GraphSearch (Path Finding)
- **File**: `include/guidance_planner/graph_search.h`
- **Purpose**: Finds paths through the PRM graph
- **Algorithm**: A* or Dijkstra-based search from start to goals

#### 5. Homotopy Comparison Classes
- **Files**: `include/guidance_planner/homotopy_comparison/`
- **Purpose**: Classify paths by their topology (how they navigate around obstacles)
- **Three implementations**:
  - **Homology**: H-signature based comparison
  - **Winding**: Winding angle based comparison  
  - **UVD**: U-distance vector based comparison

#### 6. CubicSpline3D (Trajectory Optimization)
- **File**: `include/guidance_planner/cubic_spline.h`
- **Purpose**: Smooths geometric paths into dynamically feasible trajectories
- **Features**:
  - 3D spline optimization (x, y, t)
  - Velocity and acceleration constraints
  - Collision avoidance during optimization

#### 7. Sampler (Point Generation)
- **File**: `include/guidance_planner/sampler.h`
- **Purpose**: Generates sample points for the PRM
- **Sampling strategies**:
  - Uniform random sampling
  - Reference path-guided sampling
  - Goal-biased sampling

---

## Algorithm Workflow

### Main Planning Loop (Update Cycle)

When `guidance.Update()` is called, here's the complete workflow:

#### Phase 1: Data Preparation
```cpp
// In GlobalGuidance::Update()
1. Validate input data (start, goals, obstacles)
2. Configure PRM with current data
3. Reset visualization markers
```

#### Phase 2: Graph Construction
```cpp
// In PRM::Update()
1. Propagate previous nodes forward in time
2. Sample new collision-free points
3. Build visibility connections between points
4. Add start and goal nodes to graph
```

**Detailed PRM Process**:
1. **Node Propagation**: Previous iteration's nodes are moved forward by one time step
2. **Sampling**: Generate `n_samples` random points in free space
3. **Visibility Checking**: For each sample, find which existing nodes it can "see" (collision-free line of sight)
4. **Graph Update**: Add the sample as a new node and create edges to visible neighbors

#### Phase 3: Path Search
```cpp
// In GraphSearch::Search()
1. Run A* search from start to each goal
2. Extract geometric paths from graph
3. Rank paths by cost and feasibility
```

#### Phase 4: Topology Classification
```cpp
// In HomotopyComparison implementations
1. Compute topology signatures for each path
2. Group paths with equivalent signatures
3. Keep only the best path from each group
4. Ensure we have at most n_paths distinct trajectories
```

#### Phase 5: Trajectory Optimization
```cpp
// In CubicSpline3D optimization
1. Convert geometric paths to spline control points
2. Optimize splines for smoothness and feasibility
3. Check collision constraints during optimization
4. Generate final time-parameterized trajectories
```

#### Phase 6: Result Processing
```cpp
// Back in GlobalGuidance
1. Rank trajectories by cost heuristic
2. Assign colors for visualization
3. Store results for retrieval
4. Prepare visualization data
```

---

## Homotopy Comparison Functions

### Why Topology Matters
Traditional planners find one "optimal" path, but in dynamic environments, having multiple fundamentally different options is crucial. If the robot passes **left** around an obstacle vs **right**, these represent different **topological classes**.

### 1. Homology (H-Signature)
- **File**: `src/homotopy_comparison/homology.cpp`
- **Method**: Integrates a 3D vector field around obstacle "loops"
- **Key Insight**: Paths that wrap around obstacles differently will have different H-values
- **Computation**: For each obstacle and path pair:
  ```cpp
  // Simplified concept
  h_value = integral_over_path(obstacle_vector_field)
  // Paths are equivalent if |h_a - h_b| < threshold for all obstacles
  ```

**When Called**:
- During path classification in `PRM::Update()`
- When checking if two paths are equivalent
- To compute topology-based costs between trajectories

**How It Works**:
1. Each obstacle defines a "loop" in 3D space-time
2. The H-signature measures how many times a path "winds around" each obstacle
3. Paths with the same H-signature pattern are topologically equivalent

### 2. Winding Angle
- **File**: `src/homotopy_comparison/winding_angle.cpp`  
- **Method**: Computes cumulative angle changes around obstacles
- **Advantage**: More intuitive geometric interpretation
- **Computation**: Tracks how much the path "turns" around each obstacle center

### 3. UVD (U-Distance Vector)
- **File**: `src/homotopy_comparison/uvd.cpp`
- **Method**: Uses distance vectors to characterize obstacle relationships
- **Advantage**: Computationally efficient for many obstacles
- **Use Case**: Better for environments with many small obstacles

### Selection Logic
The comparison function determines which trajectories are considered "different enough" to keep. The planner will:
1. Generate many candidate paths
2. Classify them using the chosen homotopy function
3. Keep only one representative from each topological class
4. Ensure diversity in the final trajectory set

---

## Graph Building and Propagation

### The Visibility-PRM Graph Structure

#### Nodes (Vertices)
Each node represents a point in 3D space-time:
```cpp
struct Node {
    int id_;                    // Unique identifier
    SpaceTimePoint position_;   // (x, y, t) coordinates
    NodeType type_;            // START, GOAL, GUARD, CONNECTOR
    std::vector<Node*> neighbors_; // Connected nodes
    bool is_active_;           // Still valid in current iteration
}
```

**Node Types**:
- **START**: Robot's current position (t=0)
- **GOAL**: Target positions (t=T)
- **GUARD**: High-quality nodes that persist across iterations  
- **CONNECTOR**: Temporary nodes that help connect guards

#### Edges (Connections)
Edges represent feasible motion between nodes:
- **Visibility**: Collision-free line of sight
- **Dynamics**: Respects velocity/acceleration limits
- **Time**: Later nodes must have higher time values

### Graph Propagation Process

#### Why Propagation is Needed
In dynamic environments, obstacles move over time. The graph must evolve to:
1. **Maintain consistency**: Previously good paths should remain valid if possible
2. **Adapt to changes**: New obstacle positions require graph updates
3. **Efficiency**: Reuse computation from previous iterations

#### Propagation Algorithm
```cpp
// In PRM::PropagateGraph()
1. For each node from previous iteration:
   - Check if still collision-free at new time
   - If valid, create new node at (x, y, t+dt)
   - Maintain topology information if node was part of a path

2. Update node classifications:
   - GUARD nodes become new GUARD nodes (high priority)
   - CONNECTOR nodes become CONNECTOR nodes (may be replaced)

3. Rebuild connections:
   - Check visibility between propagated and new nodes
   - Maintain graph connectivity
```

**Time Evolution**:
- At time t=0: Graph represents current state
- At time t=dt: Nodes are propagated forward, obstacles have moved
- At time t=2*dt: Process repeats, maintaining planning consistency

#### Graph Memory Management
```cpp
// Node lifecycle
Old Graph (t-1) → Propagate → New Graph (t) → Add Samples → Final Graph (t)
```

The system maintains:
- **Active nodes**: Currently valid in the graph
- **Historical data**: Information about previous successful paths
- **Topology memory**: Which paths were previously selected

---

## ROS Communication

### Topics Published

#### Visualization Topics (via ros_tools)
- **Topic**: `/visualization_marker` and `/visualization_marker_array`
- **Purpose**: RViz visualization of planning results
- **Content**:
  - Blue dots: PRM sample points
  - Black lines: Graph edges
  - Colored cylinders: Obstacle predictions
  - Colored 3D lines: Computed trajectories

#### Debug Topics (when enabled)
- **Topic**: `/guidance_planner/debug/*`
- **Purpose**: Detailed internal state information
- **Content**: Intermediate computation results, timing data

### Topics Subscribed

**Note**: The example node doesn't subscribe to topics directly, but in real applications you would typically subscribe to:
- `/odom` or `/robot_pose`: Robot state updates
- `/obstacles` or `/dynamic_obstacles`: Obstacle information
- `/goal` or `/target`: Goal updates

### Services Provided

#### 1. Guidance Planning Service
- **Service**: `guidances.srv`
- **Request**:
  ```
  ObstacleMSG[] obstacles      # Obstacle predictions
  float64 x, y                 # Robot position
  float64 orientation          # Robot heading
  float64 v                    # Robot velocity
  float64[] goals_x, goals_y   # Goal positions
  float64[] static_x, static_y, static_n  # Static obstacles
  int64 n_trajectories         # Requested number of paths
  ```
- **Response**:
  ```
  bool success                 # Planning success flag
  TrajectoryMSG[] trajectories # Computed trajectories
  RightAvoidanceMSG[] h_signature # Topology information
  ```

#### 2. Homotopy Cost Service
- **Service**: `guidances_Hcost.srv`
- **Purpose**: Compute topology cost between trajectory and a reference
- **Use Case**: Evaluate how different a new path is from a baseline

#### 3. Trajectory Selection Service  
- **Service**: `select_guidance.srv`
- **Purpose**: Manually select which trajectory to execute
- **Use Case**: Allow higher-level planners to choose among options

### Messages

#### TrajectoryMSG
```
float64[] x  # X coordinates over time
float64[] y  # Y coordinates over time
```

#### ObstacleMSG
```
float64[] x      # X positions over prediction horizon
float64[] y      # Y positions over prediction horizon  
float64 radius   # Obstacle radius
int32 id         # Unique obstacle identifier
```

#### RightAvoidanceMSG
```
bool[] right_avoidance  # For each obstacle: true if passed on right
```

### Node Communication Pattern

```
┌─────────────────┐    Topics    ┌─────────────────┐
│   Robot State   │ ──────────→  │ Planning Node   │
│   Provider      │              │  (example)      │
└─────────────────┘              └─────────────────┘
                                          │
┌─────────────────┐    Topics    ┌─────────────────┐
│ Obstacle Track  │ ──────────→  │ Visualization   │
│ ing Node        │              │  (RViz)         │
└─────────────────┘              └─────────────────┘
                                          │
┌─────────────────┐   Services   ┌─────────────────┐
│ High-Level      │ ←──────────→ │ Parameter       │
│ Planner         │              │ Server          │
└─────────────────┘              └─────────────────┘
```

---

## Data Structures

### Core Data Types

#### SpaceTimePoint
```cpp
class SpaceTimePoint {
    Vector2d position_;  // (x, y) in 2D space
    double time_;        // t in time
    
    // Key insight: Represents a point in 3D space-time
    // Used for collision checking at specific times
};
```

#### GeometricPath
```cpp
class GeometricPath {
    std::vector<Node> nodes_;     // Sequence of nodes forming the path
    double cost_;                 // Path cost (length + heuristics)
    int topology_class_;          // Homotopy equivalence class ID
    
    // Represents a sequence of connected waypoints through space-time
};
```

#### Obstacle
```cpp
struct Obstacle {
    int id_;                           // Unique identifier
    std::vector<Vector2d> positions_;  // Predicted positions [t0, t1, ..., tN]
    double radius_;                    // Collision radius
    
    // Supports both:
    // 1. Constant velocity: Obstacle(id, start_pos, velocity, dt, N, radius)
    // 2. Full prediction: Obstacle(id, position_vector, radius)
};
```

#### OutputTrajectory
```cpp
struct OutputTrajectory {
    int topology_class;              // Unique topology identifier
    int color_;                      // Visualization color index
    bool previously_selected_;       // Was this chosen in last iteration?
    bool is_new_topology_;          // First time seeing this topology?
    
    StandaloneGeometricPath path;    // Waypoint sequence
    CubicSpline3D spline;           // Smooth trajectory
    
    // Final output: contains both geometric and smooth representations
};
```

### Memory Management and Data Flow

#### During Planning
```
Input Data → PRM Graph → Search Results → Homotopy Classification → Spline Optimization → Output Trajectories
     ↓            ↓              ↓                    ↓                        ↓                    ↓
  Obstacles   Sample Points   Geometric Paths   Topology Classes      Smooth Splines      Final Results
```

#### Between Iterations
```
Previous Results → Node Propagation → Graph Update → New Planning Cycle
       ↓                   ↓              ↓              ↓
  Topology Memory    Persistent Nodes   Updated Graph   Consistent Planning
```

### Configuration Management
```cpp
class Config {
    // All parameters from params.yaml loaded here
    double T;                    // Planning horizon
    int N;                       // Time steps
    int n_samples;               // Sampling count
    std::string comparison_function;  // Homotopy method
    // ... hundreds of other parameters
    
    // Computed values
    double DT = T / N;           // Time step size
    // ... derived parameters
};
```

---

## Code Flow Examples

### Example 1: Complete Planning Cycle

Let's trace what happens when you call `guidance.Update()`:

#### Step 1: Initialization (GlobalGuidance::Update())
```cpp
// File: src/global_guidance.cpp
bool GlobalGuidance::Update() {
    // Clear previous results
    paths_.clear();
    splines_.clear();
    
    // Update PRM with current data
    Graph& graph = prm_.Update();  // ← Key call
```

#### Step 2: PRM Update (PRM::Update())
```cpp  
// File: src/prm.cpp
Graph& PRM::Update() {
    // Propagate nodes from previous iteration
    if (previous_nodes_.size() > 0) {
        PropagateGraph(previous_paths_);
    }
    
    // Sample new points
    SampleNewPoints();  // ← Calls sampler
    
    // Build graph connections
    for (auto& sample : samples) {
        FindVisibleGuards(sample, visible_guards);
        AddSample(sample, visible_guards);
    }
    
    return *graph_;
}
```

#### Step 3: Sampling (Sampler::DrawSample())
```cpp
// File: src/sampler.cpp  
Sample& Sampler::DrawSample(int sample_index) {
    // Generate random point in space-time
    SpaceTimePoint point;
    point.position_(0) = min_(0) + random_generator_.Double() * range_(0); // x
    point.position_(1) = min_(1) + random_generator_.Double() * range_(1); // y  
    point.time_ = random_generator_.Double() * Config::T;                  // t
    
    // Check if collision-free
    bool collision_free = environment_->InCollision(point);
    
    samples_[sample_index] = Sample(point);
    samples_[sample_index].success = collision_free;
    return samples_[sample_index];
}
```

#### Step 4: Visibility Checking (Environment::IsVisible())
```cpp
// File: src/environment.cpp
bool Environment::IsVisible(const SpaceTimePoint& a, const SpaceTimePoint& b) {
    // Check if line segment from a to b is collision-free
    
    // Fast check for constant velocity obstacles
    return IsVisibleRayCast(a, b);  // ← Optimized implementation
}
```

#### Step 5: Path Search (GraphSearch::Search())
```cpp
// File: src/graph_search.cpp
void GraphSearch::Search(const Graph& graph, unsigned int max_paths, 
                        std::vector<Node*>& starts, std::vector<GeometricPath>& paths,
                        const Node* goal) {
    // A* search from start to goal
    priority_queue<Node*> open_set;
    open_set.push(start_node);
    
    while (!open_set.empty()) {
        Node* current = open_set.top();
        open_set.pop();
        
        if (current == goal) {
            // Found path - reconstruct
            GeometricPath path = ReconstructPath(current);
            paths.push_back(path);
        }
        
        // Expand neighbors
        for (Node* neighbor : current->neighbors_) {
            // ... A* logic
        }
    }
}
```

#### Step 6: Homotopy Classification (Homology::AreEquivalent())
```cpp
// File: src/homotopy_comparison/homology.cpp
bool Homology::AreEquivalent(const GeometricPath& a, const GeometricPath& b, 
                           Environment& environment, bool compute_all) {
    // Get cached H-values for both paths
    std::vector<double>& h_a = cached_values_[a];
    std::vector<double>& h_b = cached_values_[b];
    
    auto& obstacles = environment.GetDynamicObstacles();
    
    // For each obstacle, compare H-values
    for (int i = 0; i < obstacles.size(); ++i) {
        double h_diff = std::abs(h_a[i] - h_b[i]);
        if (h_diff > EQUIVALENCE_THRESHOLD) {
            return false;  // Different topology
        }
    }
    
    return true;  // Same topology
}
```

### Example 2: Real-time Loop in ros1_example.cpp

```cpp
// Main control loop
ros::Rate rate(1.0);  // 1 Hz for demonstration
while (!ros::isShuttingDown()) {
    
    // 1. Update obstacles (random or manual)
    RandomizeObstacles(obstacles);
    
    // 2. Load new data into planner
    guidance.LoadObstacles(obstacles, static_obstacles);
    
    // 3. Compute guidance trajectories
    bool success = guidance.Update();  // ← Full planning cycle
    
    if (success) {
        // 4. Retrieve results
        int num_trajectories = guidance.NumberOfGuidanceTrajectories();
        CubicSpline3D& best_trajectory = guidance.GetGuidanceTrajectory(0).spline;
        
        // 5. Use trajectory (in real robot: execute, here: just log)
        RosTools::Spline2D trajectory_2d = best_trajectory.GetTrajectory();
        for (double t = 0; t < Config::T; t += Config::DT) {
            Vector2d pos = trajectory_2d.getPoint(t);
            ROS_INFO("t=%.2f: (%.2f, %.2f)", t, pos(0), pos(1));
        }
    }
    
    // 6. Visualize everything
    guidance.Visualize();
    
    ros::spinOnce();
    rate.sleep();
}
```

---

## Beginner's Guide

### Understanding the Big Picture

If you're new to this repository, start by understanding these key concepts:

#### 1. What is "Topology" in Motion Planning?
Imagine a robot trying to navigate around a table:
- **Path A**: Robot goes around the left side of the table
- **Path B**: Robot goes around the right side of the table  
- **Path C**: Robot goes around the left side, then comes back and goes around the right side

Paths A and B are **topologically different** (different sides). Path C is topologically the same as going around both sides, but inefficient. The planner finds multiple topologically distinct options so the robot can choose based on dynamic conditions.

#### 2. Why 3D Space-Time Planning?
Traditional planners work in 2D (x, y). This planner works in 3D (x, y, t) because:
- **Obstacles move over time**: A position that's blocked now might be free later
- **Timing matters**: Arriving at a location at the wrong time could cause collision
- **Prediction helps**: Knowing where obstacles will be allows better planning

#### 3. The Planning Pipeline Simplified
```
1. Sample random points in space-time that are collision-free
2. Connect nearby points if the path between them is collision-free  
3. Search for paths from start to goal through this graph
4. Group paths by how they navigate around obstacles (topology)
5. Keep the best path from each group
6. Smooth the geometric paths into dynamically feasible trajectories
```

### Getting Started - Hands-On

#### Step 1: Run the Example
```bash
# Terminal 1: Start ROS core (if not running)
roscore

# Terminal 2: Launch the example
roslaunch guidance_planner ros1_example.launch
```

**What you'll see**:
- RViz window with a 2D planning environment
- Blue dots: Sample points in the PRM
- Black lines: Graph connections
- Colored cylinders: Moving obstacles
- Colored 3D curves: Computed trajectories

#### Step 2: Understand the Output
Watch the terminal output:
```
[INFO] Guidance planner found: 4 trajectories
[Best Trajectory]
    [t = 0.00]: (0.00, 0.00)    ← Robot starts here
    [t = 0.80]: (2.40, 0.00)    ← Position after 0.8 seconds  
    [t = 1.60]: (4.80, 0.00)    ← Position after 1.6 seconds
    [t = 2.40]: (7.20, 0.00)    ← And so on...
    [t = 3.20]: (9.60, 0.00)
```

This shows the **best trajectory** sampled at regular time intervals.

#### Step 3: Experiment with Parameters
1. **Open the parameter GUI**: rqt_reconfigure should have opened automatically
2. **Try changing**:
   - `n_paths`: See more/fewer trajectory options  
   - `n_samples`: More samples = better coverage but slower
   - `comparison_function`: Switch between "Homology", "Winding", "UVD"
   - `max_velocity`: See how speed limits affect paths

#### Step 4: Understand the Visualization
In RViz, look for:
- **Grid lines**: Coordinate system reference
- **Blue dots**: PRM sample points (more dots = more sampling)
- **Black lines**: Visibility graph connections
- **Colored cylinders**: Obstacle predictions over time (height = time dimension)
- **Colored 3D lines**: Final trajectory options (different colors = different topology)

### Common Questions

#### Q: Why are there multiple colored trajectories?
**A**: Each color represents a topologically distinct way to reach the goal. For example, red might go left around all obstacles, blue might go right around the first obstacle but left around the second, etc.

#### Q: Why do the obstacles look like cylinders?
**A**: The cylinder represents the obstacle's position over time. The bottom is its current position, and the top is where it will be at the end of the planning horizon (T seconds later).

#### Q: What's the difference between the comparison functions?
**A**: They're different mathematical ways to determine if two paths are "topologically equivalent":
- **Homology**: Uses complex 3D integration (most sophisticated)
- **Winding**: Uses angle calculations (more intuitive)  
- **UVD**: Uses distance relationships (computationally efficient)

#### Q: How do I use this in my own robot?
**A**: Replace the example's obstacle generation with your own:
1. Subscribe to your robot's position (`/odom`, `/pose`, etc.)
2. Subscribe to obstacle information (`/scan`, `/obstacles`, etc.)
3. Call `guidance.SetStart()` with robot state
4. Call `guidance.LoadObstacles()` with obstacle predictions
5. Use `guidance.GetGuidanceTrajectory(0)` as your reference path

#### Q: What if planning fails?
**A**: `guidance.Succeeded()` returns false when no collision-free paths are found. This can happen when:
- Obstacles completely block all paths to goals
- Sampling didn't generate enough points near feasible regions
- Time horizon is too short for the required motion

### Advanced Understanding

Once you're comfortable with the basics, dive deeper into:

1. **Graph Theory**: Study how Visibility-PRM works
2. **Topology**: Learn about homotopy and homology in mathematics  
3. **Optimal Control**: Understand spline optimization
4. **Real-time Systems**: Learn about computational complexity and timing

### Debugging Tips

#### Planning Takes Too Long
- Reduce `n_samples` 
- Reduce `n_paths`
- Check if obstacles are too dense

#### No Trajectories Found
- Increase `n_samples`
- Check if goals are reachable
- Verify obstacle predictions are reasonable
- Increase planning horizon `T`

#### Trajectories Look Jerky
- Enable `spline_optimization`
- Increase spline smoothness weights
- Check velocity/acceleration limits

#### RViz Shows Nothing
- Check that all topics are publishing
- Verify coordinate frames match
- Enable debug visuals in parameters

---

## Conclusion

This guidance planner is a sophisticated motion planning system that provides multiple, topologically distinct trajectory options for robots operating in dynamic environments. The key insight is that having multiple fundamentally different path options (not just multiple similar paths) gives robots much better adaptability when conditions change.

The system's strength lies in its combination of:
- **Efficient sampling-based planning** (PRM)
- **Rigorous topology classification** (homotopy comparison)
- **Smooth trajectory generation** (spline optimization)  
- **Real-time operation** (node propagation and caching)

Whether you're a beginner trying to understand the concepts or an expert looking to integrate this into a larger system, this guide should provide the foundation you need to effectively use and modify the guidance planner.