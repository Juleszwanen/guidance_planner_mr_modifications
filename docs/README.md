# Guidance Planner Documentation Index

## Overview
This directory contains comprehensive documentation for understanding and using the Guidance Planner system. The documentation is organized from basic to advanced topics.

## Documentation Files

### 📋 [Quick Reference Guide](QUICK_REFERENCE.md)
**Start here for immediate practical use**
- Essential launch command understanding
- Key configuration parameters
- Common issues and solutions
- Performance expectations
- Quick tuning guide

### 📖 [Comprehensive System Guide](COMPREHENSIVE_SYSTEM_GUIDE.md) 
**Complete technical reference (30,000+ words)**
- System overview and architecture
- Algorithm workflow details
- Homotopy comparison functions explained
- Graph building and propagation
- Configuration parameter meanings
- Code flow examples
- Beginner's guide with hands-on instructions

### 🔍 [Launch Analysis](LAUNCH_ANALYSIS.md)
**Deep dive into ros1_example.launch execution**
- Launch file breakdown line-by-line
- System initialization sequence
- Component interactions and data flow
- Performance characteristics
- Error handling and edge cases
- Integration guidelines

### 📊 [Visual System Guide](VISUAL_SYSTEM_GUIDE.md)
**Diagrams and flowcharts**
- Component interaction diagrams
- Algorithm execution timeline
- Data flow visualizations
- Homotopy classification examples
- Performance scaling charts
- Configuration impact analysis

## How to Use This Documentation

### For Newcomers
1. Start with [Quick Reference Guide](QUICK_REFERENCE.md) to get running quickly
2. Read the "Beginner's Guide" section in [Comprehensive System Guide](COMPREHENSIVE_SYSTEM_GUIDE.md)
3. Use [Visual System Guide](VISUAL_SYSTEM_GUIDE.md) to understand the architecture

### For Developers
1. Review [Launch Analysis](LAUNCH_ANALYSIS.md) for implementation details
2. Study the "Algorithm Workflow" and "Component Interactions" sections
3. Use [Quick Reference Guide](QUICK_REFERENCE.md) for parameter tuning

### For Integration
1. Check "Integration Guidelines" in [Launch Analysis](LAUNCH_ANALYSIS.md)
2. Review "ROS Communication" sections for interface details
3. Follow examples in [Comprehensive System Guide](COMPREHENSIVE_SYSTEM_GUIDE.md)

### For Debugging
1. Use "Debugging Tips" in [Comprehensive System Guide](COMPREHENSIVE_SYSTEM_GUIDE.md)
2. Check "Error Handling" in [Launch Analysis](LAUNCH_ANALYSIS.md)
3. Reference performance charts in [Visual System Guide](VISUAL_SYSTEM_GUIDE.md)

## Key Topics Covered

### System Understanding
- Complete explanation of topology-aware motion planning
- How the Visibility-PRM algorithm works in 3D space-time
- Why multiple homotopy-distinct trajectories matter
- Real-time graph propagation and memory management

### Technical Implementation
- C++ class architecture and responsibilities
- ROS communication patterns (topics, services, parameters)
- Algorithm complexity analysis and performance optimization
- Memory management and real-time considerations

### Practical Usage
- Configuration parameter effects and tuning strategies
- Integration with real robot systems
- Debugging techniques and common pitfalls
- Performance expectations across different scenarios

### Mathematical Background
- Homotopy comparison functions (H-signature, Winding angles, UVD)
- Spline optimization and trajectory smoothing
- Collision detection and visibility checking
- Graph theory applications in motion planning

## External Resources

### Academic Papers
- **Journal Paper**: "Topology-Driven Parallel Trajectory Optimization in Dynamic Environments" (IEEE T-RO 2024)
- **Conference Paper**: "Globally Guided Trajectory Optimization in Dynamic Environments" (ICRA 2023)

### Related Repositories
- [mpc_planner](https://github.com/tud-amr/mpc_planner) - High-level integration example
- [ros_tools](https://github.com/oscardegroot/ros_tools) - Required dependency
- [mpc_planner_ws](https://github.com/tud-amr/mpc_planner_ws) - Containerized development environment

## Documentation Maintenance

This documentation is designed to be:
- **Self-contained**: Can be read independently of code
- **Comprehensive**: Covers all aspects from basics to advanced topics
- **Practical**: Includes working examples and real-world guidance
- **Accessible**: Written for different experience levels

If you find gaps or have suggestions for improvement, please contribute to keep this documentation current and useful.

---

**Total Documentation**: ~70,000 words across 4 comprehensive guides
**Last Updated**: Based on repository state as of creation
**Scope**: Complete system coverage from launch to integration