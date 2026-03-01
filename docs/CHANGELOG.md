# CHANGELOG.md

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)  
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] - 2026-03-02

### Added
- **MRTA Algorithm Implementations**: 
  - Distributed Hungarian (PR #9)
  - CBBA (Consensus-Based Bundle Algorithm) with task plan visualization
  - GRAPE (Game-theoretic coalition-based) allocation
  - Greedy (nearest-task) allocation
  - All ported and validated against space-simulator baseline
- **Comprehensive Framework Documentation**: 
  - Restructured README.md with architecture overview, quick start guides, and scenario descriptions
  - Added external tool links (Groot2, Webots)
  - Included validation workflow description
- **Multi-Scenario Support**:
  - `simple`: Multi-robot fire suppression with Webots physics simulator
  - `turtle_catcher`: Lightweight BT+ROS2 prototype with turtlesim
- **ROS 2 Integration Features**:
  - Action server framework for robot control
  - Communication topology visualization (RViz support)
  - Task assignment visualization markers
  - Per-robot namespace isolation
- **Data Serialization Improvements**: 
  - Refactored agent/task data structures to be JSON-serializable
  - Eliminated circular reference errors in network messaging
- **Developer Experience**:
  - CONFIG_NAME argument support in run_bt_runners.sh
  - Unified DEBUG environment variable
  - Per-robot configuration with CLI overrides

### Fixed
- Circular reference errors in message serialization (JSON encoding of Vector2 objects)
- Inconsistent environment module references across scenario configs
- Missing position field serialization for network communication
- Robot namespace isolation in Webots simulation
- Action server goal cancellation on shutdown


---

## Previous

For changes from the initial extraction from space-simulator, see [space-simulator CHANGELOG](https://github.com/inmo-jang/space-simulator/blob/main/CHANGELOG.md).

