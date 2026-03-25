# ARGOS: Autonomous Robot Group Orchestration System for Heterogeneous Firefighting Robots with Hose-Aware Path Planning

**Authors:** Minbal [Anonymous for Review]
**Affiliation:** Daegu Bukbu Fire Station, Republic of Korea
**Target Venue:** IROS/ICRA Workshop on Robots for Extreme Environments, or Proceedings of the Korean Society of Hazard Mitigation (소방방재학회) 2026

---

## Abstract

Firefighting robot deployments are currently limited to single-unit remote operation, leaving the potential of multi-robot coordination largely unexplored. We present ARGOS (Autonomous Robot Group Orchestration System), a four-layer heterogeneous robot orchestration platform designed for firefighting scenarios. ARGOS introduces hose-aware path planning — to our knowledge the first computational framework to encode physical firefighting hose constraints directly into robot motion planning — along with a Consensus-Based Bundle Algorithm (CBBA) task allocator, a unified platform abstraction for ground vehicles, aerial drones, and the HR-Sherpa suppression robot, and a mixed synthetic-real fire detection pipeline achieving mAP50 = 0.758 on real-world validation data. The system passes 813 unit tests and has been verified in Gazebo Harmonic simulation with Nav2 goal navigation succeeding within 0.50 m of target.

---

## 1. Introduction

Structural fires impose severe time pressure on firefighting personnel and increasingly involve environments that are too dangerous for human entry. To address this challenge, national fire agencies have begun deploying semi-autonomous firefighting robots. South Korea's National Fire Research Institute (NFRI) has developed the HR-Sherpa, a six-wheel-drive ground vehicle rated to 800°C, with 100 units planned for deployment nationwide between 2027 and 2030 at approximately KRW 2 billion (roughly USD 1.5M) per unit \cite{nfri2026sherpa}. Despite their cost and capability, current deployments are strictly single-unit and operator-driven: a human teleoperator controls each robot individually, and operations halt when communications degrade.

Three fundamental gaps limit the effectiveness of present firefighting robots. First, there is no multi-robot coordination framework: when multiple units are deployed at the same incident, their paths and tasks are planned independently by separate operators, leading to duplicated coverage and potential hose interference. Second, robots are entirely dependent on continuous communication; a link failure or radio shadow caused by structural collapse results in mission suspension rather than autonomous continuation. Third, and most critically, no existing system accounts for the physical constraints imposed by the firefighting hose. A charged 65 mm attack hose weighs approximately 56 kg per 20-meter section, cannot safely be reversed while pressurized, and will reduce water flow by more than 10% if kinked at a sharp angle \cite{nfri2026hose}. Ignoring these constraints in path planning is not merely a software oversimplification — it can cause equipment failure or firefighter injury during hose management.

This paper presents ARGOS, a platform that addresses all three gaps through four contributions:

1. **Heterogeneous orchestration architecture** — a four-layer design (Orchestrator, Mission Modules, Core Services, Platform) with a unified `PlatformInterface` abstraction that coordinates UGV, PX4 drone, and HR-Sherpa platforms under a single command hierarchy without requiring the orchestrator to know platform-specific details.

2. **Hose-aware path planning** — a Nav2 post-processing filter that enforces five physical hose constraints in real time: path length vs. deployed hose remaining, minimum bend radius, charged-hose reverse prohibition, inter-robot hose conflict detection, and role separation between hose-supply and suppression units.

3. **CBBA-based task allocation with bundle extension** — an implementation of the Consensus-Based Bundle Algorithm \cite{choi2009cbba} extended to support multi-task bundles per robot, enabling a drone `FireAlert` to trigger reallocation across the full team including capability-matched handoff to ground units.

4. **Mixed synthetic-real fire detection** — a YOLOv8 fine-tuning pipeline combining SYN-FIRE synthetic indoor fire imagery \cite{arlovic2025synfire} with real D-Fire data \cite{dfire2022}, achieving mAP50 = 0.758 on held-out real-world validation, compared to a COCO-pretrained baseline of 0.370.

The remainder of this paper is organized as follows. Section 2 reviews related work in multi-robot firefighting, task allocation, firefighting robot hardware, fire detection AI, and communication resilience. Section 3 describes the ARGOS system architecture. Section 4 details the hose-aware path planning algorithm. Section 5 presents the CBBA task allocation design. Section 6 covers the fire detection pipeline. Section 7 reports experimental results. Section 8 concludes with directions for future work.

---

## 2. Related Work

### 2.1 Multi-Robot Firefighting Systems

The DARPA Subterranean Challenge (SubT) provided the most comprehensive evaluation of multi-robot coordination in GPS-denied, communication-degraded environments analogous to building fires. Team CERBERUS \cite{cerberus2022} deployed a heterogeneous fleet of ANYmal legged robots and aerial drones, demonstrating that local autonomy — robots continuing their assigned mission without central coordination — outperforms systems that halt on communication loss. Team CoSTAR \cite{costar2022} achieved robust exploration through a homogeneous sensor suite ("CatPack") that allowed uniform state estimation across heterogeneous platforms. Both architectures inform ARGOS: the Supervised Autonomy pattern adopted in Section 3.3 is a direct extension of the DARPA SubT "local autonomy first, communication as bonus" principle.

Closer to the firefighting domain, South Korea has deployed a patrol robot at the Seoul Metropolitan Fire Station for corridor surveillance, but it operates as a single tethered unit with no inter-robot coordination capability. The MARL-based curriculum by Cyborg Research and Griffith University \cite{cyborg2026marl} is the most recent academic system specifically targeting firefighting autonomy, achieving 99.67% mission success in simulation through a three-stage curriculum (single fire, obstacle fire, multi-fire) that parallels the ARGOS development trajectory. Unlike ARGOS, however, that work does not address heterogeneous platforms or physical hose constraints.

### 2.2 Task Allocation

Distributed task allocation for multi-robot systems has been studied extensively. The Consensus-Based Bundle Algorithm (CBBA) introduced by Choi et al. \cite{choi2009cbba} provides a decentralized auction mechanism with convergence guarantees: each robot bids on tasks based on a marginal cost function, and an iterative consensus phase resolves conflicts without central coordination. ARGOS implements CBBA with a bundle extension that allows each robot to hold up to $N$ tasks simultaneously, which is essential when the number of fire investigation points exceeds the team size. The Adaptive CBBA (ACBBA) variant \cite{acbba2025springer} extends the base algorithm with dynamic priority adjustments — a direction we plan to integrate for real-time severity escalation in future work.

### 2.3 Firefighting Robot Hardware

The HR-Sherpa, developed jointly by Hyundai Rotem and NFRI \cite{nfri2026sherpa}, represents the current state of the art in South Korean suppression robots. Its specifications include a 3.1 m × 2.0 m × 1.9 m chassis, six in-wheel motors, dual-mode steering (Ackermann and skid-steer), a 2,650 LPM water cannon at 35 bar, 24-nozzle water curtain for self-protection, and dual thermal cameras (SWIR and LWIR). It carries a 100 m hose reel and operates for approximately five hours on its 20 kWh battery at 5 km/h. ARGOS models all these parameters in the `SherpaPlatform` implementation, including the battery drain model and hose deployment tracking. Tethered robots more broadly have been addressed in cable-constrained motion planning literature \cite{cable2021survey}, but firefighting hose constraints introduce additional operational rules — charged-state reversal prohibition, kink-induced flow loss — that are not present in generic tether models.

### 2.4 Fire Detection AI

Learning-based fire detection has advanced rapidly. The D-Fire dataset \cite{dfire2022} provides 14,122 real-world fire and smoke images with YOLO-format annotations. SYN-FIRE \cite{arlovic2025synfire}, generated in NVIDIA Omniverse, provides 2,030 photorealistic synthetic indoor fire images and has been shown to improve segmentation Dice scores by 2–16% when added to real-data training sets. The FLAME Diffuser \cite{flame2024} uses Stable Diffusion v1.5 for img2img augmentation of fire images with varied lighting and smoke conditions. ARGOS combines all three data sources and evaluates the domain gap between synthetic-only and mixed-data training (Section 6). The AI-Hub YOLOv8s model trained on Korean fire-scene data (mAP50 = 0.953 on its own validation set) provides a performance ceiling reference, though its AGPL-3.0 license requires isolation in a separate package.

### 2.5 Communication Resilience

Standard ROS 2 DDS middleware performs poorly under the intermittent Wi-Fi conditions typical of building fires. The Zenoh middleware (rmw_zenoh) \cite{zenoh2024} reduces discovery traffic by 97–99% relative to DDS in wireless environments and is the preferred transport layer for ARGOS's multi-robot topics. For platform-level control of PX4 drones, ARGOS uses the uXRCE-DDS bridge, which exposes 24 PX4 topics to ROS 2 Jazzy and has been verified at the hardware interface level. The communication fallback architecture — where each robot transitions to a local autonomy mode after a configurable heartbeat timeout — follows the pattern validated in the DARPA SubT CSIRO deployment and the MARL curriculum of \cite{cyborg2026marl}.

---

## 3. System Architecture

ARGOS is designed around a principle inherited from DARPA SubT winning teams: the orchestrator decides *what* to do; individual robots decide *how* to do it. This separation cleanly partitions the software into four layers and supports the incremental replacement of any single layer without disrupting the others.

### 3.1 Four-Layer Architecture

[Figure 1: Four-layer architecture diagram showing data flows between Orchestrator, Mission Modules, Core Services, and Platform layers]

The **Orchestrator** layer (`orchestrator_node.py`, 752 lines after SRP refactoring) is the command authority. It maintains a registry of active robots, tracks mission stage transitions (INIT → EXPLORING → FIRE\_RESPONSE → RETURNING → COMPLETE), and is the only layer that calls the CBBA allocator. It subscribes to `/orchestrator/robot_status` and `/orchestrator/fire_alert` and publishes `/orchestrator/mission_state` and `/orchestrator/autonomy_mode`. The orchestrator does not know whether any given robot is a UGV, drone, or HR-Sherpa — it interacts exclusively through the `PlatformInterface` abstraction.

The **Mission Modules** layer contains plug-in behaviors that can be swapped without modifying the orchestrator. Current modules include reconnaissance (frontier exploration), suppression assist (hose deployment sequencing), structural monitoring, and victim detection. Each module exposes a standard activate/deactivate lifecycle to the orchestrator.

The **Core Services** layer provides shared infrastructure: localization and mapping (`slam_toolbox` for UGV, PX4-native EKF for drones), path planning (Nav2 with Smac Planner 2D and MPPI Controller), fleet communication (Zenoh), state management (ROS 2 LifecycleNode state machine), and sensor fusion (`sensor_fusion.py`). These services are consumed by both the Mission Modules and the Platform layer.

The **Platform** layer implements robot-specific hardware drivers behind the common `PlatformInterface` abstract class. Three concrete implementations exist: `UGVPlatform` (Nav2 NavigateToPose action client), `PX4Platform` (TrajectorySetpoint via uXRCE-DDS), and `SherpaPlatform` (Nav2 with pre-flight hose constraint checking). The dispatch of commands from the orchestrator to specific platforms is handled by `robot_dispatcher.py`.

### 3.2 PlatformInterface Abstraction

The `PlatformInterface` abstract base class defines six methods that every platform must implement:

```
move_to(x, y, z) → bool
emergency_stop() → None
get_capabilities() → RobotCapabilities
get_pose() → PoseStamped
get_battery() → float
return_home() → bool
```

The `RobotCapabilities` dataclass carries boolean flags (`can_fly`, `can_drive`, `has_thermal`, `has_lidar`) and numeric properties (`max_speed`, `battery_capacity`, `platform_type`). When the orchestrator receives a `FireAlert` and needs to dispatch an inspection task, it iterates over registered robots, calls `get_capabilities()`, and passes only capability-matched robots to the CBBA allocator. This design mirrors the DARPA SubT CERBERUS abstraction strategy and follows the "program to an interface, not an implementation" principle.

The contract enforces two safety invariants: `move_to` must return `False` on failure rather than raising an exception (protecting the orchestrator's main loop), and `emergency_stop` must be synchronous and thread-safe (callable from any ROS 2 callback thread). All three platform implementations are validated against 53 unit tests covering normal operation, timeout, battery depletion, and emergency stop scenarios.

### 3.3 Supervised Autonomy Pattern

ARGOS implements what we term Supervised Autonomy: a human incident commander issues high-level directives through a web dashboard (rosbridge + React), the orchestrator translates these into task assignments, and individual robots execute their assigned tasks with local autonomy. [Figure 2: Supervised Autonomy command hierarchy]

The communication architecture publishes `/orchestrator/autonomy_mode` with two possible values: `CENTRALIZED` (normal operation, orchestrator-driven) and `LOCAL_AUTONOMY` (communication loss fallback). When a robot's heartbeat timeout expires (configurable, default 5 seconds), it transitions to `LOCAL_AUTONOMY` and continues frontier exploration independently, publishing status on a best-effort QoS profile. Upon reconnection, the orchestrator re-registers the robot and resumes centralized coordination.

### 3.4 LifecycleNode Initialization Ordering

ARGOS uses ROS 2 LifecycleNode for all four core nodes: `orchestrator_node`, `frontier_explorer_node`, `hotspot_detector_node`, and `gas_sensor_node`. The LifecycleNode pattern provides deterministic initialization: nodes configure (declare parameters, allocate buffers) before activating (create publishers, subscribers, and timers). This prevents the common race condition where a subscriber is created before the underlying network is ready, which caused erroneous `nav_error` readings in earlier versions. The `hose_aware_planner.py` node also implements LifecycleNode, ensuring that path filtering subscribers are created only after the hose status topic is confirmed available.

### 3.5 Orchestrator SRP Refactoring

The original orchestrator implementation grew to 1,918 lines over the course of development (Phases A through E), accumulating sensor fusion logic, robot dispatch decisions, and CBBA coordination into a single monolithic file. The Phase G-1 refactoring applied the Single Responsibility Principle by extracting three specialized modules:

- `sensor_fusion.py` — Kalman-filtered aggregation of thermal, gas, structural, and audio sensor readings
- `robot_dispatcher.py` — translates orchestrator task assignments into platform-specific commands
- `orchestrator_types.py` — shared dataclasses and command objects (reducing circular import risk)

The resulting `orchestrator_node.py` is 752 lines, contains only coordination logic, and all imports are now explicit. This decomposition is reflected in the test suite: sensor fusion and dispatcher logic are independently testable without instantiating a full LifecycleNode.

---

## 4. Hose-Aware Path Planning

### 4.1 Motivation

Firefighting hose management is a specialized operational skill that experienced firefighters develop over years of practice. A charged 65 mm attack hose carries approximately 56 kg per 20-meter section when fully loaded with water \cite{nfri2026hose}. Kinking the hose at an angle sharper than approximately 90° reduces flow by more than 10% and pressure by more than 15%; complete folding blocks the water path entirely. In the context of a robot carrying a 100 m hose reel into a burning structure, three failure modes are operationally critical: the robot advancing further than the hose can reach (leaving the hose taut and at risk of being pulled from its coupling), the robot executing a sharp turn that kinks the hose, and the robot reversing while the hose is pressurized (causing the hose to fold and potentially rupture). To our knowledge, no prior autonomous robot system — firefighting or otherwise — has encoded these constraints into its navigation stack. ARGOS addresses this gap through the Hose-Aware Planner.

### 4.2 Physical Model

The hose physical model is maintained by `hose_tether_node.py`, a ROS 2 node that subscribes to the robot's odometry and accumulates deployed hose length by integrating position increments above a minimum spacing threshold of 0.5 m (to filter odometry noise). The hose anchor point — the entry door of the building — is a configurable parameter representing where the hose couples to the external supply line.

The node publishes `/hose/status` as a `Float32MultiArray` with three fields: `remaining_m` (deployed hose subtracted from the 100 m maximum reel), `kink_risk` (computed from the circumradius of the most recent three path points), and `charged` (a boolean indicating whether the suppression pump is active). This topic is consumed by both the `SherpaPlatform` (for pre-move constraint checking) and the `HoseAwarePlanner` (for path filtering).

[Figure 3: Hose physical model diagram showing reel, anchor point, deployed length, and kink risk zones]

The kink risk metric $\kappa$ is derived from the circumradius $R$ of three consecutive odometry waypoints:

$$R = \frac{abc}{4 \cdot \text{Area}(P_0, P_1, P_2)}$$

where $a, b, c$ are the triangle side lengths computed via Euclidean distance, and Area is the shoelace formula. When $R < r_{\min}$ (default 0.5 m), the segment is flagged as a kink risk. If the three points are collinear (Area $< 10^{-9}$), $R$ is treated as infinite and no kink is reported.

### 4.3 Five Constraint Rules

ARGOS encodes five operational rules derived from firefighter field experience and NFRI engineering specifications:

**Rule 1 — Depth Limit.** The robot's planned path length must not exceed the remaining hose. If the Nav2 planner generates a path of total length $L > L_{\text{remaining}}$, the `HoseAwarePlanner` trims the path to $0.9 \times L_{\text{remaining}}$ (a 10% safety margin). This prevents the hose from being pulled taut against its supply coupling.

**Rule 2 — Sharp Turn Prohibition.** Any path segment where the circumradius of three consecutive waypoints falls below $r_{\min} = 0.5$ m is flagged as a kink hazard. The planner applies midpoint smoothing to flagged waypoints, replacing them with the average of their neighboring poses. A production deployment would substitute a full spline re-interpolation; the midpoint approximation provides correctness in simulation.

**Rule 3 — Charged-Hose Reverse Prohibition.** When `charged = True` (pump active), any path segment whose direction vector has a negative dot product with the overall start-to-goal direction vector is classified as a reverse segment. If any reverse segments are detected, the entire path is rejected rather than modified: reversal of a pressurized hose is considered unrecoverable by the planner and must be re-planned by the orchestrator.

**Rule 4 — Role Separation.** The orchestrator enforces that at most one robot per team operates in hose-deployment mode at any time. A second Sherpa unit, if present, is assigned to hose-supply support (remaining near the building entry to manage slack) rather than penetrating the structure. Drones are never assigned hose tasks due to their `can_drive = False` capability flag.

**Rule 5 — Inter-Robot Hose Conflict Avoidance.** When two Sherpa units are both inside a structure, their hose paths may physically cross. The orchestrator detects this condition using a 2D line-segment intersection test (CCW-based) on the broadcasted hose path visualizations and publishes a resolution directive on `/orchestrator/hose_conflict`. The directed conflict resolution assigns priority to the unit closer to the fire seat and instructs the lower-priority unit to hold position until the conflict is cleared.

### 4.4 Hose-Aware Planner Implementation

The `HoseAwarePlanner` node is a ROS 2 LifecycleNode that subscribes to `/{robot_id}/plan` (the Nav2 global planner output) and `/{robot_id}/hose/status`, and publishes `/{robot_id}/hose_filtered_plan` and `/{robot_id}/hose_planner/status`. The node applies Rules 1, 2, and 3 in priority order: a length violation is corrected first (trimming), then sharp turns are smoothed on the trimmed path, and finally the reverse check is performed on the smoothed path.

[Figure 4: HoseAwarePlanner data flow — Nav2 plan input, constraint evaluation, filtered plan output]

The node uses a `ReentrantCallbackGroup` to allow concurrent handling of hose status updates and plan callbacks, preventing a stale hose status from blocking path processing. QoS for the plan subscription uses `RELIABLE` reliability with depth 10; hose status uses `BEST_EFFORT` with depth 1, as timeliness is more important than delivery guarantee for sensor data.

If no `/hose/status` message has been received since activation, the planner logs a throttled warning and passes the Nav2 plan through unmodified (graceful degradation). This design choice ensures that a hose sensor failure does not halt navigation, while still surfacing the anomaly to operators through the status log.

### 4.5 Multi-Sherpa Hose Conflict Detection

When two or more Sherpa units are active inside a structure, their deployed hose segments form line segments in the 2D map frame. The orchestrator checks for intersection using the Counterclockwise (CCW) orientation test for line segments $(P_1 P_2)$ and $(P_3 P_4)$:

$$\text{intersects} \Leftrightarrow \text{CCW}(P_1, P_3, P_4) \neq \text{CCW}(P_2, P_3, P_4) \;\text{and}\; \text{CCW}(P_1, P_2, P_3) \neq \text{CCW}(P_1, P_2, P_4)$$

where $\text{CCW}(A, B, C) = (C_y - A_y)(B_x - A_x) > (B_y - A_y)(C_x - A_x)$.

A detected intersection triggers publication of a JSON conflict event on `/orchestrator/hose_conflict` containing the two robot IDs and a resolution directive. The resolution strategy assigns right-of-way based on distance to the nearest fire seat: the unit with the shorter distance to the highest-severity `FireAlert` location continues, while the other holds until the crossing is geometrically resolved. This mirrors the physical convention used by firefighters when two hose lines approach the same seat of fire from different directions.

---

*Sections 5–8 to follow in subsequent draft.*

---

## References

\bibitem{nfri2026sherpa}
National Fire Research Institute (NFRI), "HR-Sherpa Unmanned Firefighting Ground Vehicle: Technical Specifications and Field Deployment Report," Technical Report, 2026.

\bibitem{nfri2026hose}
NFRI Engineering Division, "Physical Constraints of 65mm Firefighting Hose in Robot-Assisted Operations," Internal Technical Memorandum, 2026.

\bibitem{cerberus2022}
M. Tranzatto et al., "CERBERUS: Autonomous Legged and Aerial Robotic Exploration in the Tunnel and Urban Circuits of the DARPA Subterranean Challenge," Field Robotics, vol. 2, pp. 274–324, 2022.

\bibitem{costar2022}
A.-a. Agha et al., "NeBula: Quest for Robotic Autonomy in Challenging Environments; TEAM CoSTAR at the DARPA Subterranean Challenge," Field Robotics, vol. 2, pp. 1432–1506, 2022.

\bibitem{cyborg2026marl}
Cyborg Research / Griffith University, "Multi-Agent Reinforcement Learning with Three-Stage Curriculum for Autonomous Firefighting Robot Teams," arXiv preprint, 2026.

\bibitem{choi2009cbba}
H.-L. Choi, L. Brunet, and J. P. How, "Consensus-Based Decentralized Auctions for Robust Task Allocation," IEEE Transactions on Robotics, vol. 25, no. 4, pp. 912–926, 2009.

\bibitem{acbba2025springer}
[Author et al.], "Adaptive Consensus-Based Bundle Algorithm for Dynamic Multi-Robot Task Allocation," Springer Lecture Notes in Computer Science, 2025.

\bibitem{arlovic2025synfire}
D. Arlović et al., "SYN-FIRE: Synthetic Indoor Fire Dataset Generated with NVIDIA Omniverse for Deep Learning Fire Detection," FigShare, 2025.

\bibitem{dfire2022}
D. Foggia et al., "D-Fire: An Image Dataset for Fire and Smoke Detection," arXiv:2207.03038, 2022.

\bibitem{flame2024}
[Author et al.], "FLAME Diffuser: Stable Diffusion-Based Augmentation for Fire Image Datasets," arXiv preprint, 2024.

\bibitem{cable2021survey}
[Author et al.], "A Survey of Cable and Tether-Constrained Robot Motion Planning," IEEE Transactions on Robotics, 2021.

\bibitem{zenoh2024}
Eclipse Zenoh Contributors, "Zenoh: Zero Overhead Pub/Sub, Store/Query and Compute," Eclipse Foundation Technical Report, 2024.
