# Safe Navigation in Unknown Cluttered Environments via Direction-Aware Convex Free-Region Generation


## Overview

Convex free regions provide a structured and optimization-friendly representation of collision-free space, but enlarging local free space only according to surrounding obstacle geometry can be insufficient in cluttered environments. In narrow passages, the generated region may fail to both accommodate robot geometry and preserve traversable extension along candidate motion directions. In addition, when robot geometry is modeled explicitly, checking safety only at discretized trajectory samples does not guarantee continuously collision-free motion.

To address these issues, FRGraph jointly incorporates candidate motion directions and robot geometry into convex free-region generation, and achieves continuously collision-free motion through continuous-safe trajectory generation. Within each region, the framework performs geometry-aware target pose selection and trajectory generation, together with Lipschitz-based continuous safety certification and local refinement. The resulting free regions and candidate motions are maintained in a region-based graph to support incremental planning.

<p align="center">
  <img src="doc/pipeline.png" width="900" alt="Pipeline overview"/>
</p>

---

## Key Features

- **Direction-aware convex free-region generation**  
  Candidate navigation directions are incorporated into obstacle selection and hyperplane optimization through a direction-biased QP formulation, producing free regions that better preserve traversable space while accommodating robot geometry.

- **Continuous-safe trajectory generation**  
  The framework combines Bézier trajectory parameterization with Lipschitz-based adaptive interval subdivision for continuous safety certification, together with a local QP-based control-point refinement scheme for iterative violation correction.

- **Region-based graph for incremental planning**  
  Generated free regions and candidate motions are organized in a region-based navigation graph, enabling incremental planning and recovery from locally unproductive branches.

---

## Demo

### 2D Navigation in Cluttered Environments

<p align="center">
  <img src="doc/FRGraph2Dexp.gif" width="660" alt="2D navigation demo"/>
</p>

The proposed framework enables reliable collision-free navigation in cluttered 2D environments across a wide range of obstacle densities.

---

### 3D Navigation with Dead-End Handling

<p align="center">
  <img src="doc/3dsim.gif" width="660" alt="3D navigation demo"/>
</p>

The maintained region-based graph allows the planner to backtrack from dead-end branches and continue navigation toward the goal.

---

### Real-World Experiments

<p align="center">
  <img src="doc/real_world.gif" width="660" alt="Real-world demo"/>
</p>

The framework transfers effectively to real robots and maintains collision-free navigation in cluttered real-world scenarios.

---

## Results at a Glance

- Quantitative 2D experiments show that the proposed method generates free regions better aligned with downstream traversal and enables reliable collision-free navigation.
- Additional 3D experiments demonstrate dead-end handling and backtracking enabled by the maintained region-based graph.
- Real-world experiments validate the practical applicability of the framework in both 2D and 3D cluttered environments.

<!-- <p align="center">
  <img src="assets/results_overview.png" width="760" alt="Results overview"/>
</p> -->

---

## Installation

This project is built in a ROS catkin workspace.

### Dependencies

Before building FRGraph, please make sure the following dependency is available:

- [`catkin_simple`](https://github.com/catkin/catkin_simple)

In addition, the `DecompROS` module used in this project is adapted from the original implementation:

- [`DecompROS`](https://github.com/sikang/DecompROS.git)

### Build Instructions

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <your_repo_url>
```

2. Clone `catkin_simple` into the `third_party` directory:

```bash
cd FRGraph/third_party
git clone https://github.com/catkin/catkin_simple.git
```

3. Go back to the workspace root and build the `decomp_*` packages first:

```bash
cd ~/catkin_ws
catkin build decomp_*
```

4. Build the whole workspace:

```bash
catkin build
```

If all steps finish successfully, FRGraph is ready to use.

---

### Quick Start (2D Example)

```bash
roslaunch simple_robot simple_robot.launch if_3d_Lidar:=0
roslaunch planner_manager planner.launch
```

---

## Usage

FRGraph consists of two main components:

- A robot interface (simulation or real robot)
- The planner (`planner_manager`)

### 1. Launch the Robot

You can launch the provided example robot:

```bash
roslaunch simple_robot simple_robot.launch if_3d_Lidar:=0
```

This launch file provides a simple robot setup and can be replaced with your own robot system.

---

### 2. Launch the Planner

```bash
roslaunch planner_manager planner.launch
```

---

## Running in 2D vs 3D Environments

### 2D Mode

To run in a 2D environment:

1. Launch the robot with 2D LiDAR:

```bash
roslaunch simple_robot simple_robot.launch if_3d_Lidar:=0
```

2. Set the environment type in:

```bash
src/planner_manager/config/config.yaml
```

```yaml
env_type: 0
```

3. Set robot geometry:

- Modify the robot vertex coordinates in the configuration file according to your robot shape.

4. Required topics:

- Point cloud (2D LiDAR):  
  `/scan`
- Odometry:  
  `/odom`

---

### 3D Mode

To run in a 3D environment:

1. Launch the robot with 3D LiDAR:

```bash
roslaunch simple_robot simple_robot.launch if_3d_Lidar:=1
```

2. Set the environment type:

```yaml
env_type: 1
```

3. Set robot geometry:

- Update the robot vertex coordinates to match your robot model.

4. Required topics:

- Point cloud (3D LiDAR):  
  `/velodyne_points`
- Odometry:  
  `/odom`

---

## Gap Extraction Parameters

The navigation direction extraction module converts the local LiDAR observation into a range-map representation and extracts candidate direction hypotheses from detected gap regions. These candidate directions are not final traversability certificates. Robot-size and orientation feasibility are handled later by direction-aware free-region generation, target-pose selection, and continuous safety verification.

The main gap-extraction parameters are configured in:

```bash
src/planner_manager/config/config.yaml
```

Representative values used in our experiments are summarized below.

| Item | 2D setting | 3D setting |
|---|---:|---:|
| Range-map representation | Single-row angular range map | Spherical range map over azimuth and elevation |
| Range-map size | `1600 x 1` | `1600 x 32` |
| Horizontal field of view | `[-pi, pi]` | `[-pi, pi]` |
| Vertical field of view | Single elevation row | `[-30.67 deg, 30.67 deg]` |
| Local point-cloud range | `3 m` | `3 m` |
| Open-gap minimum region size | `10` pixels | `20` pixels |
| Limited-gap minimum region size | `2` pixels | `48` pixels |
| Open-gap subregion span | `45 deg` in yaw | `45 deg` in yaw, `30 deg` in elevation |
| Limited-gap subregion span | `30 deg` in yaw | `30 deg` in yaw, `30 deg` in elevation |
| Minimum open-gap subregion size | `20` pixels | `40` pixels |
| Minimum limited-gap subregion size | `2` pixels | `32` pixels |
| Limited-gap splitting threshold | `30 deg` in yaw | `30 deg` in yaw, `30 deg` in elevation |
| Limited-gap direction bias | `10 deg` in yaw/elevation | `10 deg` in yaw/elevation |
| Number of candidate directions | Not fixed a priori; determined by detected gap subregions | Not fixed a priori; determined by detected gap subregions |
| Robot-size filtering | Not performed in gap extraction; handled later by robot-geometry-aware region generation and safety verification | Same |

Depth discontinuities are detected by comparing adjacent range-map cells using an adaptive range threshold of the form:

```text
a + b * r_near * sin(dpsi)
```

together with a geometric occlusion check. The edge-detection parameters used in our experiments are:

| Parameter group | Values |
|---|---|
| Horizontal edge detection | `a_h = 0.20`, `b_h = 0.02`, `lambda_h = 0.5` |
| Vertical edge detection | `a_v = 0.30`, `b_v = 0.05`, `lambda_v = 0.5` |

---

## Additional Experimental Statistics

### Trajectory Refinement Statistics

The following table summarizes the refinement behavior in
randomized 2D environments, using 10 scenes and 40 navigation
trials per obstacle density.

| Obstacle Density | Traj. Attempts | Accepted Traj. | Traj. Entering Refinement | Refinement Rate | Avg. Refinement Iters. | Max Refinement Iters. | Failed Attempts |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0.6 | 182 | 182 | 2 | 1.10% | 1.00 | 1 | 0 |
| 0.8 | 417 | 417 | 25 | 6.00% | 4.20 | 24 | 0 |
| 1.0 | 443 | 441 | 19 | 4.31% | 2.21 | 5 | 2 |
| 1.2 | 756 | 750 | 25 | 3.33% | 3.80 | 20 | 6 |
| Overall | 1798 | 1790 | 71 | 3.97% | 3.44 | 24 | 8 |

The refinement rate is calculated over accepted trajectories.
Overall, only 3.97% of accepted trajectories require refinement,
with an average of 3.44 refinement iterations.

Failed trajectory-generation attempts do not necessarily indicate
navigation failure. The corresponding frontier edge is invalidated,
and the graph planner attempts another available edge.

### Confidence Intervals for Navigation Performance

We report 95% Wilson score confidence intervals for Complete Rate
(CR) and Collision-Free Rate (CFR) in the randomized 2D navigation
experiments.

For each obstacle density, 10 randomly generated scenarios are
evaluated, with four repeated closed-loop runs per scenario under
identical start and goal poses, yielding 40 runs per method.

The confidence intervals are calculated from the observed binary
outcomes of the 40 runs using the Wilson score method.

#### Comparison with Baseline Planners

| Density | Planner | CR | 95% CI (CR) | CFR | 95% CI (CFR) |
|---|---|---:|---|---:|---|
| 0.6 | DDR-OPT | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 0.6 | FASTER | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 0.6 | FRTree | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 0.6 | Ours | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 0.8 | DDR-OPT | 1.00 | [0.912, 1.000] | 0.60 | [0.446, 0.737] |
| 0.8 | FASTER | 0.70 | [0.546, 0.819] | 0.28 | [0.161, 0.428] |
| 0.8 | FRTree | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 0.8 | Ours | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 1.0 | DDR-OPT | 0.95 | [0.835, 0.986] | 0.60 | [0.446, 0.737] |
| 1.0 | FASTER | 0.45 | [0.307, 0.602] | 0.20 | [0.105, 0.348] |
| 1.0 | FRTree | 1.00 | [0.912, 1.000] | 0.65 | [0.495, 0.779] |
| 1.0 | Ours | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 1.2 | DDR-OPT | 0.50 | [0.352, 0.648] | 0.63 | [0.470, 0.758] |
| 1.2 | FASTER | 0.00 | [0.000, 0.088] | 0.40 | [0.263, 0.554] |
| 1.2 | FRTree | 0.80 | [0.652, 0.895] | 0.28 | [0.161, 0.428] |
| 1.2 | Ours | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |

#### Ablation Study

| Density | Variant | CR | 95% CI (CR) | CFR | 95% CI (CFR) |
|---|---|---:|---|---:|---|
| 1.0 | w/o Direction-Aware | 0.78 | [0.625, 0.877] | 1.00 | [0.912, 1.000] |
| 1.0 | w/o Obstacle Selection | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 1.0 | w/o QP Bias | 0.78 | [0.625, 0.877] | 1.00 | [0.912, 1.000] |
| 1.0 | w/o Continuous Safety | 0.98 | [0.871, 0.996] | 0.98 | [0.871, 0.996] |
| 1.0 | Ours (Full) | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |
| 1.2 | w/o Direction-Aware | 0.48 | [0.329, 0.625] | 1.00 | [0.912, 1.000] |
| 1.2 | w/o Obstacle Selection | 0.90 | [0.769, 0.960] | 1.00 | [0.912, 1.000] |
| 1.2 | w/o QP Bias | 0.40 | [0.263, 0.554] | 1.00 | [0.912, 1.000] |
| 1.2 | w/o Continuous Safety | 0.83 | [0.681, 0.913] | 0.88 | [0.739, 0.945] |
| 1.2 | Ours (Full) | 1.00 | [0.912, 1.000] | 1.00 | [0.912, 1.000] |

#### Statistical Interpretation

The intervals are computed using the Wilson score method with
a 95% confidence level and 40 binary trial outcomes per setting.

Each obstacle configuration is evaluated through four separate
closed-loop runs. Consequently, the reported intervals represent
approximate trial-level uncertainty under a binomial model.

Because repeated runs within the same scenario share the same
obstacle configuration and start-goal pose, the intervals do not
explicitly account for within-scene correlation and should not
be interpreted as confidence intervals obtained from 40
independently generated environments.

Small differences in observed rates, such as 1.00 versus 0.90,
should not be considered conclusive evidence of statistical
superiority.


### Computational Scalability Benchmarks

We provide additional computational scalability benchmarks to evaluate how the runtime of the proposed framework varies with local obstacle-point count, convex-region complexity, and orientation-sampling resolution.

The experiments focus on two main computational modules:

- **Direction-Aware Free-Region Generation:** Construction of convex free regions from locally observed obstacle points.
- **Target-Pose Selection:** Selection of a feasible target position and orientation within a generated convex region.

All benchmarks reuse the original algorithm implementations without modifying the navigation or optimization methods.

The supplementary evaluation includes:

1. 2D point-count scalability.
2. 3D point-count scalability.
3. 3D orientation-sampling scalability.

These experiments measure individual planning modules rather than complete closed-loop navigation performance.

#### 1. Computational Complexity

The proposed free-region generation method operates on locally cropped point clouds within a fixed sensing range of 3 m.

At each iteration, the algorithm selects an obstacle point, constructs a separating hyperplane through a quadratic program (QP), and updates the remaining obstacle points.

Let:

- \(N\): Initial number of local obstacle points.
- \(N_i\): Number of remaining obstacle points at iteration \(i\).
- \(K\): Number of hyperplane-generation iterations before redundant-constraint removal.
- \(H\): Number of hyperplanes in the resulting convex region.
- \(S\): Number of evaluated target orientations.

The point-processing complexity is bounded by:

\[
O\left(\sum_{i=1}^{K}N_i\right)
\subseteq O(NK).
\]

For a fixed robot model, each separating-hyperplane QP has a fixed number of optimization variables and robot-geometry constraints, independent of the obstacle-point count.

However, increasing the number of input points may increase the number of required hyperplane-generation iterations and QP solves.

Target-pose selection does not directly process the original point cloud. Instead, its computational cost depends primarily on the number of hyperplane constraints and the number of evaluated orientations.

The implementation uses vertex enumeration for target-position optimization. The corresponding code-structure worst-case bounds are:

- **2D:** \(O(SH^3)\)
- **3D:** \(O(H^3+SH^4)\)

These expressions describe worst-case computational upper bounds, not empirically fitted scaling laws.

---

#### 2. 2D Point-Count Scalability

**Experimental Setup**

We evaluate 90 randomized 2D environments, each tested at five obstacle-point-count levels, resulting in 450 configurations.

A fixed 8192-ray reference scan is generated for each environment. Different point counts are obtained through nested random subsampling of the local obstacle points.

The subsampling does not preserve uniform angular spacing. Therefore, this experiment evaluates **point-count scaling**, rather than changes in physical LiDAR angular resolution.

For each configuration:

- The input is warmed up three times.
- Runtime is measured ten times.
- The median of these measurements is used as the configuration runtime.
- Mean and sample standard deviation are computed across the 90 environments in each group.

The experiments were conducted on an Intel Core i7-12700F using `-O3` optimization, with serial execution and CPU-core affinity.

**Results**

| Input Point Group | Actual Input Points | Region Generation Time (ms) | Target Selection Time (ms) | Hyperplane Count | QP Solves |
|---|---:|---:|---:|---:|---:|
| <200 | 152.3 ± 30.5 | 0.133 ± 0.060 | 0.069 ± 0.106 | 17.20 ± 8.11 | 21.49 ± 9.53 |
| 200–<400 | 298.6 ± 58.8 | 0.244 ± 0.106 | 0.280 ± 0.431 | 29.11 ± 13.54 | 37.52 ± 15.12 |
| 400–<600 | 508.2 ± 60.5 | 0.444 ± 0.202 | 0.986 ± 1.325 | 47.31 ± 21.76 | 62.99 ± 25.39 |
| 600–800 | 704.6 ± 60.9 | 0.668 ± 0.321 | 2.174 ± 2.966 | 64.47 ± 29.27 | 87.11 ± 33.66 |
| >800 | 1180.6 ± 246.0 | 1.343 ± 0.794 | 7.191 ± 10.559 | 102.09 ± 44.68 | 141.07 ± 56.84 |

All continuous metrics are reported as mean ± sample standard deviation, with 90 configurations per group.

**Runtime Distribution**

To characterize computational variability, we additionally report the median and 95th-percentile (P95) runtime across configurations.

| Input Point Group | Region Generation Median (ms) | Region Generation P95 (ms) | Target Selection Median (ms) | Target Selection P95 (ms) |
|---|---:|---:|---:|---:|
| <200 | 0.120 | 0.245 | 0.036 | 0.243 |
| 200–<400 | 0.234 | 0.381 | 0.124 | 0.864 |
| 400–<600 | 0.450 | 0.769 | 0.573 | 3.905 |
| 600–800 | 0.637 | 1.318 | 1.166 | 9.024 |
| >800 | 1.132 | 2.771 | 3.386 | 24.114 |

**Observations**

As the average input-point count increases from approximately 152 to 1181:

- Region-generation time increases from 0.133 ms to 1.343 ms.
- Target-pose selection time increases from 0.069 ms to 7.191 ms.
- The average number of generated hyperplanes increases from 17.20 to 102.09.
- The average number of QP solves increases from 21.49 to 141.07.

The number of evaluated orientations remains fixed at 45 across all configurations.

Therefore, the substantial increase in target-selection time is associated with increasing region-constraint complexity rather than additional orientation samples.

Runtime increases across the paired environments, with additional long-tail behavior in complex regions. In the highest point-count group, target-selection runtime reaches a maximum of 71.213 ms.

All 450 configurations successfully produce regions and geometrically feasible target poses. This does not imply continuous trajectory safety or successful closed-loop navigation.

---

#### 3. 3D Point-Count Scalability

**Experimental Setup**

We evaluate 30 fixed 3D local configurations at five obstacle-point-count levels:

250, 500, 1000, 2000, and 4000 points.

The experiments use the original 3D region-generation and target-pose selection implementations.

Input point subsets are prepared before timing. Consequently, the measurements characterize the computational cost of the region-generation and target-selection modules under controlled input sizes, rather than the entire LiDAR preprocessing pipeline.

Each point-count level contains 30 configurations.

**Results**

| Input Points | Region Generation Time (ms) | Target Selection Time (ms) | Hyperplane Count | QP Solves | Completed Regions |
|---|---:|---:|---:|---:|---:|
| 250 | 0.083 ± 0.066 | 0.435 ± 0.211 | 8.27 ± 1.48 | 4.53 ± 2.60 | 28/30 |
| 500 | 0.094 ± 0.067 | 0.540 ± 0.476 | 8.53 ± 1.93 | 5.17 ± 3.35 | 28/30 |
| 1000 | 0.110 ± 0.084 | 0.582 ± 0.558 | 8.70 ± 2.22 | 6.20 ± 5.14 | 29/30 |
| 2000 | 0.161 ± 0.126 | 0.771 ± 0.859 | 9.20 ± 2.66 | 8.00 ± 7.73 | 28/30 |
| 4000 | 0.270 ± 0.314 | 1.333 ± 2.100 | 10.17 ± 3.65 | 12.13 ± 13.70 | 29/30 |

All continuous metrics are mean ± sample standard deviation.

The reported runtime statistics include all configurations, including unsuccessful QP calls.

Across the 150 configurations, 142 complete region generation successfully, while eight encounter non-successful QP solves. Partial region outputs from unsuccessful calls are retained in the timing statistics but are not counted as completed regions.

**Observations**

As the input count increases from 250 to 4000:

- Average region-generation time increases from 0.083 ms to 0.270 ms.
- Average target-selection time increases from 0.435 ms to 1.333 ms.
- Average hyperplane count increases from 8.27 to 10.17.

The resulting regions remain relatively simple within this controlled input range, explaining the comparatively modest target-selection costs.

**Scope of the Point-Count Experiment**

This experiment is limited to 250–4000 locally selected obstacle points.

A separate high-resolution reference scan with 3200 × 64 rays was used in the 3D orientation-sampling benchmark. Across its 70 local configurations, the number of obstacle points entering the region-generation module had:

- Mean: 38,230.9
- Median: 26,729.5
- P95: 93,178.7
- Maximum: 141,329

These statistics correspond to the high-resolution reference configuration and should not be interpreted as measurements from the original 1600 × 32 LiDAR configuration.

Consequently, the controlled 250–4000-point experiment does not establish computational performance over the entire range of high-resolution local point-cloud inputs.

---

#### 4. 3D Orientation-Sampling Scalability

**Coarse-to-Fine Orientation Search**

The 3D target-pose selection module adopts a two-stage coarse-to-fine orientation search.

In the first stage, the algorithm evaluates a coarse roll-pitch-yaw grid. For each yaw sample, it retains the highest-scoring roll-pitch combination.

The three highest-scoring yaw candidates are then refined using denser local orientation sampling.

Under the default configuration:

**Coarse Stage**

- Yaw samples: 18
- Roll samples: 5
- Pitch samples: 5
- Total coarse evaluations: 450

**Fine Stage**

- Top yaw candidates: 3
- Yaw refinement range: ±20°
- Roll/Pitch refinement range: ±10°
- Fine sampling step: 5°
- Total fine evaluations: 675

The default configuration therefore performs:

\[
450+675=1125
\]

orientation evaluations.

This procedure is a coarse-to-fine grid search, not a binary search or exhaustive fine-resolution sampling of the entire roll-pitch-yaw space.

**Experimental Setup**

We evaluate three orientation-sampling settings:

| Setting | Coarse Yaw Samples | Coarse Roll Samples | Coarse Pitch Samples | Fine Step |
|---|---:|---:|---:|---:|
| Coarse | 9 | 3 | 3 | 10° |
| Default | 18 | 5 | 5 | 5° |
| Fine | 36 | 7 | 7 | 2.5° |

The experiments use the same 59 valid convex regions obtained from 70 original 3D region-generation attempts.

The robot is modeled as a 0.2 × 0.2 × 0.2 m box. The original 5 mm target-pose clearance and orientation-scoring formulation are preserved.

All three sampling settings use identical regions, robot geometry, initial poses, and candidate directions.

**Results**

| Setting | Actual Orientation Evaluations | Target Selection Time (ms) | Median (ms) | P95 (ms) | Feasible Targets | Forward Progress (m) |
|---|---:|---:|---:|---:|---:|---:|
| Coarse | 204.2 ± 12.9 | 4.863 ± 16.927 | 0.129 | 13.085 | 59/59 | 1.348 ± 0.105 |
| Default | 1125 | 27.422 ± 97.213 | 0.661 | 73.412 | 59/59 | 1.351 ± 0.106 |
| Fine | 5895 | 140.889 ± 502.942 | 3.415 | 378.166 | 59/59 | 1.352 ± 0.105 |

Continuous metrics are reported as mean ± sample standard deviation.

All three settings identify geometrically feasible target poses in the 59 successfully generated regions.

Including the 11 upstream region-generation failures, the overall target-availability rate is 59/70 (84.3%) for all three sampling settings.

The unsuccessful region-generation attempts are not attributed to orientation sampling.

**Runtime–Quality Trade-off**

Increasing orientation-sampling density substantially increases target-selection time.

However, the corresponding improvement in target quality is limited in the tested configurations.

Compared with Default sampling, Fine sampling:

- Increases the number of orientation evaluations from 1125 to 5895.
- Increases mean target-selection time from 27.422 ms to 140.889 ms.
- Increases average forward progress by only approximately 1.05 mm.
- Does not improve the observed feasible-target discovery rate.

The median paired runtime ratio between Fine and Default is 5.164.

These results indicate that denser orientation sampling provides limited additional forward progress in the tested regions despite substantially higher computational cost.

The feasible-target rate measures whether a target pose satisfying the generated region constraints and clearance requirement is found. It does not represent continuous trajectory safety or complete navigation success.

---

#### 5. Impact of Convex-Region Complexity

In addition to orientation-sampling density, the number of hyperplane constraints has a substantial influence on 3D target-selection time.

To isolate this effect, we analyze the 59 valid regions under the fixed Default configuration, which evaluates 1125 orientations per region.

| Hyperplane Count | Number of Regions | Target Selection Time (ms) | Median (ms) | P95 (ms) |
|---|---:|---:|---:|---:|
| 6–10 | 30 | 0.348 ± 0.137 | 0.302 | 0.607 |
| 11–20 | 14 | 1.835 ± 1.254 | 1.289 | 3.638 |
| 21–40 | 12 | 28.855 ± 17.356 | 22.784 | 55.602 |
| 41–72 | 3 | 411.844 ± 187.436 | 472.119 | 552.767 |

The observed Spearman correlations are:

- Hyperplane count vs. target-selection time: **0.9939**
- Valid three-plane combinations vs. target-selection time: **0.9973**

The three most computationally expensive regions account for 76.37% of the total measured target-selection time.

The most complex region contains 72 hyperplanes and 59,430 valid three-plane combinations.

Under Fine sampling, this region requires approximately 350 million candidate intersection evaluations, with a measured target-selection time of approximately 2.973 seconds.

These results identify combinatorial vertex enumeration over complex convex regions as a major computational bottleneck in 3D target-pose selection.

They also explain why target selection can be inexpensive in simple regions but substantially slower in highly constrained environments.

---

#### 6. Workspace Size and Local Computation

The proposed framework performs free-region generation and target-pose selection using locally cropped observations within a fixed sensing range of 3 m.

Therefore, for an identical local observation, robot configuration, and region complexity, increasing the dimensions of the global workspace does not directly increase the computational workload of a single local region-generation or target-selection operation.

However, larger environments may require more planning steps and produce larger navigation graphs.

The present benchmarks characterize the computational scalability of local planning modules. They do not establish workspace-independent complexity for the complete navigation system.

---

#### 7. Relation to the Runtime Reported in the Paper

The manuscript reports an average 3D target-pose selection time of 16.816 ms in its original module-runtime evaluation.

The additional orientation-sampling benchmark reports an average of 27.422 ms under the Default configuration.

These values were obtained under different evaluation conditions and should not be interpreted as direct repetitions of the same measurement.

The original result reflects module timing in the navigation experiments, whereas the supplementary benchmark evaluates fixed convex regions using repeated standalone timing measurements.

Detailed per-instance inputs and timing conditions from the original measurements are not fully available, preventing a controlled numerical reconciliation.

The supplementary results additionally reveal substantial runtime variability associated with geometric constraint complexity.

In particular, the Default configuration has:

- Mean: 27.422 ms
- Median: 0.661 ms
- P95: 73.412 ms

The difference between the mean and median is primarily associated with a small number of regions containing many hyperplane constraints.

---

#### 8. Summary

The scalability experiments support the following observations:

- **Free-region generation:** Computational cost increases with local obstacle-point count and the number of hyperplane-generation iterations. Each separating-hyperplane QP remains fixed in size for a fixed robot model.
- **Target-pose selection:** Runtime depends strongly on generated region complexity, particularly the number of hyperplane constraints.
- **3D orientation sampling:** The coarse-to-fine search limits the number of evaluated orientations compared with global fine-resolution sampling. Denser sampling substantially increases runtime but provides limited additional forward progress in the evaluated regions.
- **Local workspace dependence:** A fixed local sensing range limits the input of each local planning operation, although complete navigation complexity may still increase as the navigation graph grows.
- **Computational limitations:** Highly constrained 3D regions can produce substantial target-selection latency due to combinatorial vertex enumeration.

These results characterize both the computational advantages and the remaining scalability limitations of the proposed framework.

The reported timing benchmarks are module-level measurements and should not be interpreted as guarantees of bounded real-time latency or closed-loop navigation performance.
---

## Input Requirements

The planner requires the following inputs:

- **Sensor data (single-frame point cloud)**
  - 2D mode: `/scan`
  - 3D mode: `/velodyne_points`

- **Robot state (odometry)**
  - `/odom`

Make sure these topics are correctly published before launching the planner.

---

## Notes

- The provided `simple_robot` is only for demonstration. You can replace it with your own robot as long as the required topics are provided.
- The robot geometry (vertex representation) must be consistent with your actual robot to ensure correct collision checking.
- Ensure that the topic names match or are properly remapped if different in your system.

---

## Citation

If you find this project useful, please consider citing our paper:

**Continuous-Safe Navigation in Unknown Cluttered Environments via Direction-Aware Convex Free-Region Generation**

```bibtex
@article{song2026safe,
  title={Safe Navigation in Unknown and Cluttered Environments via Direction-Aware Convex Free-Region Generation},
  author={Song, Zhicheng and Li, Yongjian and Chen, Kai and Li, Yulin and Shi, Fan and Ma, Jun},
  journal={arXiv preprint arXiv:2604.23648},
  year={2026}
}
``` 
An additional related paper is available here:
```bibtex
@ARTICLE{10897898,
  author={Li, Yulin and Song, Zhicheng and Zheng, Chunxin and Bi, Zhihai and Chen, Kai and Wang, Michael Yu and Ma, Jun},
  journal={IEEE Robotics and Automation Letters}, 
  title={FRTree Planner: Robot Navigation in Cluttered and Unknown Environments With Tree of Free Regions}, 
  year={2025},
  volume={10},
  number={4},
  pages={3811-3818},
  keywords={Navigation;Robots;Collision avoidance;Robot sensing systems;Geometry;Trajectory optimization;Data mining;Feature extraction;Real-time systems;Space exploration;Mobile robot navigation;collision avoidance;trajectory optimization},
  doi={10.1109/LRA.2025.3544519}}

```
---

## Acknowledgment

This project builds upon the open-source project [DecompROS](https://github.com/sikang/DecompROS), which provides the foundation for convex decomposition and free-region generation.  
We thank the authors for making their implementation publicly available.

The `DecompROS` module in this repository has been adapted and modified to fit our framework.

---

<!-- ## License

This project is released under the license specified in the `LICENSE` file. -->
