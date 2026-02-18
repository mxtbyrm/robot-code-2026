# FRC Team 7840 — Robot Code 2026 "REBUILT"

Competition robot code for the 2026 FIRST Robotics Competition season. Built in Java on WPILib's command-based framework with AdvantageKit structured logging, targeting the NI RoboRIO 2.0.

> **~25,000 lines** of Java across 23 source files, 13 test files, and 7 vendor libraries.

---

## Table of Contents

- [Game Overview](#game-overview)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
- [Subsystems](#subsystems)
- [Commands & Controls](#commands--controls)
- [LED Feedback Reference](#led-feedback-reference)
- [Autonomous](#autonomous)
- [Vision](#vision)
- [Logging & Telemetry](#logging--telemetry)
- [Configuration Guidelines](#configuration-guidelines)
- [SysId Characterization](#sysid-characterization)
- [Pre-Match Checklist](#pre-match-checklist)
- [Testing](#testing)
- [Build & Deploy](#build--deploy)
- [Hardware Configuration](#hardware-configuration)
- [Project Structure](#project-structure)
- [Vendor Dependencies](#vendor-dependencies)

---

## Game Overview

**REBUILT (2026)** features two alliances competing to score FUEL balls (5.91-inch foam balls, ~215g) into a central HUB structure. The HUB alternates between Active and Inactive states on a timed shift schedule during Teleop. Robots collect FUEL from the field, depot stations, and the neutral zone, then score them via a flywheel shooter with an adjustable hood aimed by a turret.

### Scoring Summary

| Action | Points |
|---|---|
| FUEL in Active HUB (Auto or Teleop) | 1 pt |
| FUEL in Inactive HUB | 0 pts |
| Level 1 TOWER climb (Auto) | 15 pts |
| Level 1 TOWER climb (Teleop) | 10 pts |
| Level 2 TOWER climb (Teleop) | 20 pts |
| Level 3 TOWER climb (Teleop) | 30 pts |

### Ranking Points

| RP | Threshold |
|---|---|
| ENERGIZED | ≥ 100 FUEL in active HUBs |
| SUPERCHARGED | ≥ 360 FUEL in active HUBs |
| TRAVERSAL | ≥ 50 total TOWER points |

### Field Dimensions

- Field: 16.54 m × 8.07 m (651.2" × 317.7")
- Blue HUB Center: (4.03 m, 4.035 m)
- Red HUB Center: (12.51 m, 4.035 m)

### Teleop Timing (HUB Shift Schedule)

| Period | Duration | HUB State |
|---|---|---|
| Transition Shift | 10 s | Both HUBs Active |
| Shift 1 | 25 s | Alternating per auto result |
| Shift 2 | 25 s | Alternating |
| Shift 3 | 25 s | Alternating |
| Shift 4 | 25 s | Alternating |
| End Game | 30 s | Both HUBs Active |

> The alliance that scored **more FUEL in Autonomous** starts Teleop with its HUB **Inactive** in Shift 1. FUEL scored up to 3 seconds after deactivation still counts (grace period, Game Manual §6.5).

---

## Quick Start

**Prerequisites:** JDK 17, WPILib 2026 suite installed

```bash
# Clone and build
./gradlew build

# Run tests
./gradlew test

# Desktop simulation (no hardware needed)
./gradlew simulateJava

# Deploy to robot (USB or network)
./gradlew deploy
```

**First deployment setup:**
1. Set CANcoder offsets in `Constants.java` → `SwerveConstants` (`kFrontLeft/Right/BackLeft/RightEncoderOffset`)
2. Zero the turret at center (0°) before power-on
3. Zero the hood at minimum position (most open) before power-on
4. Run the **System Check** command from SmartDashboard before every match
5. Configure camera names in PhotonVision UI to match `VisionConstants`

---

## Architecture

### Execution Flow

```
Main.java
  └── RobotBase.startRobot(Robot::new)
        └── Robot (extends AdvantageKit LoggedRobot)
              ├── Logger setup (WPILOGWriter + NT4Publisher)
              ├── DataLogManager initialization
              ├── RobotState singleton (health monitoring)
              ├── HubStateTracker singleton (game state)
              └── RobotContainer
                    ├── Subsystem initialization (singleton pattern, order matters)
                    ├── ChoreoLib AutoFactory (alliance-mirrored)
                    ├── Controller bindings (driver + operator)
                    ├── Default commands
                    └── Autonomous chooser (SmartDashboard)
```

### Subsystem Initialization Order

Order matters — later subsystems depend on earlier ones via `Supplier<>` injection:

```
1. SwerveDrive.initialize()
2. Intake.initialize()
3. Spindexer.initialize()
4. Shooter.initialize(poseSupplier, fieldSpeedsSupplier)   ← needs SwerveDrive
5. Feeder.initialize(shooterReadySupplier, shooterRPSSupplier)  ← needs Shooter
6. Vision.initialize(measurementConsumer, poseSupplier)     ← needs SwerveDrive
7. Superstructure.initialize()                              ← needs all above
8. LEDs.initialize()                                        ← needs Superstructure, Shooter, Vision
```

### Key Design Patterns

- **Superstructure State Machine** — Central coordinator for Intake, Spindexer, Feeder, and Shooter. All operator actions flow through Superstructure command factories to prevent subsystem conflicts.
- **Singleton Subsystems** — Each subsystem has a static `initialize()` / `getInstance()` pattern with `resetInstance()` for test teardown.
- **Supplier Injection** — Pose and velocity data flows between subsystems via `Supplier<>` lambdas (e.g., `SwerveDrive::getPose` → Shooter auto-aim).
- **Health Monitoring** — `RobotState` singleton tracks battery voltage, CAN bus utilization, loop overruns, and per-subsystem health flags.
- **Pre-allocated Arrays** — All periodic arrays (module states, module positions) are pre-allocated at construction to avoid garbage collection in the robot loop.

---

## Subsystems

### SwerveDrive

4x SDS MK4i L2 swerve modules with Kraken X60 drive and steer motors, CANcoders for absolute steering. Pigeon2 IMU for field-oriented control.

**Key features:**
- Dedicated **250Hz odometry thread** (`Notifier`) for high-frequency pose updates independent of the 50Hz main loop
- `SwerveDrivePoseEstimator` for vision-fused odometry
- Second-order kinematics (trapezoidal midpoint prediction) to reduce trajectory lag
- `ChassisSpeeds.discretize()` to reduce rotation-induced drift
- Brownout protection: scales chassis speeds by voltage compensation factor
- Slip detection: compares commanded vs actual wheel speed, flags >30% discrepancy
- Gyro drift detection: compares Pigeon2 angular velocity vs kinematic angular velocity
- PathPlanner `AutoBuilder` configured at startup for on-the-fly pathfinding
- PathPlanner `PathfindingCommand.warmupCommand()` scheduled at init to avoid cold-start delay

**CAN IDs:** FL(1,2,3), FR(4,5,6), BL(7,8,9), BR(10,11,12), Pigeon2(13)

---

### SwerveModule

Individual MK4i module control (one instance per module, created by SwerveDrive).

**Key features:**
- **Cosine compensation** — scales drive speed by `cos(angleError)` to reduce scrubbing during large steering corrections
- **Anti-jitter threshold** — holds last steer angle when commanded drive speed < 0.05 m/s to prevent motor jitter at rest
- **Signal health monitoring** — detects stale CAN frames (signals older than 0.5s) and marks the module unhealthy
- All CAN status signals are cached and refreshed in batch (`BaseStatusSignal.refreshAll`) in the 250Hz odometry thread

---

### SwerveDriveSim

Desktop simulation support using WPILib `DCMotorSim` and Phoenix 6 simulation state APIs. Active only when `RobotBase.isSimulation()` is true. Provides realistic swerve module physics for `simulateJava` testing without hardware.

---

### Shooter

Three-axis aiming system with autonomous Hub tracking.

| Component | Motor | CAN ID | Control |
|---|---|---|---|
| Flywheel | Kraken X60 | 20 | Velocity (RPS) |
| Hood | Kraken X60 | 21 | Position (degrees) |
| Turret | Kraken X60 | 22 | Position (degrees) |

**Flywheel specs:**
- Bottom flywheel: 4" diameter, gear ratio 2:1
- Top flywheel: 2" diameter (geared 2× faster for equal surface speed)
- Tolerance: ±2 RPS at target before "ready"
- Idle pre-spin: 15 RPS (keeps flywheel warm for fast response)
- Max rated: 90 RPS (used for feeder speed normalization)

**Hood specs:**
- Travel: 27.5° – 42.5° (from horizontal)
- Gravity type: `Arm_Cosine` (kG = 0.15 V)
- Tolerance: ±0.5°
- Soft limits enabled; must be zeroed at minimum (most open) position at power-on

**Turret specs:**
- Travel: ±175° from center (soft limits; ContinuousWrap intentionally disabled)
- Tolerance: ±1°
- Must be zeroed at center (0°) at power-on
- **Wraparound protection:** when turret reaches 150°, sends a proportional rotation hint (`turretWraparoundHint`) to SwerveDriveCommand so the drivetrain auto-rotates to bring the target back into range

**Shooting pipeline:**
1. `updateHubCalculations()` runs every cycle, computing distance, turret angle, and shoot-while-moving compensation
2. Shoot-while-moving compensation: looks up time-of-flight from `timeOfFlightTable`, predicts robot future position, re-solves aim from there
3. Tables are generated by `ShooterPhysics.computeShootingTable()` at robot init — not hardcoded
4. `isReadyToShoot()` gates the feeder: flywheel at speed + hood at target + turret at target + in range + flywheel recovered from last shot

**Alliance zone dump mode:**
- Fixed parameters (no auto-aim): 35 RPS flywheel, 40° hood, 172° turret (facing backward)
- Activated via `enableAllianceZoneMode()` (operator right bumper)

---

### Superstructure

Central state machine coordinating all game piece mechanisms. **All operator inputs go through Superstructure command factories.**

```
                              ┌──────────────────────┐
                              │         IDLE          │
                              └──────────────────────┘
                                         │
              ┌──────────────────────────┼──────────────────────────┐
              ▼                          ▼                          ▼
       INTAKING                    SHOOTING                   OUTTAKING
              │                PRE_FEED_REVERSE                    │
              │                    ↕ (auto)                        │
              │                 SHOOTING                           │
              │              SHOOTING_ALLIANCE                     │
              │            SHOOTING_WHILE_INTAKING                 │
              └──────────────────────────┴──────────────────────────┘
                                         │
                                     UNJAMMING
                                         │
                                      DISABLED
```

**State behaviors:**
| State | Intake Arm | Roller | Spindexer | Feeder | Notes |
|---|---|---|---|---|---|
| IDLE | No change | Off | Off | Off | Shooter still auto-aims in its periodic() |
| INTAKING | No change | Forward | Off | Off | Roller only runs if arm is deployed |
| PRE_FEED_REVERSE | No change | Off | Reverse slow | Reverse slow | 120ms anti-jam pulse before feeding |
| SHOOTING | No change | Off | Forward (scaled) | Forward (gated) | Feeder gated by `isReadyToShoot()` + HUB state |
| SHOOTING_WHILE_INTAKING | No change | Forward | Forward (scaled) | Forward (gated) | Combined intake + shoot |
| SHOOTING_ALLIANCE | No change | Off | Forward (scaled) | Forward (gated) | Fixed shooter params |
| OUTTAKING | No change | Reverse | Reverse | Off | Arm must be deployed |
| UNJAMMING | No change | Reverse | Reverse | Reverse | Reverses everything |
| DISABLED | Stow + stop | Off | Off | Off | Emergency stop |

**Endgame auto-dump:** In the last 10 seconds of Teleop, Superstructure automatically enables shooter tracking and transitions to SHOOTING to dump all remaining FUEL. Operator can cancel via START (emergency stop) or BACK (idle) — once cancelled, the dump will not re-activate for the rest of the match.

**Ball counting:** The feeder beam break is the sole ball counter. Superstructure tracks rising edges from `Feeder.didBallPassThisCycle()`. Auto ball count is used as heuristic for HUB inactive-first determination at Teleop start.

---

### Intake

Slapdown-style intake with a powered arm and roller.

| Component | Motor | CAN ID | Control |
|---|---|---|---|
| Left deploy | Kraken X60 | 24 | Motion Magic (position) |
| Right deploy | Kraken X60 | 25 | Voltage mirror (asymmetric gravity FF) |
| Roller | Kraken X60 | 26 | Open-loop duty cycle |

**Asymmetric gravity compensation:**
The right motor carries more weight than the left due to off-center roller/linkage mass. Standard CTRE `Arm_Cosine` doesn't match the nonlinear slapdown linkage torque profile. Instead:
- Left motor runs Motion Magic + custom gravity feedforward from `InterpolatingDoubleTreeMap`
- Right motor reads the left motor's output voltage each cycle, subtracts left gravity FF, and adds its own (higher) gravity FF from a separate table
- Both tables are defined in `Constants.java → IntakeConstants` as `kDeployLeftGravityTable` / `kDeployRightGravityTable` and must be tuned on the real robot

**Positions:**
- Stowed: 0.0 mechanism rotations (inside bumpers)
- Deployed: −0.00323 mechanism rotations (past bumpers, ground contact)
- Hover: midpoint (partial deploy)

**Deploy speeds:** velocity 4.0 rot/s, acceleration 8.0 rot/s²
**Roller speeds:** intake 0.8 duty cycle, outtake −0.6 duty cycle
**Stall detection:** >25A for >25 consecutive cycles (~0.5s) triggers a warning

---

### Feeder

Ball transport from the spindexer hopper to the shooter flywheels.

| Component | CAN ID | Type |
|---|---|---|
| Feeder motor (Kraken X60) | 23 | Open-loop duty cycle |
| Beam break sensor | DIO 0 (TODO) | `DigitalInput` |

**Speed control:** Feeder speed = `kFeederSpeedRatio (0.75) × (shooterTargetRPS / kMaxFlywheelRPS)`. This creates a chain: Shooter RPS → Feeder duty → Spindexer duty, automatically enforcing the speed hierarchy.

**Feed gating:** Feeder only runs when `feedRequested == true` AND `Shooter.isReadyToShoot() == true`.

**Auto-unjam:** If stall is detected (>25A + <0.5 RPS for 10 cycles), feeder auto-reverses at −0.5 duty for 0.25 s, then returns to normal operation.

**Beam break logic:** Beam break is located between feeder and flywheels. When a ball passes: `!beamBreak.get() == true` (active-low). Rising edge detected as `ballPassedThisCycle` — read once per cycle by Superstructure for ball counting.

---

### Spindexer

Rotating hopper that indexes balls from intake toward the feeder.

| Component | CAN ID |
|---|---|
| Spindexer motor (Kraken X60) | 27 |

**Speeds:**
- Intake: 0.40 duty (slow, collecting)
- Feed: `kSpindexerToFeederRatio (0.65) × feeder_duty` (dynamically tracks feeder)
- Reverse: −0.30 duty (unjam/outtake)

**Speed hierarchy** (critical — prevents jams): Flywheel surface >> Feeder > Spindexer. The ratio chain automatically guarantees this.

**Auto-unjam:** If stall is detected (>25A + <0.3 RPS for 15 cycles), spindexer auto-reverses at −0.5 duty for 0.3 s then resumes at last commanded speed.

---

### Vision

Multi-camera AprilTag processing using PhotonVision (PhotonLib).

**Camera layout (6 cameras total):**

| Camera Name | Purpose | Position |
|---|---|---|
| `cam_front_left` | AprilTag pose estimation | Front-left corner, 45° outward, 15° up |
| `cam_front_right` | AprilTag pose estimation | Front-right corner, −45° outward, 15° up |
| `cam_back_left` | AprilTag pose estimation | Back-left corner, 135° outward, 15° up |
| `cam_back_right` | AprilTag pose estimation | Back-right corner, −135° outward, 15° up |
| `cam_intake` | Ball/object detection (ML) | Front center, below intake, 45° downward |
| `cam_side` | Situational awareness | Left side, centered, 5° upward, 90° yaw |

**Pose estimation pipeline (per AprilTag camera):**
```
PhotonLib result
  1. Skip if no targets
  2. Single-tag ambiguity filter (reject if ambiguity > 0.20)
  3. Multi-tag coprocessor estimate → fallback to lowest-ambiguity single-tag
  4. Timestamp freshness check (reject if |age| > 0.5s)
  5. Z-axis bounds (reject if z < −0.5m or > 1.5m)
  6. Field boundary check (reject if outside field + 1.0m margin)
  7. Pose-jump rejection (reject if jump > 2.0m from current odometry)
  8. Standard deviation scaling:
     - Multi-tag: (0.1, 0.1, 0.2) → then scaled by (1 + avgDist²/30)
     - Single-tag: (4.0, 4.0, 8.0) → rejected if avgDist > 5.5m
  └── SwerveDrive.addVisionMeasurement()
```

**HUB AprilTag IDs:**
- Blue HUB: Tags 2, 3, 4, 5, 8, 9, 10, 11 (2 per face × 4 faces)
- Red HUB: Tags 18, 19, 20, 21, 24, 25, 26, 27

**Ball detection (intake camera):** PhotonVision ML pipeline detects balls. Outputs `isBallDetected()`, `getBallYaw()`, `getBallPitch()`, `getBallArea()` used by `VisionIntakeCommand`.

**Distance validation:** When shooting, `Superstructure` calls `Vision.validateDistance(odometryDistance)`. Cross-references odometry-derived distance to HUB against direct camera-measured distance. Flags discrepancies > 1.0m.

---

### LEDs

AddressableLED strip on PWM port 0, 60 LEDs. Priority-based state display updated every robot cycle.

See [LED Feedback Reference](#led-feedback-reference) for the full priority table.

---

## Commands & Controls

### Driver Controller (Port 0) — Driving only

| Input | Action |
|---|---|
| Left Stick X/Y | Translation (field-relative) |
| Right Stick X | Rotation |
| **Right Trigger** (>0.3) | **Slow mode** (35% speed, 40% rotation) |
| **X Button** | **X-lock wheels** (defensive stance, hold) |
| **Y Button** | **Vision ball chase** + intake rollers (hold) |
| **START** | Zero gyro (reset field-relative forward) |
| D-Pad Up | Snap heading to 0° (away from DS) |
| D-Pad Right | Snap heading to −90° (face right) |
| D-Pad Down | Snap heading to 180° (face DS) |
| D-Pad Left | Snap heading to 90° (face left) |

### Operator Controller (Port 1) — All mechanisms

| Input | Action |
|---|---|
| **Right Trigger** (>0.3) | **Shoot** — enables Hub tracking + feed (hold) |
| **Left Trigger** (>0.3) | **Intake rollers** — hold to collect (arm must be deployed via A first) |
| **A Button** | **Intake arm toggle** — deploy/stow (press) |
| **Right Bumper** | **Alliance zone dump** — lob shot into alliance zone (hold) |
| **Left Bumper** | **Outtake** — deploy + reverse rollers (hold) |
| **B Button** | **Unjam** — reverse everything (hold) |
| **START** | Emergency stop all mechanisms |
| **BACK** | Re-enable after emergency stop |
| D-Pad Up | Hood angle trim +2° (additive to auto-aim, clamped ±4°) |
| D-Pad Down | Hood angle trim −2° |

### Teleop Drive Features

- **Heading Lock** — `ProfiledPIDController` (P=5.0, D=0.3) captures heading when rotation stick is released, holds it until driver rotates again
- **Slow Mode** — Right trigger reduces translation to 35% and rotation to 40% of max
- **Snap-to-Angle** — D-pad drives heading PID to exact cardinal angles (0/90/180/270°)
- **Slew Rate Limiting** — Translation limited to 6 m/s², rotation to 8 rad/s² to prevent wheel slip
- **Squared Inputs** — Joystick values are squared (preserving sign) for finer low-speed control
- **Intake Speed Cap** — While intaking, chassis translation is capped to 85% of roller surface speed to prevent balls being pushed away
- **Turret Wraparound Assist** — When turret nears ±150°, a proportional rotation hint is added to driver rotation input (or overrides heading lock) to auto-rotate the robot back into turret range

---

## LED Feedback Reference

Priority order (highest to lowest):

| Priority | Condition | Pattern | Color |
|---|---|---|---|
| 1 | Any subsystem unhealthy | Fast flash (5 Hz) | Red |
| 2 | Low battery (<7.0V) | Pulse (2 Hz) | Orange-Red |
| 2.5 | <2 AprilTag cameras connected | Alternating | Cyan / Red |
| 2.6 | Turret at hard limit | Rapid strobe (12 Hz) | Orange |
| 2.7 | Brownout predicted | Strobe (8 Hz) | Orange |
| 2.8 | Turret near limit (>150°) | Pulse (4 Hz) | Orange |
| 3 | Disabled | Breathe (1.5 Hz) | Alliance color |
| 4 | Shooting + ready to shoot | Strobe (10 Hz) | Green |
| 4.1 | Alliance zone dump + ready | Strobe (10 Hz) | Purple |
| 4.2 | Shooting + waiting for lock | Pulse (3 Hz) | Yellow |
| 5 | Intaking + ball detected | Flash (6 Hz) | Cyan |
| 6 | Intaking | Chase | Blue |
| 7 | Outtaking | Chase | Orange |
| 8 | Unjamming | Flash (3 Hz) | Red |
| 9 | Idle + tracking + ready | Solid | Green |
| 10 | Idle + tracking + not locked | Pulse (3 Hz) | Magenta |
| 11 | Idle + in range, not tracking | Pulse (2 Hz) | Green |
| 12 | Idle | Solid | Alliance color |

---

## Autonomous

Two autonomous systems coexist, selectable from the SmartDashboard `"Auto Chooser"`:

### PathPlanner (Primary — `[PP]` prefix)

Dynamic pathfinding using PathPlanner's AD* algorithm with `navgrid.json` for obstacle awareness. Paths are computed at runtime using `AutoBuilder.pathfindToPose()`.

| Routine | Description | Expected Score |
|---|---|---|
| `[PP] Preload & Park` | Score preload, park in neutral zone facing hub | ~8 pts |
| `[PP] Preload & Collect Center` | Score preload, collect neutral zone center, score again | ~12–16 pts |
| `[PP] Preload & Collect Side` | Score preload, collect near trench guardrail, score again | ~12–16 pts |
| `[PP] Preload & Collect Depot` | Score preload, collect from depot (24 FUEL pre-staged), score again | ~12–16 pts |
| `[PP] Double Trench Score` | Two trench crossings: score → collect → score → position for teleop | ~14–20 pts |
| `[PP] Aggressive Sweep` | Sweep near-trench → center → far-trench, score all | ~16–24 pts |
| `[PP] Defensive Preload` | Score preload, park near hub to deny opponent access | ~8 pts |

**PathPlanner configuration:**
- Max velocity: 3.0 m/s (default), 1.5 m/s (precise)
- Max acceleration: 2.5 m/s² (default), 1.5 m/s² (precise)
- Max angular velocity: 540°/s; max angular acceleration: 720°/s²
- Pathfind timeout: 6 s (default), 4 s (short/precise)
- Robot config loaded from PathPlanner GUI (`RobotConfig.fromGUISettings()`)
- Coordinate system: **always Blue Alliance frame**; AutoBuilder flips to Red automatically

**Flywheel pre-spin:** The `driveToShootingRange()` helper overlaps flywheel spin-up with drive time, saving 0.5–1.0 s per scoring cycle.

### ChoreoLib (Fallback — `[Choreo]` prefix)

Pre-planned trajectories designed in the Choreo GUI. Alliance-mirrored automatically for Red/Blue symmetry.

| Routine | Required Trajectory Files | Description |
|---|---|---|
| `[Choreo] Preload & Park` | `preload.1`, `preload.2` | Safe fallback |
| `[Choreo] Center 2-Cycle` | `center2.1–3` | Two scoring cycles from center |
| `[Choreo] Left 2-Cycle` | `left2.1–3` | Two cycles from left start |
| `[Choreo] Right 2-Cycle` | `right2.1–3` | Two cycles from right start |
| `[Choreo] Center 3-Cycle` | `center3.1–5` | Three cycles (tight on 20s) |
| `[Choreo] Shoot While Moving` | `swm.1`, `swm.2` | Continuous intake + auto-fire |
| `[Choreo] Depot Cycle` | `depot.1–3` | Depot ball collection |

**Choreo trajectory files** go in `src/main/deploy/choreo/`. The auto factory is configured with `allianceFlipping = true`.

> **Note:** Choreo routines require `.traj` files to be exported from the Choreo GUI before deploying. Currently marked as deprecated — PathPlanner is the primary system.

### Shoot-While-Moving Hysteresis (Choreo)

The `intakeAndShootWhileDriving()` helper prevents feeder jitter at the edge of shooting range:
- **5 consecutive "ready" cycles** required before switching to shoot mode
- **10 cycle minimum hold** in shoot mode before dropping back to intake on lock loss

### Pre-Match System Check

Run `"System Check"` from SmartDashboard before every match. Results appear as booleans under `SystemCheck/`:

1. **Swerve** — Drives briefly, checks all modules healthy
2. **Intake Deploy** — Deploys arm, verifies position reached within 0.5 s
3. **Spindexer** — Runs briefly, records baseline current (empty hopper)
4. **Feeder** — Runs briefly, records baseline current, calibrates `BallPresenceEstimator`
5. **Shooter** — Spins flywheel briefly, checks health flags
6. **Vision** — Counts connected AprilTag cameras (minimum 1 to pass)

---

## Vision

### PhotonVision Camera Setup

All 6 cameras must be configured in the PhotonVision web UI before use:

1. Open PhotonVision UI (robot IP:5800 by default)
2. Name each camera **exactly** as listed in `VisionConstants`:
   - `cam_front_left`, `cam_front_right`, `cam_back_left`, `cam_back_right`
   - `cam_intake`, `cam_side`
3. Set the 4 AprilTag cameras to the **AprilTag pipeline**
4. Set `cam_intake` to an **ML object detection pipeline** trained on FUEL balls
5. Set `cam_side` to your preferred pipeline (AprilTag or color)
6. Verify camera-to-robot transforms match `VisionConstants.kRobotToFrontLeftCam` etc.

### Camera Transform Convention

```java
Transform3d = new Transform3d(
    new Translation3d(x_forward, y_left, z_up),  // meters
    new Rotation3d(roll, pitch, yaw)              // radians
);
```

AprilTag cameras are at the robot corners, 45° outward facing:
- `kAprilTagCamHeightMeters = 12.0 inches` (adjust to actual mounting height)
- `kAprilTagCamPitchRadians = -15°` (looking up at tags on the HUB/walls)

### Pose Standard Deviations

Lower = more trust in vision measurement:

| Condition | Std Devs (x, y, heading) |
|---|---|
| Single tag | (4.0, 4.0, 8.0) |
| Multi-tag | (0.1, 0.1, 0.2) → scaled by distance |
| Single tag beyond 5.5m | Rejected (MAX_VALUE) |

---

## Logging & Telemetry

### AdvantageKit

- **Real robot:** Logs to USB stick (`WPILOGWriter`) + NetworkTables (`NT4Publisher`)
- **Simulation:** NetworkTables only
- **Replay:** Full deterministic replay from `.wpilog` files for post-match analysis in AdvantageScope

All subsystems use `Logger.recordOutput("Key", value)` for structured telemetry.

### Key Log Paths

| Path | Description |
|---|---|
| `Swerve/Pose` | Robot field position (Pose2d) |
| `Swerve/ModuleStates` | All 4 module states (velocity, angle) |
| `Swerve/GyroDriftWarning` | True if gyro vs kinematics omega disagrees >0.5 rad/s |
| `Shooter/DistanceToHub` | Odometry-based distance to Hub |
| `Shooter/CompensatedDistance` | Shoot-while-moving compensated distance |
| `Shooter/ReadyToShoot` | All shooter systems locked on |
| `Shooter/TurretWraparoundHint` | Rotation hint for drivetrain |
| `Shooter/TurretPose3d` | 3D turret visualization for AdvantageScope |
| `Shooter/HoodPose3d` | 3D hood visualization for AdvantageScope |
| `Super/State` | Current Superstructure state name |
| `Super/BallsShot` | Running ball count (beam break) |
| `Super/HubActive` | HUB currently active |
| `Super/HubNextShift` | Seconds until next shift boundary |
| `HUB/Status` | ACTIVE / INACTIVE / UNKNOWN |
| `HUB/ShouldShoot` | Strategic shoot decision |
| `Vision/ConnectedATCameras` | Number of connected AprilTag cameras |
| `Vision/BallDetected` | Ball visible in intake camera |
| `Vision/DistanceDiscrepancy` | Odometry vs vision distance delta |
| `BallEst/HopperEmpty` | Current-based hopper empty flag |
| `BallEst/HopperLoadLevel` | 0.0–1.0 fullness estimate |
| `RobotState/BatteryVoltage` | Battery voltage |
| `RobotState/CANUtilization` | CAN bus % utilization |
| `RobotState/LoopOverruns` | Loop overrun count (>25ms) |
| `Intake/LeftGravityFF` | Left arm gravity feedforward (V) |
| `Intake/RightGravityFF` | Right arm gravity feedforward (V) |

### WPILib DataLogManager

All SmartDashboard values and DriverStation data automatically logged to the USB data log file (separate from AdvantageKit).

---

## Configuration Guidelines

This section documents all values that **must be measured/tuned on the real robot** before competition. Items marked `TODO` in the source code are listed here.

### 1. CANcoder Offsets (Swerve)

After installing modules, calibrate each CANcoder offset so wheels point forward:

1. Power on the robot with all wheels physically aligned forward
2. Open Phoenix Tuner X → read each CANcoder's absolute position (rotations)
3. Set the offsets in `Constants.java → SwerveConstants`:

```java
public static final double kFrontLeftEncoderOffset  = 0.0; // TODO: measure
public static final double kFrontRightEncoderOffset = 0.0; // TODO: measure
public static final double kBackLeftEncoderOffset   = 0.0; // TODO: measure
public static final double kBackRightEncoderOffset  = 0.0; // TODO: measure
```

The offset is the negative of the raw reading when wheels point forward (in rotations).

### 2. Chassis Dimensions

Measure center-to-center distance between swerve module axles:

```java
public static final double kTrackWidthMeters = Units.inchesToMeters(22.75); // left-right
public static final double kWheelBaseMeters  = Units.inchesToMeters(22.75); // front-back
```

### 3. Swerve Drive PID / FF Tuning

Use `SysIdCommands.createDriveRoutine()` to characterize motors:

```java
// Drive motor (velocity control)
public static final double kDriveP = 0.1;   // tune: increase if slow to reach setpoint
public static final double kDriveI = 0.0;
public static final double kDriveD = 0.0;
public static final double kDriveS = 0.0;   // static friction (V) — from SysId
public static final double kDriveV = 0.12;  // velocity FF (V·s/rot) — from SysId

// Steer motor (position control)
public static final double kSteerP = 100.0; // tune: increase for faster response
public static final double kSteerI = 0.0;
public static final double kSteerD = 0.5;
```

**Heading lock PID** (holds heading when rotation stick released):
```java
public static final double kHeadingLockP = 5.0;   // too high = oscillation
public static final double kHeadingLockD = 0.3;
```

### 4. Shooter — Hood Encoder Zeroing

The hood encoder is zeroed at **minimum angle (most open, 27.5°)** at power-on.

**Important:** Physical setup required before each match:
1. Manually move the hood to its hard stop at the minimum (fully open) position
2. Power on the robot — `configureHoodMotor()` calls `hoodMotor.setPosition(kHoodMinRotations)`
3. Verify the DriverStation console shows no HOMING ERROR (hood should read ≈27.5°)

To calibrate `kHoodMinMotorRotations` and `kHoodMaxMotorRotations`:
```java
// TODO in Constants.java → ShooterConstants:
private static final double kHoodMinMotorRotations = 0.0; // move hood to min, read encoder
private static final double kHoodMaxMotorRotations = 0.0; // move hood to max, read encoder
```

### 5. Shooter — Turret Encoder Zeroing

The turret encoder is zeroed at **center (0°)** at power-on.

**Important:** Physical setup required before each match:
1. Manually center the turret at 0° (use a mechanical index/detent or alignment mark)
2. Power on the robot — `configureTurretMotor()` calls `turretMotor.setPosition(0)`
3. Verify the DriverStation console shows no HOMING ERROR

To calibrate `kTurretCCWLimitMotorRotations` and `kTurretCWLimitMotorRotations`:
```java
// TODO in Constants.java → ShooterConstants:
private static final double kTurretCCWLimitMotorRotations = 0.0; // move to CCW stop, read encoder
private static final double kTurretCWLimitMotorRotations  = 0.0; // move to CW stop, read encoder
public static final double kTurretGearRatio = 100.0; // TODO: measure actual gear ratio
```

### 6. Shooter — Flywheel / Hood / Turret PID Tuning

Use `SysIdCommands` for characterization:

```java
// Flywheel (velocity control)
public static final double kFlywheelP = 0.5;
public static final double kFlywheelS = 0.0;  // from SysId quasistatic
public static final double kFlywheelV = 0.12; // from SysId quasistatic
public static final double kFlywheelToleranceRPS = 2.0; // tighten if shots miss

// Hood (position + gravity)
public static final double kHoodP = 8.0;
public static final double kHoodG = 0.15; // Arm_Cosine gravity FF — tune to hold at 45°

// Turret (position)
public static final double kTurretP = 30.0;
```

### 7. Shooter — Physics Calibration

`ShooterPhysics.java` generates the lookup table from physical constants. Tune in this order:

1. **`kShooterExitHeightMeters`** — Measure carpet to ball exit point in meters:
   ```java
   public static final double kShooterExitHeightMeters = Units.inchesToMeters(24.0); // TODO
   ```

2. **`kShooterEfficiencyFactor`** — Ratio of ball exit speed to flywheel surface speed (0.0–1.0). Start at 0.30; adjust until shots land in Hub at close range:
   ```java
   public static final double kShooterEfficiencyFactor = 0.30; // tune
   ```

3. **`kBiasFactorNear` / `kBiasFactorFar`** — Distance-dependent real-world correction:
   - Start both at 1.00 (pure physics)
   - Shoot at minimum distance (1.2m): if short, increase `kBiasFactorNear`
   - Shoot at maximum distance (6.5m): if short, increase `kBiasFactorFar`
   - Mid-range interpolates automatically
   ```java
   public static final double kBiasFactorNear = 1.00;
   public static final double kBiasFactorFar  = 1.00;
   ```

4. **`kMinApexClearanceMeters`** — Minimum height the ball must peak above the Hub opening (basketball arc). Increase if balls hit the rim:
   ```java
   public static final double kMinApexClearanceMeters = 0.30; // ~12 inches
   ```

5. Verify with AdvantageScope: `ShooterPhysics/Descent Angle @Xm` — aim for >40° at all distances.

### 8. Shooter — Alliance Zone Dump Tuning

Tune these fixed parameters on the real robot to land balls in the alliance zone:
```java
public static final double kAllianceZoneFlywheelRPS    = 35.0;  // low-power lob
public static final double kAllianceZoneHoodAngleDeg   = 40.0;  // steep arc
public static final double kAllianceZoneTurretAngleDeg = 172.0; // facing backward
public static final double kAllianceZoneFeederSpeed    = 0.60;  // moderate feed
```

### 9. Intake — Gravity Feedforward Tables

The left and right gravity tables map arm position (mechanism rotations) to feedforward voltage. Tune using AdvantageScope current plots with the arm at each position:

```java
// Constants.java → IntakeConstants
public static final double[][] kDeployLeftGravityTable = {
    { 0.0,                              0.0  },  // stowed
    { kDeployExtendedRotations * 0.25,  0.10 },  // 25% deploy
    { kDeployExtendedRotations * 0.50,  0.25 },  // 50% deploy
    { kDeployExtendedRotations * 0.75,  0.35 },  // 75% deploy
    { kDeployExtendedRotations,          0.40 },  // full deploy
};
// Right table has higher voltages (more weight)
```

**Tuning process:** Move arm to each position manually, observe current to hold it still, convert to voltage and enter into the table.

### 10. Intake — Deploy Motor Measurements

```java
// Move intake to stowed position, read left motor encoder:
private static final double kStowedMotorRotations   = 0.0;    // measured
// Move intake to fully deployed, read left motor encoder:
private static final double kExtendedMotorRotations = -0.01293; // measured — verify on robot
```

### 11. Intake — Roller / Chassis Speed Limit

```java
public static final double kRollerDiameterMeters = Units.inchesToMeters(2.0); // TODO: measure
public static final double kRollerGearRatio      = 5.0;                       // TODO: verify
```
After measuring, the chassis speed limit during intake (`kMaxChassisSpeedWhileIntaking`) is automatically recomputed as 85% of roller surface speed.

### 12. Feeder — Beam Break DIO Port

```java
public static final int kBeamBreakDIOPort = 0; // TODO: set to actual DIO port on RoboRIO
```

### 13. Vision — Camera Transforms

Adjust camera positions/angles in `Constants.java → VisionConstants` if cameras move:

```java
private static final double kAprilTagCamHeightMeters  = Units.inchesToMeters(12.0); // adjust
private static final double kAprilTagCamPitchRadians  = Units.degreesToRadians(-15.0); // adjust
```

### 14. Vision — Standard Deviations Tuning

Lower values = more trust in vision. Tune based on field testing:

```java
// Multi-tag (very reliable — use tight std devs)
public static final Matrix<N3, N1> kAggressiveMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);

// Single-tag (less reliable)
public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
```

### 15. Ball Presence Estimator Calibration

Baselines are automatically calibrated during the **System Check** (run from SmartDashboard before each match). Default values are rough estimates:

```java
// BallDetectionConstants — tune from real robot with empty hopper
public static final double kDefaultSpindexerBaselineCurrent = 3.0; // amps empty
public static final double kDefaultFeederBaselineCurrent    = 2.5; // amps empty

// After measuring: current delta when 1 ball is in hopper
public static final double kSpindexerLoadedThreshold = 2.0; // amps above baseline
public static final double kSpindexerEmptyThreshold  = 0.5; // amps above baseline
public static final double kSpindexerFullLoadDelta   = 8.0; // amps with full hopper (8 balls)
public static final double kFeederBallPresentThreshold = 3.0; // amps above baseline
```

### 16. PathPlanner Robot Config

PathPlanner reads robot config from the PathPlanner GUI settings file (`src/main/deploy/pathplanner/`). Set the following in the PathPlanner app:
- Robot mass: ~54 kg
- Track width: 0.578m (22.75")
- Wheelbase: 0.578m (22.75")
- Bumper dimensions: ~0.813m × 0.813m
- Max velocity: 4.5 m/s (or `kMaxSpeedMetersPerSecond`)
- Max acceleration: 3.0 m/s²

---

## SysId Characterization

`SysIdCommands.java` provides WPILib SysId routines for characterizing all motors. Bind commands to a test controller to run:

### Available Routines

| Routine | Motor | Use For |
|---|---|---|
| `createDriveRoutine(drive)` | Swerve drive motors | kS, kV, kA feedforward |
| `createFlywheelRoutine(shooter)` | Flywheel | kS, kV feedforward |
| `createHoodRoutine(shooter)` | Hood | kG gravity, kP position |
| `createTurretRoutine(shooter)` | Turret | kP position |
| `createIntakeDeployRoutine(intake)` | Left deploy motor | kG gravity, kP position |

### SysId Procedure

1. Add SysId command bindings to a test-mode controller in `RobotContainer`
2. Enable Test mode in DriverStation
3. Run **quasistatic forward** → **quasistatic reverse** → **dynamic forward** → **dynamic reverse**
4. Collect the `.wpilog` file from USB
5. Open in WPILib SysId tool, export kS, kV, kA values
6. Update `Constants.java` with the measured values

---

## Pre-Match Checklist

Run this checklist before each match (in order):

- [ ] **Battery:** Full charge, secure terminals
- [ ] **Turret:** Physically centered at 0° before powering on
- [ ] **Hood:** Physically at minimum position (most open) before powering on
- [ ] **Power on robot**
- [ ] **System Check:** Run `"System Check"` from SmartDashboard — all `SystemCheck/*` booleans must be `true`
- [ ] **Gyro zero:** Press START on driver controller after robot is placed on field in starting position
- [ ] **Pose reset:** If auto is selected, starting pose is reset by the trajectory's `resetOdometry()` call
- [ ] **Auto chooser:** Verify correct auto routine is selected in `"Auto Chooser"` SmartDashboard
- [ ] **Auto override:** If FMS is not attached, verify `"Auto/Use Override"` is `false` (or set manually if needed)
- [ ] **Vision:** Verify `SystemCheck/Vision Cameras` ≥ 2 (prefer all 4 for max coverage)
- [ ] **LEDs:** Should show solid alliance color (idle) before enable
- [ ] **Hood trim:** Verify `Shooter/HoodTrimDeg` is 0.0 (reset between matches)

---

## Testing

13 JUnit 5 test files covering all subsystems and utilities. Tests run without hardware.

```bash
./gradlew test    # Run all tests
```

| Test File | Coverage |
|---|---|
| `IntakeTest` | Gravity table interpolation/monotonicity, deploy positions, tolerance, stall detection, roller speeds, PID constants |
| `ShooterTest` | Flywheel lookup tables, hood angle interpolation, turret wrapping, shoot-while-moving compensation |
| `VisionTest` | Pose Z-axis bounds, field boundaries, pose-jump rejection, timestamp filtering, std dev scaling, ambiguity |
| `SwerveDriveTest` | Slip detection, gyro drift, second-order prediction, voltage compensation, X-lock pattern |
| `SwerveModuleTest` | Cosine compensation, anti-jitter threshold, signal health, drive position conversions |
| `FeederTest` | Feed speed ratio, beam break edge detection, stall detection |
| `SpindexerTest` | Running state threshold, stall detection, speed hierarchy, unjam timing |
| `LEDsTest` | Flash, pulse, breathe, strobe, chase animation math |
| `SuperstructureTest` | State machine transitions |
| `ShooterTableTest` | Shooter physics lookup table generation |
| `RobotStateTest` | Health monitoring, brownout detection |
| `HubStateTrackerTest` | Hub shift schedule, FMS data parsing |
| `BallPresenceEstimatorTest` | Current baseline calibration, ball detection thresholds |

---

## Build & Deploy

**Prerequisites:** JDK 17, WPILib 2026 suite

```bash
./gradlew build              # Compile + run all tests
./gradlew test               # Run JUnit 5 tests only
./gradlew simulateJava       # Desktop simulation with GUI + DriverStation
./gradlew deploy             # Deploy to RoboRIO (USB or network)
./gradlew clean              # Remove build artifacts
```

### Simulation

`simulateJava` launches the WPILib SimGUI with a simulated DriverStation. `SwerveDriveSim` provides realistic swerve module physics using `DCMotorSim` and Phoenix 6 simulation APIs. PathPlanner pathfinding runs in simulation. Vision is not simulated (cameras return no results).

### Build Configuration (`build.gradle`)

- Java 17 source/target compatibility
- `includeDesktopSupport = true` — enables simulation
- GradleRIO version `2026.2.1`
- Fat JAR: bundles all vendor libraries + source backup into the deployed JAR
- JUnit 5.10.1 for tests

---

## Hardware Configuration

### Drivetrain

| Component | Specification |
|---|---|
| Modules | 4× SDS MK4i L2 |
| Drive Motors | Kraken X60 (TalonFX) |
| Steer Motors | Kraken X60 (TalonFX) |
| Encoders | CANcoder (absolute) |
| IMU | Pigeon2 |
| Drive Gear Ratio | 6.75:1 |
| Steer Gear Ratio | 21.43:1 (150/7) |
| Wheel Diameter | 4 inches (0.1016 m) |
| Track Width | 22.75 inches (0.578 m) |
| Wheelbase | 22.75 inches (0.578 m) |
| Max Theoretical Speed | ~4.75 m/s |
| Effective Max Speed | ~4.5 m/s (95% of theoretical) |
| CAN Bus | CANivore (`"CANivore"`) |
| Odometry Rate | 250 Hz |

### CAN Bus IDs — Complete Reference

| Device | ID |
|---|---|
| Front Left Drive | 1 |
| Front Left Steer | 2 |
| Front Left CANcoder | 3 |
| Front Right Drive | 4 |
| Front Right Steer | 5 |
| Front Right CANcoder | 6 |
| Back Left Drive | 7 |
| Back Left Steer | 8 |
| Back Left CANcoder | 9 |
| Back Right Drive | 10 |
| Back Right Steer | 11 |
| Back Right CANcoder | 12 |
| Pigeon2 IMU | 13 |
| Flywheel Motor | 20 |
| Hood Motor | 21 |
| Turret Motor | 22 |
| Feeder Motor | 23 |
| Left Deploy Motor | 24 |
| Right Deploy Motor | 25 |
| Roller Motor | 26 |
| Spindexer Motor | 27 |

### Digital I/O (RoboRIO)

| Port | Device |
|---|---|
| DIO 0 | Feeder beam break (TODO: verify) |

### PWM (RoboRIO)

| Port | Device |
|---|---|
| PWM 0 | Addressable LED strip (60 LEDs) |

### Current Limits

| Motor | Supply Limit | Stator Limit |
|---|---|---|
| Drive (per module) | 60 A | 80 A |
| Steer (per module) | 30 A | — |
| Flywheel | 80 A | — |
| Hood | 30 A | — |
| Turret | 30 A | — |
| Feeder | 30 A | — |
| Left/Right Deploy | 30 A each | — |
| Roller | 40 A | — |
| Spindexer | 30 A | — |

---

## Project Structure

```
robot2026/
├── src/
│   ├── main/
│   │   ├── java/frc/robot/
│   │   │   ├── Main.java                    # Entry point
│   │   │   ├── Robot.java                   # Lifecycle (LoggedRobot)
│   │   │   ├── RobotContainer.java          # Assembly, bindings, auto chooser
│   │   │   ├── Constants.java               # All tunable values (~1,073 lines)
│   │   │   ├── ShooterPhysics.java          # Projectile motion + table generation
│   │   │   ├── HubStateTracker.java         # Game state (Active/Inactive shift schedule)
│   │   │   ├── RobotState.java              # Centralized health monitoring
│   │   │   ├── BallPresenceEstimator.java   # Sensorless ball detection (current-based)
│   │   │   ├── subsystems/
│   │   │   │   ├── SwerveDrive.java         # Swerve drivetrain, odometry, PathPlanner
│   │   │   │   ├── SwerveModule.java        # Individual MK4i module control
│   │   │   │   ├── SwerveDriveSim.java      # Desktop simulation support
│   │   │   │   ├── Shooter.java             # Flywheel + Hood + Turret, auto-aim
│   │   │   │   ├── Superstructure.java      # State machine coordinator
│   │   │   │   ├── Intake.java              # Slapdown arm + rollers
│   │   │   │   ├── Feeder.java              # Ball transport + beam break
│   │   │   │   ├── Spindexer.java           # Ball indexing hopper
│   │   │   │   ├── Vision.java              # AprilTag processing (6 cameras)
│   │   │   │   └── LEDs.java                # Driver feedback (priority-based)
│   │   │   └── commands/
│   │   │       ├── SwerveDriveCommand.java  # Teleop driving (heading lock, slow mode)
│   │   │       ├── Autos.java               # ChoreoLib trajectory routines
│   │   │       ├── OnTheFlyAutos.java       # PathPlanner on-the-fly pathfinding
│   │   │       ├── VisionIntakeCommand.java # Vision-assisted ball chase
│   │   │       └── SysIdCommands.java       # Motor characterization (SysId)
│   │   └── deploy/
│   │       ├── example.txt
│   │       └── pathplanner/
│   │           └── navgrid.json             # Field navigation grid (obstacles for AD*)
│   └── test/
│       └── java/frc/robot/
│           ├── HubStateTrackerTest.java
│           ├── ShooterTableTest.java
│           ├── BallPresenceEstimatorTest.java
│           ├── RobotStateTest.java
│           ├── FeederTest.java
│           ├── SpindexerTest.java
│           ├── SwerveDriveTest.java
│           ├── LEDsTest.java
│           ├── SwerveModuleTest.java
│           ├── VisionTest.java
│           ├── ShooterTest.java
│           ├── IntakeTest.java
│           └── SuperstructureTest.java
├── vendordeps/                               # Vendor dependency JSON files
├── build.gradle                              # GradleRIO 2026.2.1 build config
├── settings.gradle                           # Gradle settings
├── CLAUDE.md                                 # AI assistant context file
└── WPILib-License.md                         # WPILib BSD license
```

---

## Vendor Dependencies

| Library | Version | Purpose |
|---|---|---|
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | 26.0.0 | Structured logging with deterministic replay |
| [Phoenix 6](https://pro.docs.ctr-electronics.com/) | 26.1.1 | CTRE hardware (TalonFX, CANcoder, Pigeon2) |
| [PhotonLib](https://docs.photonvision.org/) | v2026.2.2 | AprilTag vision processing + object detection |
| [REVLib](https://docs.revrobotics.com/) | 2026.0.1 | REV Robotics hardware support |
| [PathplannerLib](https://pathplanner.dev/) | 2026.1.2 | Dynamic pathfinding (AD* algorithm) |
| [ChoreoLib](https://choreo.autos/) | 2026.0.1 | Pre-planned trajectory following |
| WPILibNewCommands | 1.0.0 | Command-based framework |

Vendor JSON files are in `vendordeps/`. Update via WPILib VS Code extension or Phoenix Tuner X.

---

## License

This project is open source under the WPILib BSD license. See `WPILib-License.md`.

---

*FRC Team 7840 — 2026 Season*
