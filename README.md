# FRC Team 7840 — Robot Code 2026 "REBUILT"

Competition robot code for the 2026 FIRST Robotics Competition season. Built in Java on WPILib's command-based framework with AdvantageKit structured logging, targeting the NI RoboRIO 2.0.

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
- [Robot Configuration](#robot-configuration)
- [SysId Characterization](#sysid-characterization)
- [Pre-Match Checklist](#pre-match-checklist)
- [Testing](#testing)
- [Build & Deploy](#build--deploy)
- [Hardware Reference](#hardware-reference)
- [Project Structure](#project-structure)
- [Vendor Dependencies](#vendor-dependencies)

---

## Game Overview

**REBUILT (2026)** features two alliances competing to score FUEL balls (5.91-inch foam balls, ~215g) into a central HUB structure. The HUB alternates between Active and Inactive states on a timed shift schedule during Teleop. Robots collect FUEL from the field, depot stations, and the neutral zone, then score them via a flywheel shooter with an adjustable hood aimed by a turret.

### Scoring Summary

| Action                              | Points |
| ----------------------------------- | ------ |
| FUEL in Active HUB (Auto or Teleop) | 1 pt   |
| FUEL in Inactive HUB                | 0 pts  |
| Level 1 TOWER climb (Auto)          | 15 pts |
| Level 1 TOWER climb (Teleop)        | 10 pts |
| Level 2 TOWER climb (Teleop)        | 20 pts |
| Level 3 TOWER climb (Teleop)        | 30 pts |

### Ranking Points

| RP           | Threshold                 |
| ------------ | ------------------------- |
| ENERGIZED    | ≥ 100 FUEL in active HUBs |
| SUPERCHARGED | ≥ 360 FUEL in active HUBs |
| TRAVERSAL    | ≥ 50 total TOWER points   |

### Field Dimensions

- Field: 16.54 m × 8.07 m (651.2" × 317.7")
- Blue HUB Center: (4.03 m, 4.035 m)
- Red HUB Center: (12.51 m, 4.035 m)

### Teleop Timing (HUB Shift Schedule)

| Period           | Duration | HUB State                   |
| ---------------- | -------- | --------------------------- |
| Transition Shift | 10 s     | Both HUBs Active            |
| Shift 1          | 25 s     | Alternating per auto result |
| Shift 2          | 25 s     | Alternating                 |
| Shift 3          | 25 s     | Alternating                 |
| Shift 4          | 25 s     | Alternating                 |
| End Game         | 30 s     | Both HUBs Active            |

> The alliance that scored **more FUEL in Autonomous** starts Teleop with its HUB **Inactive** in Shift 1. FUEL scored up to 3 seconds after deactivation still counts (grace period, Game Manual §6.5).

---

## Quick Start

**Prerequisites:** JDK 17, WPILib 2026 suite installed

```bash
./gradlew build          # Compile + run all tests
./gradlew test           # Run JUnit 5 tests only
./gradlew simulateJava   # Desktop simulation with GUI
./gradlew deploy         # Deploy to robot (USB or network)
./gradlew clean          # Remove build artifacts
```

**First deployment on a new robot:**

1. Edit **`RobotConfig.java`** — this is the **only file you need to touch** for a new robot. See [Robot Configuration](#robot-configuration) and [`GUIDELINE.md`](GUIDELINE.md) for step-by-step instructions.
2. Physically center the turret at 0° before powering on
3. Physically move the hood to its minimum (most open) position before powering on
4. Run the **System Check** command from SmartDashboard before every match
5. Configure camera names in the PhotonVision UI to match the names in `VisionConstants`

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
4. Shooter.initialize(poseSupplier, fieldSpeedsSupplier)       ← needs SwerveDrive
5. Feeder.initialize(shooterReadySupplier, shooterRPSSupplier) ← needs Shooter
6. Vision.initialize(measurementConsumer, poseSupplier)        ← needs SwerveDrive
7. Superstructure.initialize()                                 ← needs all above
8. LEDs.initialize()                                           ← needs Superstructure, Shooter, Vision
```

### Key Design Patterns

- **Single-file robot configuration** — All per-robot values (CAN IDs, PID gains, hardware dimensions, camera poses, current limits) live in `RobotConfig.java`. Constants files delegate to it. No other file needs changing between robot deployments.
- **Superstructure State Machine** — Central coordinator for Intake, Spindexer, Feeder, and Shooter. All operator actions flow through Superstructure command factories to prevent subsystem conflicts.
- **Singleton Subsystems** — Each subsystem has a static `initialize()` / `getInstance()` pattern with `resetInstance()` for test teardown.
- **Supplier Injection** — Pose and velocity data flows between subsystems via `Supplier<>` lambdas (e.g., `SwerveDrive::getPose` → Shooter auto-aim).
- **Health Monitoring** — `RobotState` singleton tracks battery voltage, CAN bus utilization, loop overruns, and per-subsystem health flags.
- **Pre-allocated Arrays** — All periodic arrays (module states, module positions) are pre-allocated at construction to avoid GC pressure in the robot loop.

---

## Subsystems

### SwerveDrive

4× SDS MK4i L2 swerve modules with Kraken X60 drive and steer motors, CANcoders for absolute steering, Pigeon2 IMU for field-oriented control.

**Key features:**

- Dedicated **250 Hz odometry thread** (`Notifier`) for high-frequency pose updates independent of the 50 Hz main loop
- `SwerveDrivePoseEstimator` for vision-fused odometry
- Second-order kinematics (trapezoidal midpoint prediction) to reduce trajectory lag
- `ChassisSpeeds.discretize()` to reduce rotation-induced drift
- Brownout protection: scales chassis speeds by voltage compensation factor
- Slip detection: compares commanded vs actual wheel speed, flags >30% discrepancy
- Gyro drift detection: compares Pigeon2 angular velocity vs kinematic angular velocity
- PathPlanner `AutoBuilder` configured at startup for on-the-fly pathfinding
- PathPlanner `PathfindingCommand.warmupCommand()` scheduled at init to avoid cold-start delay

---

### SwerveModule

Individual MK4i module control (one instance per module, created by SwerveDrive).

**Key features:**

- **Cosine compensation** — scales drive speed by `cos(angleError)` to reduce scrubbing during large steering corrections
- **Anti-jitter threshold** — holds last steer angle when commanded drive speed < 0.05 m/s
- **Signal health monitoring** — detects stale CAN frames and marks the module unhealthy
- All CAN status signals are cached and refreshed in batch (`BaseStatusSignal.refreshAll`) in the 250 Hz odometry thread

---

### SwerveDriveSim

Desktop simulation support using WPILib `DCMotorSim` and Phoenix 6 simulation state APIs. Active only when `RobotBase.isSimulation()` is true.

---

### Shooter

Three-axis aiming system with autonomous Hub tracking.

| Component | Motor      | CAN ID | Control            |
| --------- | ---------- | ------ | ------------------ |
| Flywheel  | Kraken X60 | 20     | Velocity (RPS)     |
| Hood      | Kraken X60 | 21     | Position (degrees) |
| Turret    | Kraken X60 | 22     | Position (degrees) |

**Flywheel specs:**

- Bottom flywheel: 4" diameter, 2:1 gear ratio from motor
- Top flywheel: 2" diameter (geared 2× faster for equal surface speed)
- Ready window: ±2 RPS
- Idle pre-spin: 15 RPS (keeps flywheel warm between shots)
- Normalization ceiling: 90 RPS

**Hood specs:**

- Travel: 27.5° – 42.5° from horizontal
- Gravity feedforward: `kHoodG = 0.15 V` (constant holdoff)
- Tolerance: ±0.5°
- Must be physically placed at minimum (most open) position before power-on

**Turret specs:**

- Travel: ±175° from center (soft limits; `ContinuousWrap` intentionally disabled)
- Tolerance: ±1°
- Must be physically centered at 0° before power-on
- **Wraparound protection:** when turret reaches ±150°, sends a proportional rotation hint to `SwerveDriveCommand` so the drivetrain auto-rotates to bring the target back into range

**Shooting pipeline:**

1. `updateHubCalculations()` runs every cycle — computes distance, turret angle, and shoot-while-moving compensation
2. Shoot-while-moving: looks up time-of-flight from `timeOfFlightTable`, predicts robot future position, re-solves aim
3. Lookup tables generated by `ShooterPhysics.computeShootingTable()` at robot init (not hardcoded)
4. `isReadyToShoot()` gates the feeder: flywheel at speed + hood at target + turret at target + in range + flywheel recovered

**Alliance zone dump mode:**

- Fixed parameters (no auto-aim) configured in `RobotConfig.java`
- Activated via operator right bumper

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

| State                   | Intake Arm  | Roller  | Spindexer        | Feeder          | Notes                                          |
| ----------------------- | ----------- | ------- | ---------------- | --------------- | ---------------------------------------------- |
| IDLE                    | No change   | Off     | Off              | Off             | Shooter still auto-aims in its periodic()      |
| INTAKING                | No change   | Forward | Off              | Off             | Roller only runs if arm is deployed            |
| PRE_FEED_REVERSE        | No change   | Off     | Reverse slow     | Reverse slow    | Anti-jam clearance pulse before feeding        |
| SHOOTING                | No change   | Off     | Forward (scaled) | Forward (gated) | Feeder gated by `isReadyToShoot()` + HUB state |
| SHOOTING_WHILE_INTAKING | No change   | Forward | Forward (scaled) | Forward (gated) | Combined intake + shoot                        |
| SHOOTING_ALLIANCE       | No change   | Off     | Forward (scaled) | Forward (gated) | Fixed shooter params                           |
| OUTTAKING               | No change   | Reverse | Reverse          | Off             | Arm must be deployed                           |
| UNJAMMING               | No change   | Reverse | Reverse          | Reverse         | Reverses everything                            |
| DISABLED                | Stow + stop | Off     | Off              | Off             | Emergency stop                                 |

**Endgame auto-dump:** In the last 10 seconds of Teleop, Superstructure automatically enables shooter tracking and transitions to SHOOTING. Operator can cancel via START (emergency stop) or BACK (idle) — once cancelled, the dump will not re-activate for the rest of the match.

**Ball counting:** The feeder beam break is the sole ball counter. Superstructure tracks rising edges from `Feeder.didBallPassThisCycle()`.

---

### Intake

Slapdown-style intake with a powered arm and roller.

| Component    | Motor      | CAN ID | Control                                |
| ------------ | ---------- | ------ | -------------------------------------- |
| Left deploy  | Kraken X60 | 24     | Motion Magic (position)                |
| Right deploy | Kraken X60 | 25     | Voltage mirror (asymmetric gravity FF) |
| Roller       | Kraken X60 | 26     | Open-loop duty cycle                   |

**Asymmetric gravity compensation:**
Standard `Arm_Cosine` doesn't match the nonlinear slapdown linkage torque profile. Instead:

- Left motor runs Motion Magic + gravity feedforward from `InterpolatingDoubleTreeMap`
- Right motor mirrors left motor output, replacing the gravity FF component with its own higher-voltage table (more weight on right side due to off-center roller/linkage mass)
- Both gravity tables are defined in `RobotConfig.java` (`kDeployLeftGravityVoltages` / `kDeployRightGravityVoltages`) and must be tuned on the real robot

**Positions:**

- Stowed: 0.0 mechanism rotations (inside bumpers)
- Deployed: ~−0.00323 mechanism rotations (ground contact, derived from `kIntakeExtendedMotorRotations`)
- Hover: midpoint (partial deploy)

**Stall detection:** >25 A for >25 consecutive cycles (~0.5 s) triggers a warning

---

### Feeder

Ball transport from the spindexer hopper to the shooter flywheels.

| Component                 | CAN ID                                    | Type                 |
| ------------------------- | ----------------------------------------- | -------------------- |
| Feeder motor (Kraken X60) | 23                                        | Open-loop duty cycle |
| Beam break sensor         | DIO (see `RobotConfig.kBeamBreakDIOPort`) | `DigitalInput`       |

**Speed control:** `feeder_duty = kFeederSpeedRatio × (shooterTargetRPS / kMaxFlywheelRPS)`. This creates a chain (Shooter RPS → Feeder → Spindexer) that automatically enforces the speed hierarchy.

**Feed gating:** Feeder only runs when `feedRequested == true` AND `Shooter.isReadyToShoot() == true`.

**Auto-unjam:** Stall detected (>25 A + <0.5 RPS for 10 cycles) → auto-reverses, then resumes.

**Beam break logic:** Located between feeder and flywheels. Active-low (`!beamBreak.get() == true`). Rising edge = `ballPassedThisCycle`, read once per cycle by Superstructure for ball counting.

---

### Spindexer

Rotating hopper that indexes balls from the intake toward the feeder.

| Component                    | CAN ID |
| ---------------------------- | ------ |
| Spindexer motor (Kraken X60) | 27     |

**Speed hierarchy (critical — prevents jams):**

```
Flywheel surface speed  >>  Feeder  >  Spindexer  >  Intake roller
```

The `kFeederSpeedRatio` / `kSpindexerToFeederRatio` chain enforces this automatically.

**Auto-unjam:** Stall detected (>25 A + <0.3 RPS for 15 cycles) → reverses, then resumes at last commanded speed.

---

### Vision

Multi-camera AprilTag processing using PhotonVision (PhotonLib).

**Camera layout (6 cameras total):**

| Camera Name       | Purpose                                             |
| ----------------- | --------------------------------------------------- |
| `cam_front_left`  | AprilTag pose estimation — front-left corner        |
| `cam_front_right` | AprilTag pose estimation — front-right corner       |
| `cam_back_left`   | AprilTag pose estimation — back-left corner         |
| `cam_back_right`  | AprilTag pose estimation — back-right corner        |
| `cam_intake`      | Ball / object detection (ML pipeline, NOT AprilTag) |
| `cam_side`        | Situational awareness                               |

Each camera has an independently configured 6-DOF pose (X, Y, Z, roll, pitch, yaw) in `RobotConfig.java`. No two cameras share the same height or angle assumptions — measure each one individually.

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
     - Multi-tag: base devs from RobotConfig → scaled by (1 + avgDist²/30)
     - Single-tag: base devs from RobotConfig → rejected if avgDist > 5.5m
  └── SwerveDrive.addVisionMeasurement()
```

**HUB AprilTag IDs:**

- Blue HUB: Tags 2, 3, 4, 5, 8, 9, 10, 11
- Red HUB: Tags 18, 19, 20, 21, 24, 25, 26, 27

**Ball detection (intake camera):** PhotonVision ML pipeline detects FUEL balls. Outputs `isBallDetected()`, `getBallYaw()`, `getBallPitch()`, `getBallArea()` used by `VisionIntakeCommand`.

**Distance validation:** `Superstructure` calls `Vision.validateDistance(odometryDistance)` while shooting. Flags discrepancies > 1.0 m between odometry and direct camera distance to the HUB.

---

### LEDs

AddressableLED strip (PWM port and LED count set in `RobotConfig.java`). Priority-based state display updated every robot cycle.

See [LED Feedback Reference](#led-feedback-reference) for the full priority table.

---

## Commands & Controls

### Driver Controller (Port 0) — Driving only

| Input                    | Action                                        |
| ------------------------ | --------------------------------------------- |
| Left Stick X/Y           | Translation (field-relative)                  |
| Right Stick X            | Rotation                                      |
| **Right Trigger** (>0.3) | **Slow mode** (35% speed, 40% rotation)       |
| **X Button**             | **X-lock wheels** (defensive stance, hold)    |
| **Y Button**             | **Vision ball chase** + intake rollers (hold) |
| **START**                | Zero gyro (reset field-relative forward)      |
| D-Pad Up                 | Snap heading to 0° (away from DS)             |
| D-Pad Right              | Snap heading to −90° (face right)             |
| D-Pad Down               | Snap heading to 180° (face DS)                |
| D-Pad Left               | Snap heading to 90° (face left)               |

### Operator Controller (Port 1) — All mechanisms

| Input                    | Action                                                            |
| ------------------------ | ----------------------------------------------------------------- |
| **Right Trigger** (>0.3) | **Shoot** — enables Hub tracking + feed (hold)                    |
| **Left Trigger** (>0.3)  | **Intake rollers** — hold to collect (arm must be deployed first) |
| **A Button**             | **Intake arm toggle** — deploy/stow (press)                       |
| **Right Bumper**         | **Alliance zone dump** — lob shot into alliance zone (hold)       |
| **Left Bumper**          | **Outtake** — deploy + reverse rollers (hold)                     |
| **B Button**             | **Unjam** — reverse everything (hold)                             |
| **START**                | Emergency stop all mechanisms                                     |
| **BACK**                 | Re-enable after emergency stop                                    |
| D-Pad Up                 | Hood angle trim +2° (additive to auto-aim, clamped ±4°)           |
| D-Pad Down               | Hood angle trim −2°                                               |

### Teleop Drive Features

- **Heading Lock** — `ProfiledPIDController` captures heading when rotation stick is released, holds it until driver rotates again
- **Slow Mode** — Right trigger reduces translation and rotation to configurable fractions (`RobotConfig.kSlowModeSpeedMultiplier`, `kSlowModeRotMultiplier`)
- **Snap-to-Angle** — D-pad drives heading PID to exact cardinal angles
- **Slew Rate Limiting** — Translation and rotation acceleration capped (`RobotConfig.kTranslationSlewRate`, `kRotationSlewRate`)
- **Squared Inputs** — Joystick values squared (preserving sign) for finer low-speed control
- **Intake Speed Cap** — While intaking, chassis translation capped to 85% of roller surface speed to prevent balls being pushed away
- **Turret Wraparound Assist** — When turret nears ±150°, a proportional rotation hint overrides/augments driver input to auto-rotate the robot back into range

---

## LED Feedback Reference

Priority order (highest to lowest):

| Priority | Condition                     | Pattern              | Color          |
| -------- | ----------------------------- | -------------------- | -------------- |
| 1        | Any subsystem unhealthy       | Fast flash (5 Hz)    | Red            |
| 2        | Low battery (<7.0 V)          | Pulse (2 Hz)         | Orange-Red     |
| 2.5      | <2 AprilTag cameras connected | Alternating          | Cyan / Red     |
| 2.6      | Turret at hard limit          | Rapid strobe (12 Hz) | Orange         |
| 2.7      | Brownout predicted            | Strobe (8 Hz)        | Orange         |
| 2.8      | Turret near limit (>150°)     | Pulse (4 Hz)         | Orange         |
| 3        | Disabled                      | Breathe (1.5 Hz)     | Alliance color |
| 4        | Shooting + ready to shoot     | Strobe (10 Hz)       | Green          |
| 4.1      | Alliance zone dump + ready    | Strobe (10 Hz)       | Purple         |
| 4.2      | Shooting + waiting for lock   | Pulse (3 Hz)         | Yellow         |
| 5        | Intaking + ball detected      | Flash (6 Hz)         | Cyan           |
| 6        | Intaking                      | Chase                | Blue           |
| 7        | Outtaking                     | Chase                | Orange         |
| 8        | Unjamming                     | Flash (3 Hz)         | Red            |
| 9        | Idle + tracking + ready       | Solid                | Green          |
| 10       | Idle + tracking + not locked  | Pulse (3 Hz)         | Magenta        |
| 11       | Idle + in range, not tracking | Pulse (2 Hz)         | Green          |
| 12       | Idle                          | Solid                | Alliance color |

---

## Autonomous

Two autonomous systems coexist, selectable from the SmartDashboard `"Auto Chooser"`:

### PathPlanner (Primary — `[PP]` prefix)

Dynamic pathfinding using PathPlanner's AD\* algorithm with `navgrid.json` for obstacle awareness. Paths computed at runtime via `AutoBuilder.pathfindToPose()`.

| Routine                         | Description                                               | Expected Score |
| ------------------------------- | --------------------------------------------------------- | -------------- |
| `[PP] Preload & Park`           | Score preload, park in neutral zone                       | ~8 pts         |
| `[PP] Preload & Collect Center` | Score preload, collect center, score again                | ~12–16 pts     |
| `[PP] Preload & Collect Side`   | Score preload, collect near trench, score again           | ~12–16 pts     |
| `[PP] Preload & Collect Depot`  | Score preload, collect depot (24 pre-staged), score again | ~12–16 pts     |
| `[PP] Double Trench Score`      | Two trench crossings with scoring                         | ~14–20 pts     |
| `[PP] Aggressive Sweep`         | Near-trench → center → far-trench                         | ~16–24 pts     |
| `[PP] Defensive Preload`        | Score preload, park near hub to deny access               | ~8 pts         |

- Robot config read from PathPlanner GUI settings file (`RobotConfig.fromGUISettings()`)
- Coordinates always in Blue Alliance frame; `AutoBuilder` flips for Red automatically
- Flywheel pre-spin overlaps with drive time to save 0.5–1.0 s per scoring cycle

### ChoreoLib (Fallback — `[Choreo]` prefix)

Pre-planned trajectories from the Choreo GUI, alliance-mirrored automatically.

| Routine                       | Required Trajectory Files |
| ----------------------------- | ------------------------- |
| `[Choreo] Preload & Park`     | `preload.1`, `preload.2`  |
| `[Choreo] Center 2-Cycle`     | `center2.1–3`             |
| `[Choreo] Left 2-Cycle`       | `left2.1–3`               |
| `[Choreo] Right 2-Cycle`      | `right2.1–3`              |
| `[Choreo] Center 3-Cycle`     | `center3.1–5`             |
| `[Choreo] Shoot While Moving` | `swm.1`, `swm.2`          |
| `[Choreo] Depot Cycle`        | `depot.1–3`               |

Trajectory `.traj` files go in `src/main/deploy/choreo/`. Choreo is the fallback system; PathPlanner is primary.

### Pre-Match System Check

Run `"System Check"` from SmartDashboard before every match. Results appear under `SystemCheck/`:

1. **Swerve** — Brief drive, verifies all modules healthy
2. **Intake Deploy** — Deploys arm, verifies position reached within 0.5 s
3. **Spindexer** — Runs briefly, records empty-hopper current baseline
4. **Feeder** — Runs briefly, records baseline, calibrates `BallPresenceEstimator`
5. **Shooter** — Spins flywheel briefly, checks health flags
6. **Vision** — Counts connected AprilTag cameras (minimum 1 to pass)

---

## Vision

### PhotonVision Camera Setup

All 6 cameras must be configured in the PhotonVision web UI before use:

1. Open PhotonVision UI (robot IP:5800)
2. Name each camera **exactly** as listed in `VisionConstants`:
   - `cam_front_left`, `cam_front_right`, `cam_back_left`, `cam_back_right`
   - `cam_intake`, `cam_side`
3. Set the 4 corner cameras to the **AprilTag pipeline**
4. Set `cam_intake` to an **ML object detection pipeline** trained on FUEL balls
5. Set `cam_side` to your preferred pipeline

### Camera Transform Convention

Each camera's robot-to-camera transform is built from its 6 individual fields in `RobotConfig.java`:

```java
// Example: front-left AprilTag camera
Transform3d kRobotToFrontLeftCam = new Transform3d(
    new Translation3d(kFrontLeftCamX, kFrontLeftCamY, kFrontLeftCamZ),
    new Rotation3d(kFrontLeftCamRoll, kFrontLeftCamPitch, kFrontLeftCamYaw)
);
```

Axes: X = forward, Y = left, Z = up. Pitch is **negative for upward tilt** (WPILib convention). All 6 cameras are configured independently — measure each one on the actual robot. See `GUIDELINE.md` § Camera Hardware for the measurement procedure.

### Pose Standard Deviations

Configured in `RobotConfig.java` as raw `double[]` arrays; `VisionConstants` constructs the `Matrix` objects:

| Condition                                  | Default std devs (x, y, heading)           |
| ------------------------------------------ | ------------------------------------------ |
| Multi-tag                                  | (0.5, 0.5, 1.0) → then scaled by distance² |
| Single tag                                 | (4.0, 4.0, 8.0) → rejected beyond 5.5 m    |
| Aggressive multi-tag (post-collision snap) | (0.1, 0.1, 0.2)                            |

---

## Logging & Telemetry

### AdvantageKit

- **Real robot:** Logs to USB stick (`WPILOGWriter`) + NetworkTables (`NT4Publisher`)
- **Simulation:** NetworkTables only
- **Replay:** Full deterministic replay from `.wpilog` files in AdvantageScope

All subsystems use `Logger.recordOutput("Key", value)` for structured telemetry.

### Key Log Paths

| Path                           | Description                                   |
| ------------------------------ | --------------------------------------------- |
| `Swerve/Pose`                  | Robot field position (Pose2d)                 |
| `Swerve/ModuleStates`          | All 4 module states                           |
| `Swerve/GyroDriftWarning`      | Gyro vs kinematics omega disagrees >0.5 rad/s |
| `Shooter/DistanceToHub`        | Odometry-based distance to Hub                |
| `Shooter/CompensatedDistance`  | Shoot-while-moving compensated distance       |
| `Shooter/ReadyToShoot`         | All shooter systems locked                    |
| `Shooter/TurretWraparoundHint` | Rotation hint for drivetrain                  |
| `Shooter/TurretPose3d`         | 3D turret visualization (AdvantageScope)      |
| `Shooter/HoodPose3d`           | 3D hood visualization (AdvantageScope)        |
| `Super/State`                  | Current Superstructure state name             |
| `Super/BallsShot`              | Running ball count (beam break)               |
| `Super/HubActive`              | HUB currently active                          |
| `Super/HubNextShift`           | Seconds until next shift boundary             |
| `HUB/Status`                   | ACTIVE / INACTIVE / UNKNOWN                   |
| `HUB/ShouldShoot`              | Strategic shoot decision                      |
| `Vision/ConnectedATCameras`    | Number of connected AprilTag cameras          |
| `Vision/BallDetected`          | Ball visible in intake camera                 |
| `Vision/DistanceDiscrepancy`   | Odometry vs vision distance delta             |
| `BallEst/HopperEmpty`          | Current-based hopper empty flag               |
| `BallEst/HopperLoadLevel`      | 0.0–1.0 fullness estimate                     |
| `RobotState/BatteryVoltage`    | Battery voltage                               |
| `RobotState/CANUtilization`    | CAN bus % utilization                         |
| `RobotState/LoopOverruns`      | Loop overrun count (>25 ms)                   |
| `Intake/LeftGravityFF`         | Left arm gravity feedforward (V)              |
| `Intake/RightGravityFF`        | Right arm gravity feedforward (V)             |

---

## Robot Configuration

### The One-File Policy

**`RobotConfig.java` is the single file to edit when deploying to a new physical robot.**

All subsystems, commands, and tests import constants from the `constants/` package. Every field in that package that depends on physical hardware or robot-specific tuning delegates to `RobotConfig`. This means:

- You never touch a subsystem file to change a CAN ID or PID gain
- You never touch a constants file for a new robot deployment
- `RobotConfig.java` is the canonical list of everything that differs between robots

### What lives in RobotConfig

| Category                            | Examples                                                                                                             |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Controller ports & deadband         | `kDriverControllerPort`, `kDeadband`                                                                                 |
| LED hardware                        | `kLedPort`, `kLedCount`                                                                                              |
| All 13 swerve CAN IDs               | `kFrontLeftDriveMotorId` … `kPigeonId`                                                                               |
| Swerve module hardware              | `kDriveGearRatio`, `kWheelDiameterMeters`                                                                            |
| Swerve motor current limits         | `kDriveCurrentLimit`, `kDriveStatorCurrentLimit`, `kSteerCurrentLimit`                                               |
| All swerve PID / FF / driver tuning | `kDriveP/V/S`, `kSteerP`, `kAutoTranslationP`, `kHeadingLockP`, `kSlowModeSpeedMultiplier`, …                        |
| Shooter CAN IDs                     | `kFlywheelMotorId`, `kHoodMotorId`, `kTurretMotorId`                                                                 |
| Shooter hardware dimensions         | flywheel diameters, gear ratios, turret/hood heights                                                                 |
| Shooter geometry                    | `kShooterPositionOffset`, `kShooterExitHeightMeters`                                                                 |
| Hood & turret calibration           | hard-stop encoder readings, turret gear ratio, mount offset                                                          |
| Shooter PID / FF / tuning           | flywheel / hood / turret PID, efficiency factor, bias factors                                                        |
| Alliance zone dump params           | `kAllianceZoneFlywheelRPS`, hood angle, turret angle, feeder speed                                                   |
| Pre-feed reverse params             | duration, feeder speed, spindexer speed                                                                              |
| Feeder & spindexer CAN + tuning     | motor IDs, beam break DIO, speed ratios, current limits                                                              |
| Intake CAN IDs                      | `kLeftDeployMotorId`, `kRightDeployMotorId`, `kRollerMotorId`                                                        |
| Intake hardware / calibration       | gear ratios, deploy travel, roller dimensions, PID, Motion Magic, gravity FF voltages, current limits, roller speeds |
| Per-camera 6-DOF poses              | X, Y, Z, roll, pitch, yaw for each of 6 cameras                                                                      |
| Vision std devs & ball-chase PID    | `kMultiTagStdDevs`, `kBallChaseYawP`, …                                                                              |
| Ball detection thresholds           | baseline currents, loaded/empty/full thresholds                                                                      |
| Swerve geometry                     | `kTrackWidthMeters`, `kWheelBaseMeters`                                                                              |
| CANcoder offsets                    | one per module                                                                                                       |

### Full configuration instructions

See [`GUIDELINE.md`](GUIDELINE.md) for:

- Step-by-step configuration order (19 steps, hardware vs software)
- Exact measurement procedures for every physical value
- PID tuning procedures for every subsystem
- A quick reference table of all ~110 fields with defaults and units
- Post-configuration verification checklist

---

## SysId Characterization

`SysIdCommands.java` provides WPILib SysId routines for characterizing all motors.

| Routine                             | Motor               | Use For                         |
| ----------------------------------- | ------------------- | ------------------------------- |
| `createDriveRoutine(drive)`         | Swerve drive motors | `kDriveS`, `kDriveV`, `kDriveA` |
| `createFlywheelRoutine(shooter)`    | Flywheel            | `kFlywheelS`, `kFlywheelV`      |
| `createHoodRoutine(shooter)`        | Hood                | `kHoodG`, `kHoodP`              |
| `createTurretRoutine(shooter)`      | Turret              | `kTurretP`                      |
| `createIntakeDeployRoutine(intake)` | Left deploy motor   | `kDeployS`, `kDeployP`          |

**Procedure:**

1. Add SysId command bindings to a test-mode controller in `RobotContainer`
2. Enable Test mode in DriverStation
3. Run quasistatic forward → quasistatic reverse → dynamic forward → dynamic reverse
4. Collect the `.wpilog` from USB
5. Open in WPILib SysId tool, export kS/kV/kA values
6. Enter the measured values into `RobotConfig.java`

---

## Pre-Match Checklist

- [ ] Battery: full charge, secure terminals
- [ ] Turret: physically centered at 0° before powering on
- [ ] Hood: physically at minimum position (most open) before powering on
- [ ] Power on robot
- [ ] **System Check:** run from SmartDashboard — all `SystemCheck/*` booleans must be `true`
- [ ] **Gyro zero:** press START on driver controller after placing robot at starting position
- [ ] **Auto chooser:** correct routine selected in `"Auto Chooser"`
- [ ] **Auto override:** if FMS not attached, verify `"Auto/Use Override"` is `false`
- [ ] **Vision:** `SystemCheck/Vision Cameras` ≥ 2 (prefer all 4)
- [ ] **LEDs:** solid alliance color (idle) before enable
- [ ] **Hood trim:** `Shooter/HoodTrimDeg` is 0.0 (reset between matches)

---

## Testing

13 JUnit 5 test files covering all subsystems and utilities. No hardware required.

```bash
./gradlew test
```

| Test File                   | Coverage                                                                                                             |
| --------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `IntakeTest`                | Gravity table interpolation/monotonicity, deploy positions, tolerance, stall detection, roller speeds, PID constants |
| `ShooterTest`               | Flywheel lookup tables, hood angle interpolation, turret wrapping, shoot-while-moving compensation                   |
| `VisionTest`                | Pose Z-axis bounds, field boundaries, pose-jump rejection, timestamp filtering, std dev scaling, ambiguity           |
| `SwerveDriveTest`           | Slip detection, gyro drift, second-order prediction, voltage compensation, X-lock pattern                            |
| `SwerveModuleTest`          | Cosine compensation, anti-jitter threshold, signal health, drive position conversions                                |
| `FeederTest`                | Feed speed ratio, beam break edge detection, stall detection                                                         |
| `SpindexerTest`             | Running state threshold, stall detection, speed hierarchy, unjam timing                                              |
| `LEDsTest`                  | Flash, pulse, breathe, strobe, chase animation math                                                                  |
| `SuperstructureTest`        | State machine transitions                                                                                            |
| `ShooterTableTest`          | Shooter physics lookup table generation                                                                              |
| `BallPresenceEstimatorTest` | Current baseline calibration, ball detection thresholds                                                              |
| `HubStateTrackerTest`       | Hub shift schedule, FMS data parsing                                                                                 |
| `VisionTest`                | All pose estimation filtering stages                                                                                 |

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

**Simulation:** `SwerveDriveSim` provides realistic swerve module physics using `DCMotorSim` and Phoenix 6 simulation APIs. PathPlanner pathfinding runs in simulation. Vision cameras return no results in simulation.

**Build configuration (`build.gradle`):**

- Java 17 source/target compatibility
- `includeDesktopSupport = true` — enables simulation
- GradleRIO `2026.2.1`
- JUnit 5.10.1 for tests

---

## Hardware Reference

### Drivetrain

| Specification         | Value                   |
| --------------------- | ----------------------- |
| Modules               | 4× SDS MK4i L2          |
| Drive / Steer Motors  | Kraken X60 (TalonFX)    |
| Encoders              | CANcoder (absolute)     |
| IMU                   | Pigeon2                 |
| Drive Gear Ratio      | 6.75:1                  |
| Steer Gear Ratio      | 21.43:1 (150/7)         |
| Wheel Diameter        | 4 in (0.1016 m)         |
| Track Width           | 22.75 in (0.578 m)      |
| Wheelbase             | 22.75 in (0.578 m)      |
| Theoretical Max Speed | ~4.75 m/s               |
| Effective Max Speed   | ~4.5 m/s (95%)          |
| CAN Bus               | CANivore (`"CANivore"`) |
| Odometry Rate         | 250 Hz                  |

### CAN Bus IDs

| Device               | ID  |
| -------------------- | --- |
| Front Left Drive     | 1   |
| Front Left Steer     | 2   |
| Front Left CANcoder  | 3   |
| Front Right Drive    | 4   |
| Front Right Steer    | 5   |
| Front Right CANcoder | 6   |
| Back Left Drive      | 7   |
| Back Left Steer      | 8   |
| Back Left CANcoder   | 9   |
| Back Right Drive     | 10  |
| Back Right Steer     | 11  |
| Back Right CANcoder  | 12  |
| Pigeon2 IMU          | 13  |
| Flywheel Motor       | 20  |
| Hood Motor           | 21  |
| Turret Motor         | 22  |
| Feeder Motor         | 23  |
| Left Deploy Motor    | 24  |
| Right Deploy Motor   | 25  |
| Roller Motor         | 26  |
| Spindexer Motor      | 27  |

> All CAN IDs are configured in `RobotConfig.java` — change them there if wiring differs.

### I/O Ports

| Port                                      | Device                | Config             |
| ----------------------------------------- | --------------------- | ------------------ |
| DIO (see `RobotConfig.kBeamBreakDIOPort`) | Feeder beam break     | `RobotConfig.java` |
| PWM (see `RobotConfig.kLedPort`)          | Addressable LED strip | `RobotConfig.java` |

### Default Current Limits

All configurable in `RobotConfig.java`:

| Motor               | Supply Limit | Stator Limit |
| ------------------- | ------------ | ------------ |
| Drive (per module)  | 60 A         | 80 A         |
| Steer (per module)  | 30 A         | —            |
| Feeder              | 30 A         | —            |
| Left / Right Deploy | 30 A each    | —            |
| Roller              | 40 A         | —            |
| Spindexer           | 30 A         | —            |

---

## Project Structure

```
robot2026/
├── src/
│   ├── main/
│   │   ├── java/frc/robot/
│   │   │   ├── Main.java                       # Entry point
│   │   │   ├── Robot.java                      # Lifecycle (LoggedRobot)
│   │   │   ├── RobotContainer.java             # Assembly, bindings, auto chooser
│   │   │   ├── RobotConfig.java                # ALL per-robot configurables (edit this for new robots)
│   │   │   ├── ShooterPhysics.java             # Projectile motion + table generation
│   │   │   ├── HubStateTracker.java            # Game state (Active/Inactive shift schedule)
│   │   │   ├── RobotState.java                 # Centralized health monitoring
│   │   │   ├── BallPresenceEstimator.java      # Sensorless ball detection (current-based)
│   │   │   ├── constants/
│   │   │   │   ├── SwerveConstants.java        # Swerve — delegates to RobotConfig
│   │   │   │   ├── ShooterConstants.java       # Shooter — delegates to RobotConfig
│   │   │   │   ├── IntakeConstants.java        # Intake — delegates to RobotConfig
│   │   │   │   ├── FeederConstants.java        # Feeder — delegates to RobotConfig
│   │   │   │   ├── SpindexerConstants.java     # Spindexer — delegates to RobotConfig
│   │   │   │   ├── VisionConstants.java        # Vision — delegates to RobotConfig
│   │   │   │   ├── LEDConstants.java           # LEDs — delegates to RobotConfig
│   │   │   │   ├── BallDetectionConstants.java # Ball detection — delegates to RobotConfig
│   │   │   │   ├── OperatorConstants.java      # Controllers — delegates to RobotConfig
│   │   │   │   ├── HubConstants.java           # Hub field geometry (game constants)
│   │   │   │   └── AutoConstants.java          # Auto motion profile limits (game constants)
│   │   │   ├── subsystems/
│   │   │   │   ├── SwerveDrive.java
│   │   │   │   ├── SwerveModule.java
│   │   │   │   ├── SwerveDriveSim.java
│   │   │   │   ├── Shooter.java
│   │   │   │   ├── Superstructure.java
│   │   │   │   ├── Intake.java
│   │   │   │   ├── Feeder.java
│   │   │   │   ├── Spindexer.java
│   │   │   │   ├── Vision.java
│   │   │   │   └── LEDs.java
│   │   │   └── commands/
│   │   │       ├── SwerveDriveCommand.java
│   │   │       ├── Autos.java
│   │   │       ├── OnTheFlyAutos.java
│   │   │       ├── VisionIntakeCommand.java
│   │   │       └── SysIdCommands.java
│   │   └── deploy/
│   │       └── pathplanner/
│   │           └── navgrid.json                # Field navigation grid (AD* obstacles)
│   └── test/
│       └── java/frc/robot/
│           ├── (13 test files — see Testing section)
├── vendordeps/                                 # Vendor dependency JSON files
├── build.gradle                                # GradleRIO 2026.2.1 build config
├── GUIDELINE.md                                # Step-by-step robot configuration guide
└── WPILib-License.md
```

---

## Vendor Dependencies

| Library                                                              | Version   | Purpose                                      |
| -------------------------------------------------------------------- | --------- | -------------------------------------------- |
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | 26.0.0    | Structured logging with deterministic replay |
| [Phoenix 6](https://pro.docs.ctr-electronics.com/)                   | 26.1.1    | CTRE hardware (TalonFX, CANcoder, Pigeon2)   |
| [PhotonLib](https://docs.photonvision.org/)                          | v2026.2.2 | AprilTag vision + object detection           |
| [REVLib](https://docs.revrobotics.com/)                              | 2026.0.1  | REV Robotics hardware support                |
| [PathplannerLib](https://pathplanner.dev/)                           | 2026.1.2  | Dynamic pathfinding (AD\* algorithm)         |
| [ChoreoLib](https://choreo.autos/)                                   | 2026.0.1  | Pre-planned trajectory following             |
| WPILibNewCommands                                                    | 1.0.0     | Command-based framework                      |

Vendor JSON files are in `vendordeps/`. Update via WPILib VS Code extension or Phoenix Tuner X.

---

## License

Open source under the WPILib BSD license. See `WPILib-License.md`.

---

_FRC Team 7840 — 2026 Season_
