# FRC Team 7840 — Robot Code 2026 "REBUILT"

Competition robot code for the 2026 FIRST Robotics Competition season. Built in Java on WPILib's command-based framework with AdvantageKit structured logging, targeting the NI RoboRIO 2.0.

> **25,000+ lines** of Java across 23 source files, 13 test files, and 7 vendor libraries.

---

## Table of Contents

- [Game Overview](#game-overview)
- [Architecture](#architecture)
- [Subsystems](#subsystems)
- [Commands & Controls](#commands--controls)
- [Autonomous](#autonomous)
- [Vision](#vision)
- [Logging & Telemetry](#logging--telemetry)
- [Constants & Tuning](#constants--tuning)
- [Testing](#testing)
- [Build & Deploy](#build--deploy)
- [Hardware Configuration](#hardware-configuration)
- [Project Structure](#project-structure)
- [Vendor Dependencies](#vendor-dependencies)

---

## Game Overview

**REBUILT (2026)** features two alliances competing to score FUEL balls into a central HUB structure. The HUB alternates between Active and Inactive states on a timed shift schedule. Robots collect balls from the field, depot stations, and the neutral zone, then score them via a flywheel shooter with an adjustable hood aimed by a turret. An "Alliance Zone Dump" mechanic allows lobbing balls from the neutral zone into the alliance scoring area.

**Field Dimensions:** 16.54 m x 8.02 m
- Blue HUB: (4.03 m, 4.035 m)
- Red HUB: (12.51 m, 4.035 m)

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
                    ├── Subsystem initialization (singleton pattern)
                    ├── Controller bindings (driver + operator)
                    ├── Default commands
                    └── Autonomous chooser (SmartDashboard)
```

### Key Design Patterns

- **Superstructure State Machine** — Central coordinator for Intake, Spindexer, Feeder, and Shooter. All operator actions flow through Superstructure command factories to prevent subsystem conflicts.
- **Singleton Subsystems** — Each subsystem has a static `initialize()` / `getInstance()` pattern with dependency injection via `Supplier<>` lambdas.
- **Supplier Injection** — Pose and velocity data flows between subsystems via suppliers (e.g., SwerveDrive provides pose to Shooter for shoot-while-moving).
- **Health Monitoring** — `RobotState` singleton tracks battery voltage, CAN bus utilization, loop overruns, and per-subsystem health flags.

---

## Subsystems

### SwerveDrive
4x SDS MK4i L2 swerve modules with Kraken X60 motors and CANcoders. Pigeon2 IMU for field-oriented control. Runs a dedicated 250Hz odometry thread for high-frequency pose updates. Supports PathPlanner and ChoreoLib trajectory following. Features slip detection, voltage compensation, and X-lock defensive stance.

### Shooter
Three-axis aiming system: **Flywheel** (velocity control), **Hood** (angle control), and **Turret** (heading control). Uses `InterpolatingDoubleTreeMap` lookup tables generated from `ShooterPhysics.java` projectile motion calculations. Supports shoot-while-moving compensation that leads the target based on chassis velocity. Alliance-aware HUB targeting with automatic red/blue switching.

### Superstructure
Finite state machine coordinating the intake-to-shoot pipeline:

```
IDLE ──► INTAKING ──► PRE_FEED_REVERSE ──► SHOOTING
  │                                            │
  ├──► SHOOTING_ALLIANCE                       │
  ├──► OUTTAKING                               │
  ├──► UNJAMMING                               │
  ├──► DISABLED                                │
  └────────────────────────────────────────────┘
```

### Intake
Slapdown-style intake with powered arm + passive linkage that folds when stowed and extends when deployed. Dual deploy motors with **asymmetric gravity compensation** — the right motor carries more weight than the left, handled by a custom software follower algorithm:

- Left motor runs PID + Motion Magic position control
- Right motor mirrors the left's PID output voltage with its own (higher) gravity feedforward from a position-based lookup table
- `InterpolatingDoubleTreeMap` maps arm position to gravity feedforward voltage for each side independently, accounting for the non-linear torque profile of the slapdown linkage

Roller motor for ball collection with configurable intake/outtake speeds.

### Feeder
Ball transport mechanism between the spindexer and shooter. Uses beam break sensors for ball detection and current monitoring for stall protection. Coordinates with Shooter readiness before feeding.

### Spindexer
Ball indexing hopper that rotates to position balls for feeding. Implements jam detection via motor current monitoring with configurable thresholds. Supports intake, outtake, and unjam speeds.

### Vision
Multi-camera AprilTag processing using PhotonVision (PhotonLib). 4 cameras with known robot-to-camera transforms. Implements multi-layer pose validation:

1. **Z-axis bounds** — Rejects poses below floor or above ceiling
2. **Field boundary check** — Rejects poses outside field + margin
3. **Pose jump rejection** — Rejects sudden large jumps from current odometry
4. **Timestamp filtering** — Rejects stale or future timestamps
5. **Ambiguity filtering** — Rejects high-ambiguity single-tag solutions
6. **Distance scaling** — Standard deviations scale quadratically with tag distance

Three confidence tiers: single-tag, multi-tag, and aggressive multi-tag standard deviations.

### LEDs
AddressableLED driver with priority-based state display. Supports flash, pulse, breathe, strobe, and chase animations. Priority order: fault > low battery > ready to shoot > intaking > shooting > idle.

### SwerveModule
Individual MK4i module control. Implements cosine compensation (scales drive speed by angle error), anti-jitter threshold (holds steer angle at low speeds), and signal health monitoring (detects stale CAN frames).

### SwerveDriveSim
Desktop simulation support using WPILib `DCMotorSim` and Phoenix 6 simulation state APIs for realistic swerve behavior without hardware.

---

## Commands & Controls

### Driver Controller (Port 0)

| Input | Action |
|---|---|
| Left Stick | Translation (field-relative X/Y) |
| Right Stick | Rotation |
| Right Trigger | Slow mode (hold for fine positioning) |
| X Button | X-lock wheels (defensive stance) |
| Y Button | Vision ball chase (auto-drives to nearest ball) |
| START | Zero gyro (reset field-relative forward) |
| D-Pad | Snap-to-angle (0/90/180/270 degrees) |

### Operator Controller (Port 1)

| Input | Action |
|---|---|
| Right Trigger | Shoot (Hub tracking + feed) |
| Left Trigger | Intake rollers (hold to spin) |
| A Button | Intake arm toggle (deploy/stow) |
| Right Bumper | Alliance zone dump (fixed lob shot) |
| Left Bumper | Outtake (eject balls) |
| B Button | Unjam (reverse everything) |
| START | Emergency stop all mechanisms |
| BACK | Re-enable after emergency stop |
| D-Pad Up/Down | Hood angle trim (+/-2 degrees) |

### Teleop Drive Features
- **Heading Lock** — ProfiledPIDController maintains heading when not rotating
- **Slow Mode** — Trigger-based speed reduction for precise positioning
- **Snap-to-Angle** — D-pad sets target heading to cardinal directions
- **Slew Rate Limiting** — Prevents wheel slip on sudden input changes
- **Intake Speed Cap** — Limits chassis speed below roller surface speed while intaking

---

## Autonomous

Two autonomous systems coexist, selectable from the SmartDashboard chooser:

### PathPlanner (Primary)
Dynamic pathfinding using the AD* algorithm with a field navigation grid (`navgrid.json`) for obstacle awareness. Paths are computed at runtime, allowing adaptive routines.

| Routine | Description |
|---|---|
| Preload & Park | Score preloaded ball, park in safe zone |
| Preload & Collect Center | Score preload, collect center field balls |
| Preload & Collect Side | Score preload, collect side station balls |
| Preload & Collect Depot | Score preload, collect from depot |
| Double Trench Score | Two-cycle trench run |
| Aggressive Sweep | Fast multi-ball field sweep |
| Defensive Preload | Score preload, play defense |

### ChoreoLib (Fallback)
Pre-planned trajectories designed in the Choreo GUI. Alliance-mirrored automatically for red/blue symmetry.

| Routine | Description |
|---|---|
| Preload & Park | Safe fallback |
| Center 2-Cycle | Two scoring cycles from center |
| Left/Right 2-Cycle | Two scoring cycles from side |
| Center 3-Cycle | Three scoring cycles from center |
| Shoot While Moving | Continuous shooting during traversal |
| Depot Cycle | Depot ball collection and scoring |

### Pre-Match System Check
A dashboard-triggered command that sequentially validates each subsystem before a match:
1. Swerve drive health
2. Intake deploy/stow
3. Spindexer + baseline current calibration
4. Feeder + baseline current calibration
5. Shooter flywheel spin-up
6. Vision camera connectivity

Results display as pass/fail booleans on SmartDashboard. Baseline currents feed into `BallPresenceEstimator` for sensorless ball detection.

---

## Vision

### Camera Configuration
4 AprilTag cameras with known robot-to-camera transforms, processed via PhotonVision. Each camera's pose estimate is independently validated before fusion into the swerve odometry.

### Pose Estimation Pipeline
```
PhotonLib Camera Result
  ├── Z-axis height check (-0.5m to 1.5m)
  ├── Field boundary check (with configurable margin)
  ├── Pose jump rejection (max jump threshold)
  ├── Timestamp freshness check (±0.5s)
  ├── Single-tag ambiguity filter
  └── Distance-scaled standard deviations (quadratic)
        ├── Single tag: base std devs, rejected beyond max distance
        ├── Multi-tag: tighter std devs
        └── Aggressive multi-tag: tightest std devs
              └── SwerveDrive.addVisionMeasurement()
```

### Distance Validation
Cross-references vision-derived distance to the HUB against odometry-derived distance. Large discrepancies flag suspect measurements.

---

## Logging & Telemetry

### AdvantageKit
- **Real Robot**: Logs to USB stick (`WPILOGWriter`) + NetworkTables (`NT4Publisher`)
- **Simulation**: NetworkTables only
- **Replay**: Full deterministic replay from log files for post-match analysis
- All subsystems use `Logger.recordOutput()` for structured telemetry

### WPILib DataLogManager
All SmartDashboard values and DriverStation data logged automatically to the USB data log.

### Telemetry Highlights
- Per-module swerve state (velocity, angle, health)
- Shooter lookup table interpolation (target RPS, hood angle, turret heading)
- Vision pose estimates with confidence levels
- Superstructure state machine transitions
- Intake left/right motor voltages, gravity feedforward values, PID output
- Battery voltage, CAN utilization, loop timing

---

## Constants & Tuning

All tunable values live in `Constants.java` (~1,073 lines), organized by subsystem:

| Inner Class | Contents |
|---|---|
| `OperatorConstants` | Controller ports, deadband |
| `SwerveConstants` | CAN IDs, gear ratios, wheel dimensions, PID/FF gains, kinematics |
| `HubConstants` | Hub positions, scoring geometry |
| `ShooterConstants` | Flywheel/hood/turret PID, lookup table ranges, shoot-while-moving gains |
| `IntakeConstants` | Deploy PID, gravity lookup tables (left/right), roller speeds, stall thresholds |
| `SpindexerConstants` | Motor speeds, jam detection thresholds |
| `FeederConstants` | Motor speeds, beam break config, current limits |
| `VisionConstants` | Camera transforms, std devs, pose validation thresholds |
| `LEDConstants` | LED port, buffer size, animation parameters |
| `AutoConstants` | Trajectory constraints, pathfinding parameters |
| `BallDetectionConstants` | Current baselines for sensorless ball detection |

### Naming Convention
- `k` prefix for class-level constants (e.g., `kDeployP`, `kMaxSpeedMetersPerSecond`)
- `UPPER_CASE` for static finals

---

## Testing

13 JUnit 5 test files covering all subsystems and utilities. Tests validate constants, math logic, lookup tables, state transitions, and detection algorithms without requiring hardware.

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
Desktop simulation is enabled in `build.gradle`. Running `simulateJava` launches the WPILib SimGUI alongside a simulated DriverStation. `SwerveDriveSim` provides realistic swerve module physics using `DCMotorSim` and Phoenix 6 simulation APIs.

---

## Hardware Configuration

### Drivetrain
| Component | Specification |
|---|---|
| Modules | 4x SDS MK4i L2 |
| Drive Motors | Kraken X60 (TalonFX) |
| Steer Motors | Kraken X60 (TalonFX) |
| Encoders | CANcoder (absolute) |
| IMU | Pigeon2 |
| Drive Gear Ratio | 6.75:1 |
| Steer Gear Ratio | 21.43:1 |
| Wheel Diameter | ~4 inches (0.1016 m) |
| Max Speed | ~4-6 m/s |

### CAN Bus IDs

| Module | Drive | Steer | CANcoder |
|---|---|---|---|
| Front Left | 1 | 2 | 3 |
| Front Right | 4 | 5 | 6 |
| Back Left | 7 | 8 | 9 |
| Back Right | 10 | 11 | 12 |
| Pigeon2 IMU | 13 | — | — |

### Odometry
250Hz dedicated thread for high-frequency pose updates, fused with multi-camera AprilTag vision measurements.

---

## Project Structure

```
robot2026/
├── src/
│   ├── main/
│   │   ├── java/frc/robot/
│   │   │   ├── Main.java                    # Entry point
│   │   │   ├── Robot.java                   # Lifecycle (LoggedRobot)
│   │   │   ├── RobotContainer.java          # Assembly & bindings
│   │   │   ├── Constants.java               # All tunable values
│   │   │   ├── ShooterPhysics.java          # Projectile motion calculations
│   │   │   ├── HubStateTracker.java         # Game state tracking
│   │   │   ├── RobotState.java              # Health monitoring
│   │   │   ├── BallPresenceEstimator.java   # Sensorless ball detection
│   │   │   ├── subsystems/
│   │   │   │   ├── SwerveDrive.java         # Swerve drivetrain
│   │   │   │   ├── SwerveModule.java        # Individual module
│   │   │   │   ├── SwerveDriveSim.java      # Simulation support
│   │   │   │   ├── Shooter.java             # Flywheel + Hood + Turret
│   │   │   │   ├── Superstructure.java      # State machine coordinator
│   │   │   │   ├── Intake.java              # Slapdown arm + rollers
│   │   │   │   ├── Feeder.java              # Ball transport
│   │   │   │   ├── Spindexer.java           # Ball indexing hopper
│   │   │   │   ├── Vision.java              # AprilTag processing
│   │   │   │   └── LEDs.java                # Driver feedback
│   │   │   └── commands/
│   │   │       ├── SwerveDriveCommand.java  # Teleop driving
│   │   │       ├── Autos.java               # ChoreoLib trajectories
│   │   │       ├── OnTheFlyAutos.java       # PathPlanner pathfinding
│   │   │       ├── VisionIntakeCommand.java # Vision ball chase
│   │   │       └── SysIdCommands.java       # System identification
│   │   └── deploy/
│   │       └── pathplanner/
│   │           └── navgrid.json             # Field navigation grid
│   └── test/
│       └── java/frc/robot/
│           └── (13 test files)
├── vendordeps/                               # Vendor dependency JSONs
├── build.gradle                              # GradleRIO build config
├── settings.gradle                           # Gradle settings
└── CLAUDE.md                                 # AI assistant context
```

---

## Vendor Dependencies

| Library | Version | Purpose |
|---|---|---|
| [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) | 26.0.0 | Structured logging with deterministic replay |
| [Phoenix 6](https://pro.docs.ctr-electronics.com/) | 26.1.1 | CTRE hardware (TalonFX, CANcoder, Pigeon2) |
| [PhotonLib](https://docs.photonvision.org/) | v2026.2.2 | AprilTag vision processing |
| [REVLib](https://docs.revrobotics.com/) | 2026.0.1 | REV Robotics hardware support |
| [PathplannerLib](https://pathplanner.dev/) | 2026.1.2 | Dynamic pathfinding (AD* algorithm) |
| [ChoreoLib](https://choreo.autos/) | 2026.0.1 | Pre-planned trajectory following |
| WPILibNewCommands | 1.0.0 | Command-based framework |

---

## License

This project is open source under the WPILib BSD license.

---

*FRC Team 7840 — 2026 Season*
