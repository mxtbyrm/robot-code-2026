# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 7840 robot code for the 2026 "REBUILT" game season. Java-based, using WPILib's command-based framework with GradleRIO build system. Targets RoboRIO deployment.

## Build & Run Commands

```bash
./gradlew build            # Compile + run tests
./gradlew test             # Run JUnit 5 tests only
./gradlew simulateJava     # Desktop robot simulation with GUI
./gradlew deploy           # Deploy to RoboRIO (requires USB/network connection)
./gradlew clean            # Remove build artifacts
```

There is no configured linter or formatter.

## Architecture

### Entry Point & Lifecycle
`Main.java` → `Robot.java` (extends AdvantageKit `LoggedRobot`) → `RobotContainer.java` (assembly & bindings)

### Key Coordination Pattern
**Superstructure** (`subsystems/Superstructure.java`) is the central state machine coordinating Intake, Spindexer, Feeder, and Shooter subsystems. States: IDLE, INTAKING, PRE_FEED_REVERSE, SHOOTING, SHOOTING_ALLIANCE, OUTTAKING, UNJAMMING, DISABLED. Most operator actions go through Superstructure command factories rather than controlling subsystems directly.

### Subsystems (10 total, in `src/main/java/frc/robot/subsystems/`)
- **SwerveDrive** — 4 MK4i swerve modules (Kraken X60 + CANCoders), Pigeon2 IMU, 250Hz odometry thread, field-relative driving, PathPlanner/ChoreoLib integration
- **Shooter** — Flywheel + Hood + Turret with distance-based lookup tables (InterpolatingDoubleTreeMap), shoot-while-moving compensation, alliance-aware HUB targeting
- **Superstructure** — State machine coordinating intake/feed/shoot pipeline
- **Feeder** — Ball transport with beam break detection
- **Spindexer** — Ball indexing with jam detection via current monitoring
- **Intake** — Arm deploy/stow + roller motor
- **Vision** — PhotonLib AprilTag processing, multi-camera, odometry fusion
- **LEDs** — Driver feedback indicators
- **SwerveModule** — Individual module control (drive + steer + encoder)
- **SwerveDriveSim** — Desktop simulation support

### Commands (`src/main/java/frc/robot/commands/`)
- **SwerveDriveCommand** — Default drive command with heading lock, slow mode, snap-to-angle
- **Autos** — ChoreoLib pre-planned trajectory routines (alliance-mirrored)
- **OnTheFlyAutos** — PathPlanner dynamic pathfinding routines
- **VisionIntakeCommand** — Vision-assisted ball chase

### Constants & Physics
- **Constants.java** (~53KB) — All tunable values organized by subsystem (CAN IDs, PID gains, gear ratios, speeds, dimensions). This is the single source of truth for hardware configuration.
- **ShooterPhysics.java** — Projectile motion computation generating flywheel RPS and hood angle lookup tables from distance
- **HubStateTracker.java** — Tracks HUB active/inactive shift pattern from FMS game data with heuristic fallback
- **RobotState.java** — Singleton health monitor (battery, CAN bus, loop overruns, per-subsystem health)
- **BallPresenceEstimator.java** — Motor current baseline calibration for ball detection

### Hardware Layout
- Swerve CAN IDs: FL(1,2,3), FR(4,5,6), BL(7,8,9), BR(10,11,12), Pigeon2(13)
- Drive gear ratio 6.75:1, steer gear ratio 21.43:1 (MK4i L2)
- Field: Blue HUB at (4.03, 4.035), Red HUB at (12.51, 4.035)

### Vendor Dependencies (in `vendordeps/`)
AdvantageKit 26.0.0, Phoenix 6 26.1.1 (CTRE), PhotonLib, REVLib, PathplannerLib 2026.1.2, ChoreoLib 2026, WPILibNewCommands

### Logging
AdvantageKit structured logging with replay capability. Real robot logs to USB stick (WPILOGWriter) and NetworkTables (NT4Publisher). Use `Logger.recordOutput()` for telemetry.

### Autonomous
Two systems coexist: **ChoreoLib** for pre-planned trajectories and **PathPlanner** for dynamic on-the-fly pathfinding. Auto routines are selected via SmartDashboard. PathPlanner configs live in `src/main/deploy/pathplanner/`.

## Code Conventions
- Constants use `K` prefix for class-level constants, `UPPER_CASE` for static finals
- All subsystems extend `SubsystemBase` and report health status
- Singletons used for RobotState, HubStateTracker, BallPresenceEstimator
- Supplier pattern for injecting pose/velocity data between subsystems (e.g., SwerveDrive → Shooter)
- Java 17 source/target compatibility
