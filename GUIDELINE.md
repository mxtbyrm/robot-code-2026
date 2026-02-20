# Robot Configuration Guideline — FRC Team 7840 (2026 "REBUILT")

**The only file you ever edit to deploy to a new robot is:**
```
src/main/java/frc/robot/RobotConfig.java
```
Everything else (subsystems, commands, constants) reads from `RobotConfig` automatically.
This guide walks through every section in order, explains what each value means, and tells you exactly how to measure or calibrate it.

---

## Table of Contents

1. [Coordinate System Reference](#coordinate-system-reference)
2. [Tools You Need](#tools-you-need)
3. [Step-by-Step Configuration Order](#step-by-step-configuration-order)
4. [Section 1 — Operator Controller Ports](#section-1--operator-controller-ports)
5. [Section 2 — LED Hardware](#section-2--led-hardware)
6. [Section 3 — Swerve CAN Wiring](#section-3--swerve-can-wiring)
7. [Section 4 — Swerve Module Hardware](#section-4--swerve-module-hardware)
8. [Section 5 — Swerve PID / FF / Driver Params](#section-5--swerve-pid--ff--driver-params)
9. [Section 6 — Shooter CAN Wiring](#section-6--shooter-can-wiring)
10. [Section 7 — Shooter Hardware](#section-7--shooter-hardware)
11. [Section 8 — Shooter Geometry](#section-8--shooter-geometry)
12. [Section 9 — Hood Calibration](#section-9--hood-calibration)
13. [Section 10 — Turret Calibration](#section-10--turret-calibration)
14. [Section 11 — Shooter Tuning](#section-11--shooter-tuning)
15. [Section 12 — Feeder & Spindexer](#section-12--feeder--spindexer-can--tuning)
16. [Section 13 — Intake CAN Wiring](#section-13--intake-can-wiring)
17. [Section 14 — Intake Hardware / Calibration](#section-14--intake-hardware--calibration)
18. [Section 15 — Camera Hardware](#section-15--camera-hardware-per-camera-6-dof-pose)
19. [Section 16 — Vision Tuning](#section-16--vision-tuning)
20. [Section 17 — Ball Detection Tuning](#section-17--ball-detection-tuning)
21. [Section 18 — Swerve Geometry](#section-18--swerve-geometry)
22. [Section 19 — CANcoder Offsets](#section-19--cancoder-offsets)
23. [SysId Characterization Workflow](#sysid-characterization-workflow)
24. [Turret Spring Feedforward](#turret-spring-feedforward)
25. [Pre-Match Checklist](#pre-match-checklist)
26. [Competition Day Procedures](#competition-day-procedures)
27. [Common Failure Modes & Diagnostics](#common-failure-modes--diagnostics)
28. [AdvantageScope Tuning Reference](#advantagescope-tuning-reference)
29. [Quick Reference — All RobotConfig Fields](#quick-reference--all-fields-in-robotconfigjava)

---

## Coordinate System Reference

All robot-frame measurements use the **WPILib standard**:

| Axis | Direction | Unit |
|------|-----------|------|
| X | Forward (toward shooter front) | meters |
| Y | Left | meters |
| Z | Up | meters |
| Yaw | CCW positive, 0 = facing forward | radians or degrees |
| Pitch | **Negative = tilted upward** (WPILib convention) | radians or degrees |
| Roll | CCW positive around X axis | radians or degrees |

The **robot center** is the point on the floor equidistant from all four swerve module wheel contact patches (center of the four axles).

**Field coordinate system:**
- Origin = bottom-left corner of the field (from the Blue alliance driver station perspective)
- X increases toward the Red alliance wall
- Y increases toward the top of the field
- Blue HUB center: (4.03 m, 4.035 m)
- Red HUB center: (12.51 m, 4.035 m)

---

## Tools You Need

| Tool | Purpose |
|------|---------|
| **Phoenix Tuner X** (Windows/Android) | Read CAN IDs, zero encoders, read live motor signals, run self-test |
| **FRC Driver Station** | Set controller ports, monitor CAN bus health |
| **AdvantageScope** | Plot telemetry for PID tuning, verify odometry, watch mechanism behavior |
| **Digital angle gauge / digital level** | Measure camera pitch/roll, turret mount angle |
| **Calipers** | Measure wheel diameter, flywheel diameter, roller diameter |
| **Tape measure / ruler** | Swerve geometry, camera positions, shooter offset |
| **Straight edge (long ruler or piece of aluminum)** | CANcoder offset calibration (align wheels) |
| **WPILib SysId tool** | Flywheel and drive motor characterization (kS, kV, kA) |

---

## Step-by-Step Configuration Order

Follow this order. Steps marked **[HARDWARE]** require the physical robot. Steps marked **[SOFTWARE]** can be done at a desk.

```
 1. [SOFTWARE] CAN IDs & ports
 2. [HARDWARE] Swerve geometry (track width, wheelbase)
 3. [HARDWARE] CANcoder offsets
 4. [HARDWARE] Swerve module hardware (gear ratio, wheel diameter)
 5. [HARDWARE] Swerve current limits
 6. [HARDWARE] Drive + steer PID / FF  (use SysId for kS/kV/kA)
 7. [HARDWARE] Intake deploy calibration (travel endpoint)
 8. [HARDWARE] Intake current limits & roller speeds
 9. [HARDWARE] Hood calibration
10. [HARDWARE] Turret calibration
11. [HARDWARE] Shooter geometry (offset + exit height)
12. [HARDWARE] Camera poses (position + full rotation, each camera)
13. [HARDWARE] Ball detection baselines
14. [HARDWARE] Flywheel PID / FF  (use SysId for kS/kV/kA)
15. [HARDWARE] Shooter physics tuning (efficiency, bias)
16. [HARDWARE] Turret spring feedforward
17. [HARDWARE] Alliance zone lob tuning
18. [HARDWARE] Pre-feed reverse tuning
19. [HARDWARE] Intake gravity FF table
20. [SOFTWARE] Remaining driver / LED preferences
```

---

## Section 1 — Operator Controller Ports

```java
kDriverControllerPort   = 0;
kOperatorControllerPort = 1;
kDeadband               = 0.1;
```

**What:** USB port numbers assigned by the Driver Station to each Xbox controller.

**How:** Plug in both controllers. In the FRC Driver Station → USB tab, each joystick appears with its port number (0–5). Match driver and operator to ports 0 and 1.

**Driver layout (do not rebind without updating RobotContainer):**
| Input | Function |
|-------|----------|
| Left stick | Translation (X/Y) |
| Right stick | Rotation |
| Right trigger (>0.3) | Slow mode (35% translation, 40% rotation) |
| X button | X-lock wheels (defense) |
| Y button | Vision ball chase (auto-drives to nearest ball) |
| D-pad | Snap-to-angle (Up=0°, Right=−90°, Down=180°, Left=90°) |
| START | Zero gyro (reset field-relative forward) |

**Operator layout:**
| Input | Function |
|-------|----------|
| Right trigger (>0.3) | SHOOT (Hub tracking + feed) |
| Left trigger (>0.3) | Intake rollers (hold to spin) |
| A button | Intake arm toggle (deploy / stow) |
| Right bumper | Alliance zone dump (fixed lob) |
| Left bumper | Outtake (eject balls) |
| B button | Unjam (reverse everything) |
| D-pad Up/Down | Hood trim ±2° |
| START | Emergency stop all mechanisms |
| BACK | Re-enable after emergency stop |

**Deadband:** 0.1 (10%) is the default. Increase if the robot drifts with stick at rest; decrease if low-speed response feels sluggish. Watch for controller stick drift in AdvantageScope (`Swerve/RawDriverX`, `Swerve/RawDriverY`).

---

## Section 2 — LED Hardware

```java
kLedPort  = 0;   // RoboRIO PWM port
kLedCount = 60;  // total LEDs on the strip
```

**What:** The RoboRIO PWM port that the LED strip's data line connects to, and the total LED count.

**How:** Check the wiring diagram for the PWM port. Count the LEDs or read the strip datasheet. Getting `kLedCount` wrong causes patterns to wrap mid-strip or cut off early.

**LED feedback meanings:**
| Pattern | Meaning |
|---------|---------|
| Solid green | All systems healthy |
| Yellow (pulsing) | Minor subsystem fault |
| Red | Major fault — check DriverStation |
| Blue (dim) | Flywheel idle / not tracking |
| Cyan (breathing) | Flywheel spinning up |
| White (solid) | Ready to shoot |
| Blinking white | HUB currently inactive |
| Load bar | Hopper fill level |

---

## Section 3 — Swerve CAN Wiring

```java
kPigeonId = 13;

kFrontLeftDriveMotorId = 1;   kFrontLeftSteerMotorId = 2;   kFrontLeftEncoderId = 3;
kFrontRightDriveMotorId = 4;  kFrontRightSteerMotorId = 5;  kFrontRightEncoderId = 6;
kBackLeftDriveMotorId = 7;    kBackLeftSteerMotorId = 8;    kBackLeftEncoderId = 9;
kBackRightDriveMotorId = 10;  kBackRightSteerMotorId = 11;  kBackRightEncoderId = 12;
```

**What:** CAN IDs for the Pigeon2 gyro and the 12 TalonFX / CANcoder devices across 4 swerve modules.

**How:**
1. Open **Phoenix Tuner X** → connect via USB (robot enabled not required).
2. "Devices" tab shows all CAN devices. Match each physical module to its motors and encoder by physically noting which wires go where.
3. To change a CAN ID in Tuner X: select device → "Config" → set ID → save. Do this for any that do not match the above.

> Convention: FL = 1,2,3 / FR = 4,5,6 / BL = 7,8,9 / BR = 10,11,12 / Pigeon = 13. Shooter motors start at 20+, intake at 24+.

**Pigeon2:** Must be on the **CANivore bus** (separate USB-to-CAN adapter, not the RoboRIO CAN bus) to support the 250 Hz odometry thread. Verify the Pigeon2 is assigned to the CANivore bus in Tuner X.

---

## Section 4 — Swerve Module Hardware

```java
kDriveGearRatio      = 6.75;         // MK4i L2
kSteerGearRatio      = 150.0 / 7.0;  // ~21.43:1, all MK4i variants
kWheelDiameterMeters = Units.inchesToMeters(4.0);
```

**Drive gear ratio** — check which MK4i configuration you have:
| MK4i Level | Ratio | Max Speed |
|------------|-------|-----------|
| L1 | 8.14:1 | ~3.7 m/s |
| L2 | 6.75:1 | ~4.5 m/s ← this robot |
| L3 | 6.12:1 | ~5.0 m/s |
| L4 | 5.14:1 | ~5.9 m/s |

**Steer gear ratio:** 150/7 ≈ 21.43 for ALL MK4i variants — do not change.

**Wheel diameter:** Measure with calipers **after some use** (wheels wear smaller). Even a 2 mm error compounds into visible odometry drift over a full-field traversal. Use `Units.inchesToMeters(measured_inches)`.

---

## Section 5 — Swerve PID / FF / Driver Params

### Drive Motor PID + FF (velocity control)

```java
kDriveP = 0.1;   kDriveI = 0.0;   kDriveD = 0.0;
kDriveS = 0.0;   kDriveV = 0.12;  kDriveA = 0.0;
```

**Best practice: use SysId to characterize kS, kV, kA first**, then set kP for residual error correction. See [SysId Characterization Workflow](#sysid-characterization-workflow).

**Manual tuning order (if SysId is not available):**
1. **`kDriveV` first** — `kDriveV ≈ 1 / freeSpeedRPS`. Kraken X60 free speed ≈ 100 RPS at 12 V → `kDriveV ≈ 0.01 V·s/rot`. Increase from 0 until the wheel tracks a constant velocity setpoint without needing P.
2. **`kDriveS`** — apply a slowly increasing constant voltage until the wheel barely starts moving. That voltage is `kDriveS`.
3. **`kDriveP`** — increase from 0 until velocity tracking error is corrected quickly without oscillation.
4. Leave `kDriveI` and `kDriveA` at 0 unless you have persistent steady-state velocity error.

### Swerve Current Limits

```java
kDriveCurrentLimit        = 60.0;  // supply current (breaker protection)
kDriveStatorCurrentLimit  = 80.0;  // stator current (wheel slip prevention)
kSteerCurrentLimit        = 30.0;  // supply current
```

**Supply current** protects wiring, PDH, and breakers. **Stator current** prevents wheel spin under hard acceleration.

| Symptom | Fix |
|---------|-----|
| Drive breakers trip | Lower `kDriveCurrentLimit` |
| Wheel slip during hard accel | Lower `kDriveStatorCurrentLimit` |
| Steer motors run hot | Lower `kSteerCurrentLimit` |
| Robot feels sluggish / weak | Carefully increase current limits |

### Steer Motor PID

```java
kSteerP = 100.0;   kSteerI = 0.0;   kSteerD = 0.5;
```

MK4i steer has low inertia — high P is normal and expected. If the wheel oscillates around the target angle, increase `kSteerD`. If it undershoots badly, increase `kSteerP`. The steer motor uses **Motion Magic** — tune `kSteerP` against the motion profile, not against a step input.

### Auto Trajectory PID

```java
kAutoTranslationP = 10.0;   kAutoTranslationI = 0.0;   kAutoTranslationD = 0.0;
kAutoRotationP    = 7.5;    kAutoRotationI    = 0.0;   kAutoRotationD    = 0.0;
```

**What:** Correction gains for Choreo/PathPlanner to fix positional error during autonomous. These are applied on top of the feedforward trajectory following.

**Tuning:**
1. Run a straight-line path 3 m forward and back. Plot `Swerve/OdometryPose` vs the planned path in AdvantageScope.
2. If the robot consistently overshoots the endpoint → lower Translation P.
3. If it consistently arrives late → increase Translation P.
4. Rotation P controls heading correction during path following — increase if heading drifts, decrease if it oscillates.

### Heading Lock PID

```java
kHeadingLockP = 5.0;   kHeadingLockI = 0.0;   kHeadingLockD = 0.3;
kMaxAngularAccelRadiansPerSecondSq = 8.0;
```

**What:** Holds the robot's heading when the driver releases the rotation stick.

**Tuning:**
- Increase P until heading snaps back quickly after bumps.
- Add D if heading overshoots and oscillates when released.
- Lower `kMaxAngularAccelRadiansPerSecondSq` if the snap back feels too jerky for drivers.

### Slow Mode & Slew Rate

```java
kSlowModeSpeedMultiplier = 0.35;
kSlowModeRotMultiplier   = 0.40;
kTranslationSlewRate     = 6.0;   // m/s²
kRotationSlewRate        = 8.0;   // rad/s²
```

**Slow mode** (right trigger): Tune based on driver preference for precision near scoring.

**Slew rate:** Limits acceleration. Higher = more responsive but more wheel spin. Watch AdvantageScope `Swerve/SlipDetected` — if wheel slip is frequent during hard direction changes, lower translation slew rate.

---

## Section 6 — Shooter CAN Wiring

```java
kFlywheelMotorId = 20;
kHoodMotorId     = 21;
kTurretMotorId   = 22;
```

Same procedure as Section 3 — verify all three IDs in Phoenix Tuner X against physical wiring.

---

## Section 7 — Shooter Hardware

```java
kFlywheelGearRatio            = 2.0;   // motor rotations per bottom flywheel rotation
kTopToBottomGearRatio         = 2.0;   // top flywheel spins 2× faster than bottom
kBottomFlywheelDiameterMeters = Units.inchesToMeters(4.0);
kTopFlywheelDiameterMeters    = Units.inchesToMeters(2.0);
kHoodGearRatio                = 2.0;
kTurretHeightMeters           = 0.45;
kHoodHeightMeters             = 0.50;
```

**Flywheel diameters:** Measure with calipers — the physics model uses surface speed to compute exit velocity. Even 3 mm of error shifts the entire shot table.

**Gear ratios:** Count teeth or verify from CAD.
- `kFlywheelGearRatio` = motor rotations per bottom flywheel rotation (e.g., if 1 motor rotation → 0.5 flywheel rotation, ratio = 2.0).
- `kTopToBottomGearRatio` = how many times faster the top flywheel spins, sized so both flywheels have equal surface speed. 4-inch bottom + 2-inch top → top must spin 2× faster.

**Heights:** Measure from carpet (not the frame) to the turret rotation axis and to the hood pivot. These feed directly into projectile motion trajectory calculations.

---

## Section 8 — Shooter Geometry

```java
kShooterPositionOffset   = new Translation2d(0.0, 0.0);  // TODO: measure
kShooterExitHeightMeters = Units.inchesToMeters(24.0);   // TODO: measure
```

### Shooter Position Offset
The offset of the ball exit point from the robot's odometry center, in the **robot frame** (X = forward, Y = left).

**Measurement procedure:**
1. Place the robot on flat surface. Drop a plumb line to mark the robot center on the floor (equidistant from the 4 wheel patches).
2. Drop a plumb line from the flywheel exit gap. Measure the distance between the two floor marks.
3. X = forward component (positive = ahead of center). Y = lateral component (positive = left of center).
4. Example: 10 cm forward, 2 cm right → `Translation2d(0.10, -0.02)`.

> Tip: If the shooter is roughly centered on the robot, start with `(0, 0)` for initial bring-up. The offset shifts auto-aim accuracy but the robot will still shoot.

### Exit Height
Height from carpet to the center of the flywheel gap.

**Measurement:** Hold a ruler vertically beside the shooter. Measure from floor to flywheel gap center. Convert: `Units.inchesToMeters(yourMeasurement)`.

---

## Section 9 — Hood Calibration

```java
kHoodMinMotorRotations = 0.0;  // TODO — motor rotations at fully open hard stop
kHoodMaxMotorRotations = 0.0;  // TODO — motor rotations at fully closed hard stop
```

**What:** Kraken encoder readings at the two physical hood hard stops. Used to map motor position → hood angle in degrees.

**Procedure:**
1. Power on robot. Open **Phoenix Tuner X** → select the hood motor (ID = `kHoodMotorId`).
2. Move hood to its **minimum (most open)** hard stop — the position the ball exits at the steepest arc. Read the "Position" signal (motor rotations). Enter as `kHoodMinMotorRotations`.
3. Move hood to its **maximum (most closed)** hard stop — flattest trajectory. Read and enter as `kHoodMaxMotorRotations`.
4. Code zeroes the encoder to `kHoodMinRotations` at startup. Make sure the hood is physically at the minimum position when the robot powers on, or all angle commands will be offset.

> **Verify:** Command `kHoodMinAngleDegrees` → hood should go to open hard stop. Command `kHoodMaxAngleDegrees` → hood should go to closed hard stop. Soft limits prevent overshooting.

---

## Section 10 — Turret Calibration

```java
kTurretGearRatio              = 100.0;  // TODO — measure
kTurretCCWLimitMotorRotations = 0.0;    // TODO — calibrate
kTurretCWLimitMotorRotations  = 0.0;    // TODO — calibrate
kTurretMountOffsetDegrees     = 0.0;
```

### Gear Ratio

1. In Phoenix Tuner X, zero the turret motor encoder with the turret at a marked reference position.
2. Manually rotate the turret **exactly one full mechanical revolution** (reference mark returns to start).
3. Read the motor "Position" signal in Tuner X — that number is `kTurretGearRatio`.

### Hard Stop Limits

1. Power on with turret physically centered and facing robot forward. The encoder is zeroed to 0 at power-on.
2. In Tuner X, slowly command the motor CCW (positive direction) until it reaches the mechanical hard stop. Read motor position → `kTurretCCWLimitMotorRotations` (should be positive).
3. Return to center. Drive CW until the other hard stop. Read motor position → `kTurretCWLimitMotorRotations` (should be negative).

> **Safety:** Set soft limits slightly inside the hard stops (the code adds a 3° buffer). If the limits are set wrong, the turret can slam into hard stops at full PID output.

### Mount Offset

If the turret's mechanical "zero" direction does not align with robot forward (e.g., the sprocket was installed 5° rotated), measure the misalignment with a digital angle gauge and enter as `kTurretMountOffsetDegrees`. CCW from robot forward is positive. Leave at 0.0 if well-aligned.

---

## Section 11 — Shooter Tuning

### Efficiency Factor

```java
kShooterEfficiencyFactor = 0.30;
```

**What:** Fraction of flywheel surface speed that transfers to ball exit velocity. 0.30 = 30% transfer (rest is lost to compression, slip, and friction with foam balls).

**Tuning:**
1. Start at 0.30.
2. Shoot from a known distance (e.g., 3 m from the HUB center). Use a tape measure on the field.
3. Shots consistently short → increase. Consistently long → decrease. Step by 0.02 at a time.
4. Dial this in **before** touching bias factors.

### Bias Factors

```java
kBiasFactorNear = 1.00;  // multiplier at minimum shooting distance (~1.2 m)
kBiasFactorFar  = 1.00;  // multiplier at maximum shooting distance (~6.5 m)
```

**What:** Corrects for air drag and ball spin that the vacuum projectile model ignores. Linearly interpolated between near and far distances.

**Tuning (after efficiency is dialled in):**
1. Shoot at minimum distance. Still short → increase `kBiasFactorNear` by 0.02.
2. Shoot at maximum distance. Short at range but fine up close → increase `kBiasFactorFar`.
3. Mid-range shots interpolate automatically — recheck at 3–4 m after changing either bias.

### Flywheel PID + FF

```java
kFlywheelP = 0.5;   kFlywheelI = 0.0;   kFlywheelD = 0.0;
kFlywheelS = 0.0;   kFlywheelV = 0.12;  kFlywheelA = 0.0;
kFlywheelToleranceRPS = 2.0;
kIdleFlywheelRPS      = 15.0;
kMaxFlywheelRPS       = 90.0;
```

**Use SysId for kS/kV/kA** — see [SysId Characterization Workflow](#sysid-characterization-workflow).

Manual tuning: same approach as drive motor — `kFlywheelV` first, then `kFlywheelS`, then `kFlywheelP`. Monitor `Shooter/FlywheelActualRPS` vs `Shooter/FlywheelTargetRPS` in AdvantageScope.

| Parameter | Effect |
|-----------|--------|
| `kFlywheelToleranceRPS` | Tighten for more consistent shots; loosen to reduce wait time |
| `kIdleFlywheelRPS` | Higher = faster spin-up, more battery draw between shots |
| `kMaxFlywheelRPS` | Must be ≥ highest RPS in the physics table, used to normalize feeder duty |

### Hood PID + Gravity Feedforward

```java
kHoodP = 8.0;   kHoodI = 0.0;   kHoodD = 0.2;   kHoodG = 0.15;
```

**`kHoodG`** is the `GravityType.Arm_Cosine` feedforward — Phoenix 6 automatically scales it by `cos(angle)` to match the arm's gravity torque at every position.

**Tuning `kHoodG`:**
1. Disable PID temporarily (set `kHoodP = 0`).
2. Command the hood to 45°.
3. Increase `kHoodG` until the hood holds 45° without drifting. The cosine compensation means this should hold all angles.
4. Re-enable P and tune for tracking speed.

### Turret PID

```java
kTurretP = 30.0;   kTurretI = 0.0;   kTurretD = 0.5;
```

High P is required for fast target tracking. If the turret oscillates around the target, increase D. If it lags on fast target motion (robot driving quickly while tracking), increase P. Keep `kTurretI = 0` — use the [spring feedforward](#turret-spring-feedforward) instead of integral to handle steady-state offset.

### Alliance Zone Dump

```java
kAllianceZoneFlywheelRPS    = 35.0;   // low-power lob
kAllianceZoneHoodAngleDeg   = 40.0;   // steep arc
kAllianceZoneTurretAngleDeg = 172.0;  // facing nearly backward
kAllianceZoneFeederSpeed    = 0.60;
```

**What:** Fixed parameters for lobbing FUEL from the neutral zone into the alliance zone during Shifts / Endgame (no auto-aim — turret faces backward at a fixed angle).

**Tuning procedure:**
1. Position robot in the neutral zone at the intended dump position.
2. Enable alliance zone dump mode (operator right bumper).
3. Adjust `kAllianceZoneHoodAngleDeg` (steeper arc = higher angle = shorter horizontal range) and `kAllianceZoneFlywheelRPS` (more speed = longer throw) until balls land in the alliance zone.
4. If the target is off-center laterally, adjust `kAllianceZoneTurretAngleDeg` from 172° (max = 175°).
5. Re-tune whenever the dump position changes.

### Pre-Feed Reverse

```java
kPreFeedReverseDurationSeconds = 0.12;  // 120 ms
kPreFeedReverseFeederSpeed     = -0.25;
kPreFeedReverseSpindexerSpeed  = -0.15;
```

**What:** A brief backward pulse before feeding that creates a small gap between the next ball and the flywheel gap. Prevents jams when a ball is sitting against a spinning flywheel.

**Tuning:**
- If balls still jam entering the flywheel → increase duration to 0.15–0.20 s.
- If the reverse is too aggressive (ball falls back / re-jams) → reduce duration or speeds.
- Speeds must be gentle — this is clearance, not unjam.

---

## Section 12 — Feeder & Spindexer CAN + Tuning

```java
kFeederMotorId    = 23;
kBeamBreakDIOPort = 0;     // TODO: set actual DIO port
kSpindexerMotorId = 27;

kFeederSpeedRatio       = 0.75;
kSpindexerIntakeSpeed   = 0.4;
kSpindexerToFeederRatio = 0.65;

kFeederCurrentLimit    = 30.0;
kSpindexerCurrentLimit = 30.0;
```

**Beam break DIO port:** Check the RoboRIO DIO wiring — valid ports are 0–9. Use a multimeter to confirm which port the beam-break sensor connects to.

**Critical speed hierarchy (must always be maintained to prevent jams):**
```
Flywheel surface speed  >>  Feeder  >  Spindexer  >  Intake roller
```
This chain is enforced automatically as long as `kFeederSpeedRatio < 1.0` and `kSpindexerToFeederRatio < 1.0`.

| Symptom | Fix |
|---------|-----|
| Balls jam between spindexer and feeder | Reduce `kSpindexerToFeederRatio` |
| Balls jam entering the flywheel gap | Reduce `kFeederSpeedRatio` |
| Balls fall back from feeder to spindexer | Increase `kFeederSpeedRatio` |
| Spindexer current trips at 30 A | Increase `kSpindexerCurrentLimit` slightly |

---

## Section 13 — Intake CAN Wiring

```java
kLeftDeployMotorId  = 24;
kRightDeployMotorId = 25;
kRollerMotorId      = 26;
```

Verify in Phoenix Tuner X. The right deploy motor is **mirror-inverted in software** — no hardware wiring change needed.

---

## Section 14 — Intake Hardware / Calibration

### Deploy Travel Calibration

```java
kDeployGearRatio              = 4.0;
kIntakeExtendedMotorRotations = -0.01293;  // TODO: re-measure after any rebuild
```

**`kIntakeExtendedMotorRotations`** is the raw Kraken motor encoder reading when the arm is at full extension (floor contact).

**Procedure:**
1. Power on with arm stowed (encoder = 0 at startup).
2. In Phoenix Tuner X or via a test mode, drive the left deploy motor until the arm is fully deployed.
3. Read the "Position" signal (motor rotations). Enter the value (negative if extension is in the negative direction).

> Re-measure any time the intake is rebuilt, the hard stop changes, or the motor is swapped.

### Deploy PID + Motion Magic

```java
kDeployP = 40.0;   kDeployI = 0.0;   kDeployD = 1.0;   kDeployS = 0.12;
kDeployMaxVelocity     = 4.0;   // rot/s
kDeployMaxAcceleration = 8.0;   // rot/s²
```

The intake deploy uses **Motion Magic** (trapezoidal velocity profile) — tune velocity/acceleration limits first, then PID.

**Tuning order:**
1. Set `kDeployMaxVelocity` and `kDeployMaxAcceleration` to conservative values (3.0 / 6.0). Increase until arm extends smoothly without slamming hard stops.
2. `kDeployS` — static friction: slowly apply voltage until arm just starts moving.
3. `kDeployP` — increase until arm reaches target without overshoot. High P is normal with Motion Magic since the profile controls velocity.
4. `kDeployD` — add if the arm oscillates at the target position.

### Roller Speeds

```java
kRollerIntakeSpeed  =  0.8;   // collecting balls inward
kRollerOuttakeSpeed = -0.6;   // ejecting balls outward
```

**`kRollerIntakeSpeed`:** Roller surface speed must exceed chassis speed (the code enforces this at 85%). Start at 0.8. If balls are pushed away instead of collected → increase. If roller jams → decrease slightly.

**`kRollerOuttakeSpeed`:** Tune so balls are ejected cleanly and far enough not to re-enter.

### Deploy Gravity FF Voltage Tables

```java
kDeployLeftGravityVoltages  = { 0.0, 0.10, 0.25, 0.35, 0.40 };  // 0%, 25%, 50%, 75%, 100% deploy
kDeployRightGravityVoltages = { 0.0, 0.15, 0.35, 0.50, 0.60 };
```

**What:** Voltage feedforward at 5 positions across the arm's travel. Applied in addition to PID to counteract gravity — left and right are separate because the right side carries more weight (roller + linkage).

**Tuning using AdvantageScope:**
1. Plot `Intake/LeftDeploySupplyCurrent` and `Intake/RightDeploySupplyCurrent`.
2. Hold arm at stowed (0%) — record idle current as your baseline.
3. Command arm to 25%, hold for 2 seconds — observe current. Increase the `[1]` entry until current drops to baseline level.
4. Repeat for 50%, 75%, 100%.
5. With correct values, current is flat across all arm positions and P correction is near zero.

---

## Section 15 — Camera Hardware (Per-Camera 6-DOF Pose)

Each camera has its own independent 6-axis pose. Do not assume any two cameras share height, pitch, or yaw.

### Field Definitions

| Suffix | Unit | Description |
|--------|------|-------------|
| `X` | meters | Forward from robot center (positive = toward front) |
| `Y` | meters | Left from robot center (positive = left) |
| `Z` | meters | Height above floor (positive = up) |
| `Roll` | radians | Rotation about forward axis (0 = level) |
| `Pitch` | radians | **Negative = tilted upward** (WPILib). −15° tilt = `degreesToRadians(-15.0)` |
| `Yaw` | radians | CCW from robot forward. 0 = forward, 90° = left, −90° = right |

### Cameras to Configure

| Prefix | Purpose | Pipeline |
|--------|---------|---------|
| `kFrontLeftCam` | AprilTag localization | AprilTag |
| `kFrontRightCam` | AprilTag localization | AprilTag |
| `kBackLeftCam` | AprilTag localization | AprilTag |
| `kBackRightCam` | AprilTag localization | AprilTag |
| `kIntakeCam` | Ball detection (ML) | Object Detection — NOT AprilTag |
| `kSideCam` | Lateral awareness | AprilTag or None |

### Measurement Procedure

1. Place robot on a flat surface. Mark the robot center on the floor (equidistant from all four wheel contact patches).
2. **X, Y:** Measure horizontal from floor mark to directly below the camera lens.
3. **Z:** Measure vertically from floor to the camera optical center (the lens, not the bracket or PCB).
4. **Yaw:** Use a digital angle gauge on the camera body. CCW from robot forward is positive.
5. **Pitch:** Tilt from horizontal — enter negative for upward tilt (camera looking up = negative pitch).
6. **Roll:** Most cameras are mounted level (roll = 0).

> **Best source:** If the robot was designed in CAD (Onshape, Fusion 360, SolidWorks), export the camera 6-DOF transforms from the model — more accurate than tape measure.

### Vision Accuracy Tips

- Camera calibration (lens distortion) is done in PhotonVision's web UI — calibrate with a printed ChArUco board. Use at least 50 snapshots at varied distances and angles.
- After physical camera pose measurement, verify the pose estimate in AdvantageScope by driving to a known field position and comparing `Vision/EstimatedPoseField` to the actual position.
- If vision estimates are consistently offset in one direction, re-check the `Yaw` value for the camera that is contributing most.

---

## Section 16 — Vision Tuning

### Pose Estimation Standard Deviations

```java
kMultiTagStdDevs           = { 0.5, 0.5, 1.0 };  // x_m, y_m, heading_rad
kSingleTagStdDevs          = { 4.0, 4.0, 8.0 };
kAggressiveMultiTagStdDevs = { 0.1, 0.1, 0.2 };
```

**What:** These control how much the Kalman pose estimator trusts vision vs wheel odometry. Lower = more trust in vision.

| Symptom | Fix |
|---------|-----|
| Pose jumps erratically in auto | Increase `kMultiTagStdDevs` |
| Odometry drifts, vision doesn't correct | Decrease `kMultiTagStdDevs` |
| Single-tag causes wrong position snap | Increase `kSingleTagStdDevs` |
| Post-collision pose recovery too slow | Decrease `kAggressiveMultiTagStdDevs` |
| Vision pulls pose off when moving fast | Increase all std devs during high-speed driving |

`kSingleTagStdDevs` should always be significantly larger than multi-tag — single-tag has perspective ambiguity (flip problem).

### Ball Chase PID

```java
kBallChaseYawP = 0.04;   kBallChaseYawI = 0.0;   kBallChaseYawD = 0.002;
```

**What:** Yaw correction PID for `VisionIntakeCommand` — centers the robot on a detected ball using the intake camera.

**Tuning:** Increase P if the robot is slow to center. Add D if it oscillates side-to-side while chasing.

---

## Section 17 — Ball Detection Tuning

```java
kDefaultSpindexerBaselineCurrent = 3.0;  // amps, empty hopper spinning
kDefaultFeederBaselineCurrent    = 2.5;  // amps, empty feeder spinning
kSpindexerLoadedThreshold        = 2.0;  // delta amps → has balls
kSpindexerEmptyThreshold         = 0.5;  // delta amps → empty (hysteresis)
kSpindexerFullLoadDelta          = 8.0;  // delta amps → fully loaded
kFeederBallPresentThreshold      = 3.0;  // delta amps → ball in feeder
```

**What:** Ball presence is detected by comparing current draw to an empty-mechanism baseline. Balls add friction → higher current.

**Tuning procedure using AdvantageScope:**
1. Run spindexer + feeder with an **empty hopper** for 10 seconds. Plot `Spindexer/SupplyCurrent` and `Feeder/SupplyCurrent`. Record steady-state average → update `kDefaultSpindexerBaselineCurrent` and `kDefaultFeederBaselineCurrent`.
2. Load **one ball** into the hopper. Record the current delta above baseline.
3. `kSpindexerLoadedThreshold` ≈ 80% of that one-ball delta (detect reliably, ignore noise).
4. `kSpindexerEmptyThreshold` ≈ 20% of the one-ball delta (hysteresis gap prevents flicker).
5. Load a **completely full hopper**. Record spindexer delta → `kSpindexerFullLoadDelta` (used to normalize load level 0–1 for LEDs).
6. Feed one ball through the feeder. Record the current spike → `kFeederBallPresentThreshold`.

> **Note:** The pre-match system check calibrates baseline automatically — run it before every match for best accuracy.

---

## Section 18 — Swerve Geometry

```java
kTrackWidthMeters = Units.inchesToMeters(22.75);  // FL–FR distance
kWheelBaseMeters  = Units.inchesToMeters(22.75);  // FL–BL distance
```

**What:** Center-to-center distances between swerve modules. Used for kinematics and swerve odometry math.

**Measurement:**
1. Drop plumb lines from the FL and FR axle centers. Measure between marks on the floor → `kTrackWidthMeters`.
2. Drop plumb from FL and BL axle centers. Measure → `kWheelBaseMeters`.
3. If the robot is not perfectly square, use the average of both diagonals.

> **High accuracy matters here.** A 5 mm error in track width causes small circles when driving straight and odometry that curves over long distances.

---

## Section 19 — CANcoder Offsets

```java
kFrontLeftEncoderOffset  = 0.0;  // rotations — calibrate!
kFrontRightEncoderOffset = 0.0;
kBackLeftEncoderOffset   = 0.0;
kBackRightEncoderOffset  = 0.0;
```

**What:** Zeroes each CANcoder so that 0 = wheel pointing straight forward. Getting this wrong causes crab-walking (wheels at wrong angles at power-on).

**Calibration procedure:**
1. Place robot on flat surface. Use a long straight edge along the inside of all four wheels to ensure they all face forward simultaneously.
2. Open **Phoenix Tuner X** → select each CANcoder.
3. Read the **"Absolute Position"** signal (NOT "Position" — absolute is the magnet's true reading, not cumulative).
4. Enter that value (with sign) as the offset for that module.
5. Deploy, enable, command 0° heading on all modules. Verify all wheels are parallel and pointing forward.
6. Joystick forward → robot drives straight.

> **Sign convention:** Phoenix 6 `MagnetSensor.MagnetOffset` — positive offset rotates the zero CCW when viewed from the top of the encoder.

> Re-calibrate after any wheel swap, CANcoder replacement, or module rebuild.

---

## SysId Characterization Workflow

**SysId** (WPILib System Identification tool) measures your motor's true `kS` (static friction), `kV` (velocity constant), and `kA` (acceleration constant) empirically. These values produce better feedforward than guessing and reduce how much you need from PID.

### Which mechanisms to characterize
- **Drive motors** (each wheel independently or as a group) — improves trajectory following
- **Flywheel** — improves shot consistency, reduces spin-up time error

### Procedure

1. **In code:** `SysIdCommands.java` already contains the sysid routines for drive and flywheel. Bind them to controller buttons temporarily (or run via SmartDashboard commands).
2. **Deploy** to robot on a flat, open area (need ~4–5 m of free space for drive sysid).
3. **Open** WPILib SysId tool on your laptop.
4. **Connect** to the robot NetworkTables (set team number in SysId).
5. **Run** the four tests in order: Quasistatic Forward, Quasistatic Reverse, Dynamic Forward, Dynamic Reverse. Allow each to complete fully.
6. **Load** the resulting log file into SysId. Read off `kS`, `kV`, `kA`.
7. **Enter** values into `RobotConfig.java`.

### Tips
- Drive SysId: run with the robot on the actual field carpet — carpet friction differs from other surfaces.
- Flywheel SysId: run unloaded (no ball in system) to measure pure motor response.
- After entering SysId values, reduce `kP` significantly — feedforward handles most of the work now.
- If SysId gives unrealistic values (kV < 0 or very high kA), check that your gear ratio is correct in the config.

---

## Turret Spring Feedforward

The turret uses a **Vulcan spring** for cable management. This spring follows **Hooke's Law** — the restoring force increases proportionally with angle from center:

```
F_spring = k_spring × θ
```

As the turret rotates further from 0°, the spring pulls harder back toward center. Without compensation the turret drifts when holding off-center positions. With `kTurretI = 0`, the PID cannot correct this steady-state bias — the spring feedforward is the solution.

### How the Compensation Works

In `Shooter.java` → `setTurretAngle()`:
```java
double springFF = kTurretSpringFeedForwardVPerDeg * cachedTurretPositionDeg;
turretMotor.setControl(turretRequest
    .withPosition(targetTurretAngleDeg / 360.0)
    .withFeedForward(springFF));
```

| Turret angle | springFF | Effect |
|---|---|---|
| 0° | 0 V | Spring at rest, no force |
| +90° | +k×90 | Spring pulls back hard → motor adds forward voltage |
| −90° | −k×90 | Opposite direction |

This exactly mirrors Hooke's Law — feedforward grows linearly with displacement, opposing the spring at every angle.

### Constant

```java
kTurretSpringFeedForwardVPerDeg = 0.0;  // start here, tune up
```

Located in `RobotConfig.java`. Units: **volts per degree** of turret rotation from center.

### Tuning Procedure

1. Deploy with `kTurretSpringFeedForwardVPerDeg = 0.0`.
2. Enable Hub tracking. Command the turret to **+90°** and hold.
3. Open AdvantageScope. Watch `Shooter/TurretActualDeg` vs `Shooter/TurretTargetDeg` and `Shooter/TurretSpringFF`.
4. If turret slowly drifts back toward 0° → spring is winning → **increase by 0.005**.
5. If turret overshoots or oscillates → feedforward too high → **decrease by 0.003**.
6. Repeat at **−90°** to confirm symmetry.
7. Spot-check at **±45°** and **±135°** — the linear feedforward should work at all angles.

> **Telemetry:** `Shooter/TurretSpringFF` is logged every cycle — watch it in AdvantageScope to confirm it scales correctly with angle.

---

## Pre-Match Checklist

Run these in order before every match. The **System Check** command on the dashboard automates steps 5–10.

**Before powering on:**
- [ ] Turret is physically centered (at the index/detent mark facing forward)
- [ ] Hood is physically at its minimum (most open) hard stop
- [ ] All bumper numbers visible from 3 sides
- [ ] Battery voltage > 12.5 V
- [ ] Battery connector fully seated and locked

**After powering on:**
- [ ] All CAN devices appear healthy in DriverStation (green CAN utilization)
- [ ] No red fault LEDs on robot
- [ ] Homing warning messages appear in Driver Station (expected at startup)
- [ ] AdvantageScope connected — all subsystem logs appearing

**System Check (run from dashboard):**
- [ ] SystemCheck/Swerve — PASS
- [ ] SystemCheck/Intake Deploy — PASS
- [ ] SystemCheck/Spindexer — PASS (note baseline current)
- [ ] SystemCheck/Feeder — PASS (note baseline current)
- [ ] SystemCheck/Shooter — PASS
- [ ] SystemCheck/Vision Cameras — count ≥ 1 (ideally ≥ 3)
- [ ] SystemCheck/ALL PASSED — true

**Field verification:**
- [ ] Joystick forward → robot drives straight
- [ ] Heading lock holds heading after releasing stick
- [ ] Turret tracks toward HUB when right trigger held
- [ ] Hood reaches commanded angle (watch `Shooter/HoodAtTarget`)
- [ ] Flywheel reaches target RPS (watch `Shooter/FlywheelAtSpeed`)
- [ ] Vision pose estimate matches actual position on field
- [ ] Intake deploys/stows without binding
- [ ] Ball intake and feed cycle works

---

## Competition Day Procedures

### Between Matches

1. Check battery voltage — swap if below 12.2 V.
2. Check for loose CAN connectors (vibration during matches loosens them).
3. If wheels feel "wrong" → verify CANcoder offsets (wheels can get bumped).
4. Check if any breakers tripped — reset and investigate cause.
5. Watch AdvantageScope log from the previous match for any fault or anomaly.

### If a Subsystem Faults Mid-Match

| Fault | Operator action |
|-------|----------------|
| Shooter fault | Continue — robot can score with reduced accuracy |
| Turret fault | Use alliance zone dump (right bumper) as fallback |
| Feeder jam | Hold B button (unjam) for 1–2 seconds |
| Intake jam | Release left trigger, hold B button (unjam) |
| Swerve module fault | Drive with 3 modules — slower but functional |
| Vision lost | Odometry continues from wheel encoders — auto may be less accurate |

### Emergency Stop
- **Operator START** → emergency stop all mechanisms (robot still drives).
- **Operator BACK** → re-enable mechanisms.
- Use START if a mechanism is behaving dangerously (stuck actuator, burning smell).

### After Competition

1. Review all match logs in AdvantageScope — look for loop overruns, CAN faults, voltage brownouts.
2. Document any issues in the team's maintenance log.
3. If CANcoder offsets drifted (robot drives crooked) → recalibrate before next event.

---

## Common Failure Modes & Diagnostics

### Robot drives in circles / crab-walks at startup
**Cause:** CANcoder offset wrong for one or more modules.
**Fix:** Recalibrate CANcoder offsets (Section 19). Check which module's wheel is angled wrong, fix that offset first.

### Turret drifts back toward center when holding a position
**Cause:** Vulcan spring restoring force overcomes P gain with no integral.
**Fix:** Increase `kTurretSpringFeedForwardVPerDeg`. See [Turret Spring Feedforward](#turret-spring-feedforward).

### Flywheel never reaches "at speed" (ReadyToShoot never true)
**Cause:** `kFlywheelToleranceRPS` too tight, or `kFlywheelV` too low causing persistent error.
**Fix:** Increase `kFlywheelV` (or run SysId), then verify `kFlywheelToleranceRPS ≥ 2.0`.

### Balls jam entering the flywheel
**Cause:** Feeder too fast relative to flywheel, or pre-feed reverse not sufficient.
**Fix:** Reduce `kFeederSpeedRatio`, increase `kPreFeedReverseDurationSeconds`.

### Hood sags at high angles
**Cause:** `kHoodG` gravity feedforward too low.
**Fix:** Increase `kHoodG` until the hood holds all positions without drifting down.

### Vision pose jumps erratically
**Cause:** Single-tag ambiguity (perspective flip), or camera pose measurement wrong.
**Fix:** Increase `kSingleTagStdDevs`. Verify camera Yaw in RobotConfig matches physical mounting.

### CAN bus errors / timeouts (red in Driver Station)
**Cause:** Loose CAN connector, damaged cable, or too many devices on one bus.
**Fix:** Check all CAN connectors for seating. Verify termination resistors (120Ω) are present at both ends of the bus.

### Loop overruns (logged in RobotState)
**Cause:** Periodic method taking > 20 ms. Often caused by blocking calls or too many status signal refreshes.
**Fix:** Ensure `BaseStatusSignal.refreshAll()` is called once per periodic (already done). Check for any `Thread.sleep()` or blocking IO in periodic methods.

### Battery brownout during shoot-while-driving
**Cause:** Simultaneous high-current draw from drive + flywheel + intake.
**Fix:** `RobotState` automatically reduces flywheel to 70% on brownout prediction. If brownouts persist, lower `kDriveStatorCurrentLimit` and `kFlywheelP`.

---

## AdvantageScope Tuning Reference

AdvantageScope is the primary tool for understanding robot behavior. Use these log keys for tuning each subsystem:

### Swerve

| Key | Use for |
|-----|---------|
| `Swerve/OdometryPose` | Compare to planned path in auto |
| `Swerve/ModuleStates` | Verify each module's angle and speed |
| `Swerve/SlipDetected` | Tune stator current limit if true |
| `Swerve/HeadingError` | Tune heading lock PID |
| `Swerve/RawDriverX/Y` | Check for controller deadband issues |

### Shooter

| Key | Use for |
|-----|---------|
| `Shooter/FlywheelTargetRPS` vs `Shooter/FlywheelActualRPS` | Tune flywheel PID/FF |
| `Shooter/HoodTargetDeg` vs `Shooter/HoodActualDeg` | Tune hood PID/G |
| `Shooter/TurretTargetDeg` vs `Shooter/TurretActualDeg` | Tune turret PID |
| `Shooter/TurretSpringFF` | Monitor spring feedforward magnitude |
| `Shooter/ReadyToShoot` | Verify all conditions met before feeding |
| `Shooter/DistanceToHub` | Verify distance calculation |
| `Shooter/InRange` | Check shooting range bounds |

### Intake / Feeder / Spindexer

| Key | Use for |
|-----|---------|
| `Intake/LeftDeploySupplyCurrent` | Tune gravity FF table |
| `Intake/DeployPositionDeg` | Verify deploy travel and endpoint |
| `Spindexer/SupplyCurrent` | Tune ball detection thresholds |
| `Feeder/SupplyCurrent` | Tune ball detection thresholds |
| `Feeder/BeamBreakTripped` | Verify beam-break wiring |

### Vision

| Key | Use for |
|-----|---------|
| `Vision/EstimatedPoseField` | Compare to odometry pose |
| `Vision/TagsVisible` | Count of visible AprilTags |
| `Vision/CamerasConnected` | Camera connectivity |

### System Health

| Key | Use for |
|-----|---------|
| `RobotState/BatteryVoltage` | Monitor brownout risk |
| `RobotState/LoopTime` | Detect loop overruns (should be ≤ 20 ms) |
| `RobotState/CANUtilization` | CAN bus health (should be < 80%) |
| `Shooter/TurretFaultCycles` | Detect turret jam early |
| `Shooter/FlywheelFaultCycles` | Detect flywheel motor fault |

---

## Quick Reference — All Fields in RobotConfig.java

| Field | Section | Default | Unit |
|-------|---------|---------|------|
| `kDriverControllerPort` | 1 | 0 | USB port |
| `kOperatorControllerPort` | 1 | 1 | USB port |
| `kDeadband` | 1 | 0.1 | — |
| `kLedPort` | 2 | 0 | PWM port |
| `kLedCount` | 2 | 60 | count |
| `kPigeonId` | 3 | 13 | CAN ID |
| `kFrontLeftDriveMotorId` | 3 | 1 | CAN ID |
| `kFrontLeftSteerMotorId` | 3 | 2 | CAN ID |
| `kFrontLeftEncoderId` | 3 | 3 | CAN ID |
| `kFrontRightDriveMotorId` | 3 | 4 | CAN ID |
| `kFrontRightSteerMotorId` | 3 | 5 | CAN ID |
| `kFrontRightEncoderId` | 3 | 6 | CAN ID |
| `kBackLeftDriveMotorId` | 3 | 7 | CAN ID |
| `kBackLeftSteerMotorId` | 3 | 8 | CAN ID |
| `kBackLeftEncoderId` | 3 | 9 | CAN ID |
| `kBackRightDriveMotorId` | 3 | 10 | CAN ID |
| `kBackRightSteerMotorId` | 3 | 11 | CAN ID |
| `kBackRightEncoderId` | 3 | 12 | CAN ID |
| `kDriveGearRatio` | 4 | 6.75 | — |
| `kSteerGearRatio` | 4 | 150/7 | — |
| `kWheelDiameterMeters` | 4 | 0.1016 | m |
| `kDriveP/I/D/S/V/A` | 5 | 0.1/0/0/0/0.12/0 | — |
| `kDriveCurrentLimit` | 5 | 60.0 | A |
| `kDriveStatorCurrentLimit` | 5 | 80.0 | A |
| `kSteerCurrentLimit` | 5 | 30.0 | A |
| `kSteerP/I/D` | 5 | 100/0/0.5 | — |
| `kAutoTranslationP/I/D` | 5 | 10/0/0 | — |
| `kAutoRotationP/I/D` | 5 | 7.5/0/0 | — |
| `kHeadingLockP/I/D` | 5 | 5/0/0.3 | — |
| `kMaxAngularAccelRadiansPerSecondSq` | 5 | 8.0 | rad/s² |
| `kSlowModeSpeedMultiplier` | 5 | 0.35 | — |
| `kSlowModeRotMultiplier` | 5 | 0.40 | — |
| `kTranslationSlewRate` | 5 | 6.0 | m/s² |
| `kRotationSlewRate` | 5 | 8.0 | rad/s² |
| `kFlywheelMotorId` | 6 | 20 | CAN ID |
| `kHoodMotorId` | 6 | 21 | CAN ID |
| `kTurretMotorId` | 6 | 22 | CAN ID |
| `kFlywheelGearRatio` | 7 | 2.0 | — |
| `kTopToBottomGearRatio` | 7 | 2.0 | — |
| `kBottomFlywheelDiameterMeters` | 7 | 0.1016 | m |
| `kTopFlywheelDiameterMeters` | 7 | 0.0508 | m |
| `kHoodGearRatio` | 7 | 2.0 | — |
| `kTurretHeightMeters` | 7 | 0.45 | m |
| `kHoodHeightMeters` | 7 | 0.50 | m |
| `kShooterPositionOffset` | 8 | (0,0) | m |
| `kShooterExitHeightMeters` | 8 | 0.610 | m |
| `kHoodMinMotorRotations` | 9 | 0.0 ⚠️ | rot |
| `kHoodMaxMotorRotations` | 9 | 0.0 ⚠️ | rot |
| `kTurretGearRatio` | 10 | 100.0 ⚠️ | — |
| `kTurretCCWLimitMotorRotations` | 10 | 0.0 ⚠️ | rot |
| `kTurretCWLimitMotorRotations` | 10 | 0.0 ⚠️ | rot |
| `kTurretMountOffsetDegrees` | 10 | 0.0 | deg |
| `kShooterEfficiencyFactor` | 11 | 0.30 | — |
| `kBiasFactorNear` | 11 | 1.00 | — |
| `kBiasFactorFar` | 11 | 1.00 | — |
| `kFlywheelP/I/D/S/V/A` | 11 | 0.5/0/0/0/0.12/0 | — |
| `kFlywheelToleranceRPS` | 11 | 2.0 | rps |
| `kIdleFlywheelRPS` | 11 | 15.0 | rps |
| `kMaxFlywheelRPS` | 11 | 90.0 | rps |
| `kHoodP/I/D/G` | 11 | 8/0/0.2/0.15 | — |
| `kTurretP/I/D` | 11 | 30/0/0.5 | — |
| `kTurretSpringFeedForwardVPerDeg` | 11 | 0.0 | V/deg |
| `kAllianceZoneFlywheelRPS` | 11 | 35.0 | rps |
| `kAllianceZoneHoodAngleDeg` | 11 | 40.0 | deg |
| `kAllianceZoneTurretAngleDeg` | 11 | 172.0 | deg |
| `kAllianceZoneFeederSpeed` | 11 | 0.60 | duty |
| `kPreFeedReverseDurationSeconds` | 11 | 0.12 | s |
| `kPreFeedReverseFeederSpeed` | 11 | −0.25 | duty |
| `kPreFeedReverseSpindexerSpeed` | 11 | −0.15 | duty |
| `kFeederMotorId` | 12 | 23 | CAN ID |
| `kBeamBreakDIOPort` | 12 | 0 ⚠️ | DIO port |
| `kSpindexerMotorId` | 12 | 27 | CAN ID |
| `kFeederSpeedRatio` | 12 | 0.75 | — |
| `kSpindexerIntakeSpeed` | 12 | 0.4 | duty |
| `kSpindexerToFeederRatio` | 12 | 0.65 | — |
| `kFeederCurrentLimit` | 12 | 30.0 | A |
| `kSpindexerCurrentLimit` | 12 | 30.0 | A |
| `kLeftDeployMotorId` | 13 | 24 | CAN ID |
| `kRightDeployMotorId` | 13 | 25 | CAN ID |
| `kRollerMotorId` | 13 | 26 | CAN ID |
| `kDeployGearRatio` | 14 | 4.0 | — |
| `kIntakeExtendedMotorRotations` | 14 | −0.01293 | rot |
| `kRollerDiameterMeters` | 14 | 0.0508 | m |
| `kRollerGearRatio` | 14 | 5.0 | — |
| `kDeployP/I/D/S` | 14 | 40/0/1/0.12 | — |
| `kDeployMaxVelocity` | 14 | 4.0 | rot/s |
| `kDeployMaxAcceleration` | 14 | 8.0 | rot/s² |
| `kDeployCurrentLimit` | 14 | 30.0 | A |
| `kRollerCurrentLimit` | 14 | 40.0 | A |
| `kRollerIntakeSpeed` | 14 | 0.8 | duty |
| `kRollerOuttakeSpeed` | 14 | −0.6 | duty |
| `kDeployLeftGravityVoltages[5]` | 14 | {0, 0.10, 0.25, 0.35, 0.40} | V |
| `kDeployRightGravityVoltages[5]` | 14 | {0, 0.15, 0.35, 0.50, 0.60} | V |
| `kFrontLeftCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kFrontRightCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kBackLeftCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kBackRightCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kIntakeCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kSideCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders ⚠️ | m / rad |
| `kMultiTagStdDevs[3]` | 16 | {0.5, 0.5, 1.0} | m / rad |
| `kSingleTagStdDevs[3]` | 16 | {4.0, 4.0, 8.0} | m / rad |
| `kAggressiveMultiTagStdDevs[3]` | 16 | {0.1, 0.1, 0.2} | m / rad |
| `kBallChaseYawP/I/D` | 16 | 0.04/0/0.002 | — |
| `kDefaultSpindexerBaselineCurrent` | 17 | 3.0 | A |
| `kDefaultFeederBaselineCurrent` | 17 | 2.5 | A |
| `kSpindexerLoadedThreshold` | 17 | 2.0 | A |
| `kSpindexerEmptyThreshold` | 17 | 0.5 | A |
| `kSpindexerFullLoadDelta` | 17 | 8.0 | A |
| `kFeederBallPresentThreshold` | 17 | 3.0 | A |
| `kTrackWidthMeters` | 18 | 0.5779 | m |
| `kWheelBaseMeters` | 18 | 0.5779 | m |
| `kFrontLeftEncoderOffset` | 19 | 0.0 ⚠️ | rot |
| `kFrontRightEncoderOffset` | 19 | 0.0 ⚠️ | rot |
| `kBackLeftEncoderOffset` | 19 | 0.0 ⚠️ | rot |
| `kBackRightEncoderOffset` | 19 | 0.0 ⚠️ | rot |

⚠️ = placeholder value, **must be calibrated** before competition use.
