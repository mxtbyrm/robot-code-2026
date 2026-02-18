# Robot Configuration Guideline — FRC Team 7840 (2026 "REBUILT")

**The only file you ever edit to deploy to a new robot is:**
```
src/main/java/frc/robot/RobotConfig.java
```
Everything else (subsystems, commands, constants) reads from `RobotConfig` automatically.
This guide walks through every section in order, explains what each value means, and tells you exactly how to measure or calibrate it.

---

## Coordinate System Reference

All robot-frame measurements use the **WPILib standard**:

| Axis | Direction | Unit |
|------|-----------|------|
| X | Forward (toward intake / shooter front) | meters |
| Y | Left | meters |
| Z | Up | meters |
| Yaw | CCW positive, 0 = facing forward | radians or degrees |
| Pitch | Negative = tilted upward (WPILib convention) | radians or degrees |
| Roll | CCW positive around X axis | radians or degrees |

The **robot center** is the point on the floor equidistant from all four swerve module wheel contact patches.

---

## Step-by-Step Configuration Order

Follow this order. Steps marked **[HARDWARE]** require the physical robot. Steps marked **[SOFTWARE]** can be done at a desk.

```
 1. [SOFTWARE] CAN IDs & ports
 2. [HARDWARE] Swerve geometry (track width, wheelbase)
 3. [HARDWARE] CANcoder offsets
 4. [HARDWARE] Swerve module hardware (gear ratio, wheel diameter)
 5. [HARDWARE] Swerve current limits
 6. [HARDWARE] Drive + steer PID / FF
 7. [HARDWARE] Intake deploy calibration (travel endpoint)
 8. [HARDWARE] Intake current limits & roller speeds
 9. [HARDWARE] Hood calibration
10. [HARDWARE] Turret calibration
11. [HARDWARE] Shooter geometry
12. [HARDWARE] Camera poses (position + full rotation, each camera)
13. [HARDWARE] Ball detection baselines
14. [HARDWARE] Flywheel PID / FF
15. [HARDWARE] Shooter physics tuning (efficiency, bias)
16. [HARDWARE] Alliance zone lob tuning
17. [HARDWARE] Pre-feed reverse tuning
18. [HARDWARE] Intake gravity FF table
19. [SOFTWARE] Remaining driver / LED preferences
```

---

## Section 1 — Operator Controller Ports

```java
kDriverControllerPort   = 0;
kOperatorControllerPort = 1;
kDeadband               = 0.1;
```

**What:** USB port numbers assigned by the Driver Station to each Xbox controller.

**How:** Plug in both controllers. In the FRC Driver Station app, the USB tab shows each joystick with its assigned port number. Set driver and operator ports to match.

**Deadband:** 0.1 (10%) is a good starting point. Increase if the robot drifts with the stick at rest; decrease if the response feels sluggish at low inputs.

---

## Section 2 — LED Hardware

```java
kLedPort  = 0;   // RoboRIO PWM port
kLedCount = 60;  // total LEDs on the strip
```

**What:** The RoboRIO PWM port that the LED strip's data line connects to, and the total LED count.

**How:** Check the wiring diagram for the PWM port. Count the LEDs or read the datasheet. Getting `kLedCount` wrong causes patterns to wrap incorrectly or cut off mid-strip.

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
1. Open Phoenix Tuner X and connect via USB or Wi-Fi.
2. Under "Devices," all CAN devices appear with their current IDs.
3. Match each physical module (FL, FR, BL, BR) to its drive motor, steer motor, and CANcoder, then enter here.

> Convention used: FL = 1,2,3 / FR = 4,5,6 / BL = 7,8,9 / BR = 10,11,12 / Pigeon = 13. Change only if wired differently.

---

## Section 4 — Swerve Module Hardware

```java
kDriveGearRatio      = 6.75;         // MK4i L2
kSteerGearRatio      = 150.0 / 7.0;  // ~21.43:1, all MK4i variants
kWheelDiameterMeters = Units.inchesToMeters(4.0);
```

**Drive gear ratio:** Check which MK4i level you have. L1 = 8.14, L2 = 6.75, L3 = 6.12, L4 = 5.14.

**Steer gear ratio:** 150/7 ≈ 21.43 for ALL MK4i variants — do not change unless you have a non-standard module.

**Wheel diameter:** Measure the actual wheel with calipers after some wear. A worn wheel is slightly smaller than the nominal 4 inches. Even a 2 mm error compounds into noticeable odometry drift over a full field traversal.

---

## Section 5 — Swerve PID / FF / Driver Params

### Drive Motor PID + FF (velocity control)

```java
kDriveP = 0.1;   kDriveI = 0.0;   kDriveD = 0.0;
kDriveS = 0.0;   kDriveV = 0.12;  kDriveA = 0.0;
```

**Recommended tuning order:**
1. **`kDriveV` first** — set to `1 / freeSpeedRPS`. Kraken free speed ≈ 100 RPS → `kDriveV ≈ 0.01`. Increase until the wheel tracks velocity setpoints without needing P correction.
2. **`kDriveS`** — slowly apply a constant voltage until the wheel just barely starts moving. That voltage is `kDriveS`.
3. **`kDriveP`** — increase from 0 until velocity error is corrected quickly without oscillation.
4. Leave `kDriveI` and `kDriveA` at 0.0 unless you have persistent steady-state error.

### Swerve Current Limits

```java
kDriveCurrentLimit        = 60.0;  // supply amps
kDriveStatorCurrentLimit  = 80.0;  // stator amps (prevents wheel slip)
kSteerCurrentLimit        = 30.0;  // supply amps
```

**What:** Motor protection thresholds. Supply limits protect wiring and breakers; stator limits prevent wheel slip on carpet under hard acceleration.

**Tuning:**
- If drive motors trip breakers or overheat, lower `kDriveCurrentLimit`.
- If wheels spin out under hard acceleration (visible on AdvantageScope slip detection), lower `kDriveStatorCurrentLimit`.
- Steer motors draw very little current — 30 A is generous; lower only if steer motors run hot.

### Steer Motor PID

```java
kSteerP = 100.0;   kSteerI = 0.0;   kSteerD = 0.5;
```

**Tuning:** MK4i steer has low inertia — high P is normal. If the wheel oscillates around the target angle, increase `kSteerD`. If it undershoots, increase `kSteerP`.

### Auto Trajectory PID

```java
kAutoTranslationP = 10.0;   kAutoTranslationI = 0.0;   kAutoTranslationD = 0.0;
kAutoRotationP    = 7.5;    kAutoRotationI    = 0.0;   kAutoRotationD    = 0.0;
```

**What:** Correction gains for Choreo/PathPlanner to fix positional error during autonomous.

**Tuning:** Run a straight-line path. If the robot overshoots the endpoint, lower Translation P. If it drifts or arrives late, increase it. Rotation P controls heading correction during path following.

### Heading Lock PID

```java
kHeadingLockP = 5.0;   kHeadingLockI = 0.0;   kHeadingLockD = 0.3;
kMaxAngularAccelRadiansPerSecondSq = 8.0;
```

**What:** Holds the robot's heading when the driver releases the rotation stick.

**Tuning:** Increase P until heading snaps back quickly. Add D if it overshoots. Lower `kMaxAngularAccelRadiansPerSecondSq` if the heading correction feels jerky when releasing the stick.

### Slow Mode & Slew Rate

```java
kSlowModeSpeedMultiplier = 0.35;
kSlowModeRotMultiplier   = 0.40;
kTranslationSlewRate     = 6.0;   // m/s²
kRotationSlewRate        = 8.0;   // rad/s²
```

**Slow mode:** Applied when the driver holds a trigger. Adjust to taste for precision driving near scoring positions.

**Slew rate:** Limits acceleration rate. Higher = more responsive but more wheel spin on carpet. Lower = smoother but feels sluggish. Watch for wheel spin during hard directional changes.

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
kFlywheelGearRatio            = 2.0;
kTopToBottomGearRatio         = 2.0;
kBottomFlywheelDiameterMeters = Units.inchesToMeters(4.0);
kTopFlywheelDiameterMeters    = Units.inchesToMeters(2.0);
kHoodGearRatio                = 2.0;
kTurretHeightMeters           = 0.45;
kHoodHeightMeters             = 0.50;
```

**Flywheel diameters:** Measure with calipers. The physics model uses surface speed to compute exit velocity — even a few mm of error shifts the entire shot table.

**Gear ratios:** Count teeth or verify from CAD. `kFlywheelGearRatio` = motor rotations per bottom flywheel rotation. `kTopToBottomGearRatio` = how many times faster the top flywheel spins (sized so both have equal surface speed).

**Turret / hood heights:** Measure from the floor (carpet) to the turret rotation axis and to the hood pivot. Used in projectile motion calculations.

---

## Section 8 — Shooter Geometry

```java
kShooterPositionOffset   = new Translation2d(0.0, 0.0);
kShooterExitHeightMeters = Units.inchesToMeters(24.0);
```

**Shooter position offset:**
1. Place robot on a flat surface. Drop a plumb line to mark the robot center on the floor.
2. Measure horizontal distance from the center mark to directly below the ball exit gap.
3. X = forward component (positive toward front). Y = lateral component (positive left).
4. Example: 10 cm forward, 2 cm to the right → `Translation2d(0.10, -0.02)`.

**Exit height:** Hold a ruler vertically beside the shooter. Measure from floor to the center of the flywheel gap. Convert: `Units.inchesToMeters(yourInches)`.

---

## Section 9 — Hood Calibration

```java
kHoodMinMotorRotations = 0.0;  // TODO — motor rotations at fully open (minimum) hard stop
kHoodMaxMotorRotations = 0.0;  // TODO — motor rotations at fully closed (maximum) hard stop
```

**What:** Kraken encoder readings at the two physical hood hard stops. Used to map motor position → hood angle.

**Procedure:**
1. Power on robot. Open Phoenix Tuner X → select the hood motor (ID = `kHoodMotorId`).
2. Move the hood to its **minimum** (fully open) hard stop. Read the "Position" signal (motor rotations). Enter as `kHoodMinMotorRotations`.
3. Move the hood to its **maximum** (fully closed) hard stop. Read the position. Enter as `kHoodMaxMotorRotations`.

> Verify: commanding `kHoodMinAngleDegrees` should move the hood to the open hard stop; `kHoodMaxAngleDegrees` to the closed hard stop.

---

## Section 10 — Turret Calibration

```java
kTurretGearRatio              = 100.0;  // TODO — measure
kTurretCCWLimitMotorRotations = 0.0;    // TODO — calibrate
kTurretCWLimitMotorRotations  = 0.0;    // TODO — calibrate
kTurretMountOffsetDegrees     = 0.0;
```

### Gear Ratio

1. In Phoenix Tuner X, zero the turret motor encoder at a known position. Mark a reference line on the turret and the frame.
2. Rotate the turret exactly one full mechanical revolution (mark returns to start).
3. Read the motor position — that number is `kTurretGearRatio`.

### Hard Stop Limits

1. Power on with turret centered (encoder = 0, facing forward).
2. Slowly rotate **CCW** until the mechanical hard stop. Read position in Tuner X → `kTurretCCWLimitMotorRotations` (positive).
3. Return to center. Rotate **CW** to the other stop. Read position → `kTurretCWLimitMotorRotations` (negative).

### Mount Offset

If the encoder cannot be zeroed so that 0 = facing forward, measure the angular misalignment with a digital angle gauge and enter as `kTurretMountOffsetDegrees`. CCW from robot forward is positive. Leave at 0.0 if well-aligned.

---

## Section 11 — Shooter Tuning

### Efficiency Factor

```java
kShooterEfficiencyFactor = 0.30;
```

**What:** Ratio of actual ball exit speed to flywheel surface speed. 0.30 means 30% of surface speed transfers to ball velocity (the rest is lost to compression, slip, and friction).

**Tuning:**
1. Start at 0.30.
2. Shoot at a known distance (e.g., 3 m from the HUB center).
3. Consistently short → increase. Consistently long → decrease.
4. Dial this in before touching bias factors.

### Bias Factors

```java
kBiasFactorNear = 1.00;  // at minimum shooting distance (~1.2 m)
kBiasFactorFar  = 1.00;  // at maximum shooting distance (~6.5 m)
```

**What:** Corrects for air drag and ball spin that the vacuum physics model ignores. Linearly interpolated between near and far distances.

**Tuning:**
1. After `kShooterEfficiencyFactor` is dialled in, shoot at the minimum distance. Still short → increase `kBiasFactorNear`.
2. Shoot at maximum distance. Short at range but fine up close → increase `kBiasFactorFar`.
3. Mid-range shots interpolate automatically.

### Flywheel PID + FF

```java
kFlywheelP = 0.5;   kFlywheelI = 0.0;   kFlywheelD = 0.0;
kFlywheelS = 0.0;   kFlywheelV = 0.12;  kFlywheelA = 0.0;
kFlywheelToleranceRPS = 2.0;
kIdleFlywheelRPS      = 15.0;
kMaxFlywheelRPS       = 90.0;
```

Same approach as drive motor: tune `kFlywheelV` to match free speed, `kFlywheelS` for static friction, then `kFlywheelP` for tracking.

- **`kFlywheelToleranceRPS`:** ±RPS window before the robot considers "at speed." Tighten for better consistency; loosen to reduce wait time.
- **`kIdleFlywheelRPS`:** Pre-spin speed between shots. Higher = faster spin-up, more battery draw.
- **`kMaxFlywheelRPS`:** Normalization ceiling for feeder/spindexer duty cycles. Must be ≥ the highest RPS value in the shooting table.

### Hood PID + Gravity FF

```java
kHoodP = 8.0;   kHoodI = 0.0;   kHoodD = 0.2;   kHoodG = 0.15;
```

**`kHoodG`** is a constant voltage to counteract hood sag under gravity. Tune by: command a mid-range angle, disable P temporarily, and increase `kHoodG` until the hood holds position without drifting down.

### Turret PID

```java
kTurretP = 30.0;   kTurretI = 0.0;   kTurretD = 0.5;
```

High P is needed for fast tracking. If the turret oscillates, increase D. If it lags on fast target motion, increase P.

### Alliance Zone Dump

```java
kAllianceZoneFlywheelRPS    = 35.0;
kAllianceZoneHoodAngleDeg   = 40.0;
kAllianceZoneTurretAngleDeg = 172.0;
kAllianceZoneFeederSpeed    = 0.60;
```

**What:** Fixed flywheel speed, hood angle, turret angle, and feeder duty for the alliance zone lob shot (turret faces backward, no auto-aim). The right values are entirely robot-geometry-dependent and must be tuned on the real robot.

**Tuning procedure:**
1. Position robot in the neutral zone at the location it would shoot from.
2. Enable the alliance zone dump mode.
3. Adjust `kAllianceZoneHoodAngleDeg` (steeper arc = higher angle) and `kAllianceZoneFlywheelRPS` (more power = longer throw) until balls land in the alliance zone.
4. `kAllianceZoneTurretAngleDeg = 172°` faces nearly straight backward; adjust only if the alliance zone target is off-center.

### Pre-Feed Reverse

```java
kPreFeedReverseDurationSeconds = 0.12;
kPreFeedReverseFeederSpeed     = -0.25;
kPreFeedReverseSpindexerSpeed  = -0.15;
```

**What:** A brief backward pulse before feeding that creates a small gap between the next ball and the flywheel. Prevents jams when the flywheel is spinning and a ball is sitting directly against it.

**Tuning:**
- If balls still jam entering the flywheel, increase `kPreFeedReverseDurationSeconds` (e.g., 0.15–0.20 s).
- If the reverse pulse causes the ball to back out too far and re-jam, reduce the duration or speeds.
- Speeds should remain slow and gentle — this is not an unjam, just a clearance pulse.

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

**CAN IDs:** Verify in Phoenix Tuner X.

**Beam break DIO port:** Check the RoboRIO DIO wiring. Valid ports are 0–9. Set `kBeamBreakDIOPort` to the actual port number.

**Speed hierarchy (critical — prevents jams):**
```
Flywheel surface speed  >>  Feeder  >  Spindexer  >  Intake roller
```
This chain is enforced automatically as long as `kFeederSpeedRatio < 1.0` and `kSpindexerToFeederRatio < 1.0`.

- If balls jam between spindexer and feeder → reduce `kSpindexerToFeederRatio`.
- If balls jam entering the flywheel → reduce `kFeederSpeedRatio`.

**Current limits:** Lower if motors run hot. The feeder and spindexer are lightly loaded — 30 A is conservative.

---

## Section 13 — Intake CAN Wiring

```java
kLeftDeployMotorId  = 24;
kRightDeployMotorId = 25;
kRollerMotorId      = 26;
```

Verify in Phoenix Tuner X. The right deploy motor is configured as mirror-inverted in software — no hardware change needed.

---

## Section 14 — Intake Hardware / Calibration

### Deploy Travel Calibration

```java
kDeployGearRatio              = 4.0;
kIntakeExtendedMotorRotations = -0.01293;  // TODO: re-measure after any rebuild
```

**`kIntakeExtendedMotorRotations`** is the raw Kraken encoder reading when the intake arm is at full extension (touching the floor / bumper hard stop).

**Procedure:**
1. Power on robot. In Phoenix Tuner X or via a test command, drive the left deploy motor until the arm is fully deployed.
2. Read the "Position" signal (motor rotations) in Tuner X.
3. Enter that value (negative if the motor extends in the negative direction).
4. The stowed position is assumed to be encoder = 0 (home at power-on).

> Re-measure any time the intake is rebuilt or the hard stop position changes.

### Deploy PID + Motion Magic

```java
kDeployP = 40.0;   kDeployI = 0.0;   kDeployD = 1.0;   kDeployS = 0.12;
kDeployMaxVelocity     = 4.0;   // rotations/sec
kDeployMaxAcceleration = 8.0;   // rotations/sec²
```

- **`kDeployS`:** Slowly increase until the arm just begins to move from rest.
- **`kDeployP`:** Increase until the arm reaches target position without oscillation.
- **`kDeployMaxVelocity / Acceleration`:** Reduce if the arm slams hard stops aggressively.

### Current Limits

```java
kDeployCurrentLimit = 30.0;  // amps
kRollerCurrentLimit = 40.0;  // amps
```

**Deploy:** Low mass arm — 30 A is plenty. Lower if deploy motors overheat.
**Roller:** Higher limit because the roller contacts game pieces. If the roller motor trips under ball load, slightly increase. If it runs hot during long intake sessions, decrease.

### Roller Speeds

```java
kRollerIntakeSpeed  =  0.8;  // duty cycle, collecting balls inward
kRollerOuttakeSpeed = -0.6;  // duty cycle, ejecting balls outward
```

**Tuning:**
- `kRollerIntakeSpeed`: Start at 0.8. If balls are pushed away instead of collected, increase (roller surface speed must exceed chassis speed). If the roller jams on contact, decrease slightly.
- `kRollerOuttakeSpeed`: Adjust so balls are ejected cleanly. More negative = more aggressive eject.

### Roller Dimensions

```java
kRollerDiameterMeters = Units.inchesToMeters(2.0);
kRollerGearRatio      = 5.0;
```

**What:** Used to compute roller surface speed. The robot caps chassis speed while intaking so the roller always sweeps faster than the floor relative to the robot (prevents pushing balls away).

**How:** Measure the roller outer diameter with calipers. Verify gear ratio from CAD.

### Deploy Gravity FF Voltage Tables

```java
kDeployLeftGravityVoltages  = { 0.0, 0.10, 0.25, 0.35, 0.40 };
kDeployRightGravityVoltages = { 0.0, 0.15, 0.35, 0.50, 0.60 };
```

**What:** Five voltage feedforward values corresponding to the arm at 0%, 25%, 50%, 75%, and 100% of deploy travel. Applied as additional motor output to counteract gravity — left and right are separate because the right side carries extra roller weight.

**Tuning procedure:**
1. Open AdvantageScope. Plot `Intake/LeftDeploySupplyCurrent` and `Intake/RightDeploySupplyCurrent`.
2. Slowly move the arm to 0% (stowed). Record idle current as your baseline.
3. Move to 25%, 50%, 75%, 100% in turn. At each position, increase the corresponding voltage entry until the current at that position matches the idle baseline.
4. With correct values, the arm should hold any position with near-zero P correction and flat current across all positions.

---

## Section 15 — Camera Hardware (Per-Camera 6-DOF Pose)

Each camera has its own independent 6-axis pose. Do not assume any two cameras share height, pitch, or position.

### Field definitions

| Field suffix | Unit | Description |
|---|---|---|
| `X` | meters | Forward offset from robot center (positive = forward) |
| `Y` | meters | Left offset from robot center (positive = left) |
| `Z` | meters | Height above floor (positive = up) |
| `Roll` | radians | Rotation about forward axis (0 = level) |
| `Pitch` | radians | **Negative = tilted upward** (WPILib convention). −15° tilt = `Units.degreesToRadians(-15.0)` |
| `Yaw` | radians | CCW from robot forward. 0° = forward, 90° = facing left, −90° = facing right |

### Cameras to configure

| Prefix | Purpose |
|--------|---------|
| `kFrontLeftCam` | AprilTag — front-left corner |
| `kFrontRightCam` | AprilTag — front-right corner |
| `kBackLeftCam` | AprilTag — back-left corner |
| `kBackRightCam` | AprilTag — back-right corner |
| `kIntakeCam` | Ball detection (ML pipeline, NOT AprilTag) |
| `kSideCam` | Lateral awareness |

### Measurement procedure

1. Place robot on a flat surface. Mark the robot center on the floor (equidistant from all four wheel contact patches).
2. **X, Y:** Measure the horizontal distance from the floor mark to directly below the camera lens. X = forward component, Y = lateral (left = positive).
3. **Z:** Measure from the floor to the camera optical center (the lens, not the bracket).
4. **Yaw:** With a digital angle gauge on the camera body, measure the angle from robot forward. CCW positive.
5. **Pitch:** Measure the tilt from horizontal. Enter as negative for upward tilt (camera looking up → negative pitch).
6. **Roll:** Measure rotation about the forward axis. Most cameras are mounted level (roll = 0).

> **Tip:** If the robot was designed in CAD (Onshape, SolidWorks, Fusion), export camera poses from the model — more accurate than tape-measure estimates.

### Example

```java
// Front-left AprilTag camera at robot corner, facing 45° outward, 15° upward tilt
kFrontLeftCamX     = Units.inchesToMeters(11.375);  // forward of center
kFrontLeftCamY     = Units.inchesToMeters(11.375);  // left of center
kFrontLeftCamZ     = Units.inchesToMeters(12.0);    // height above floor
kFrontLeftCamRoll  = 0.0;                           // level
kFrontLeftCamPitch = Units.degreesToRadians(-15.0); // 15° upward tilt
kFrontLeftCamYaw   = Units.degreesToRadians(45.0);  // facing 45° left of forward
```

---

## Section 16 — Vision Tuning

### Pose Estimation Standard Deviations

```java
kMultiTagStdDevs           = { 0.5, 0.5, 1.0 };  // x_m, y_m, heading_rad
kSingleTagStdDevs          = { 4.0, 4.0, 8.0 };
kAggressiveMultiTagStdDevs = { 0.1, 0.1, 0.2 };
```

**What:** These control how much the Kalman filter trusts vision versus wheel odometry. Lower = more trust in vision.

| Scenario | Guidance |
|----------|----------|
| Pose jumps erratically in auto | Increase `kMultiTagStdDevs` |
| Odometry drifts, vision doesn't correct | Decrease `kMultiTagStdDevs` |
| Single-tag causing wrong snap | Increase `kSingleTagStdDevs` |
| Post-collision pose recovery too slow | Decrease `kAggressiveMultiTagStdDevs` |

`kSingleTagStdDevs` should always be significantly larger than multi-tag — single-tag estimates have perspective ambiguity.

### Ball Chase PID

```java
kBallChaseYawP = 0.04;   kBallChaseYawI = 0.0;   kBallChaseYawD = 0.002;
```

**What:** Yaw correction PID for `VisionIntakeCommand` when centering on a ball with the intake camera.

**Tuning:** Increase P if the robot is slow to center. Add D if it oscillates left-right while chasing.

---

## Section 17 — Ball Detection Tuning

```java
kDefaultSpindexerBaselineCurrent = 3.0;  // amps, empty hopper
kDefaultFeederBaselineCurrent    = 2.5;  // amps, empty feeder
kSpindexerLoadedThreshold        = 2.0;  // amps above baseline → has balls
kSpindexerEmptyThreshold         = 0.5;  // amps above baseline → empty
kSpindexerFullLoadDelta          = 8.0;  // amps above baseline → fully full
kFeederBallPresentThreshold      = 3.0;  // amps above baseline → ball in feeder
```

**What:** Motor current rises when balls add friction. These thresholds detect ball presence by comparing current to an empty-mechanism baseline.

**Tuning procedure:**
1. Run with **empty hopper**. In AdvantageScope, plot `Spindexer/SupplyCurrent` and `Feeder/SupplyCurrent` for ~10 s.
2. Record steady-state average → `kDefaultSpindexerBaselineCurrent` and `kDefaultFeederBaselineCurrent`.
3. Load **one ball**. Record the current delta above baseline.
4. Set `kSpindexerLoadedThreshold` ≈ 80% of that delta (triggers reliably, ignores noise).
5. Set `kSpindexerEmptyThreshold` ≈ 20% of that delta (hysteresis to prevent flickering).
6. Load a **full hopper** (8 balls). Record spindexer delta → `kSpindexerFullLoadDelta`.
7. Shoot one ball through the feeder. Record feeder current spike → `kFeederBallPresentThreshold`.

---

## Section 18 — Swerve Geometry

```java
kTrackWidthMeters = Units.inchesToMeters(22.75);
kWheelBaseMeters  = Units.inchesToMeters(22.75);
```

**What:** Center-to-center distances between swerve modules. Used to build kinematics and position cameras.

**How to measure:**
1. Drop a plumb line from the center of each front-left and front-right axle. Measure between marks → `kTrackWidthMeters`.
2. Drop plumb from front-left and back-left axle centers. Measure → `kWheelBaseMeters`.
3. All four corners should be symmetric. If not, use the average.

---

## Section 19 — CANcoder Offsets

```java
kFrontLeftEncoderOffset  = 0.0;  // rotations — calibrate!
kFrontRightEncoderOffset = 0.0;
kBackLeftEncoderOffset   = 0.0;
kBackRightEncoderOffset  = 0.0;
```

**What:** Zeroes each CANcoder so the wheel points straight forward when the encoder reads 0. Getting this wrong causes crab-walking.

**Calibration procedure:**
1. Place robot on flat surface. Align all four wheels exactly straight forward (use a straight edge along the wheel face).
2. Open Phoenix Tuner X → select each CANcoder in turn.
3. Read the **"Absolute Position"** signal (NOT "Position"). This is the raw magnet reading in rotations.
4. Enter that value for the corresponding offset field.
5. Deploy and verify: joystick straight forward → robot drives straight, all wheels parallel.

> **Sign convention:** Phoenix 6 treats this as `MagnetSensor.MagnetOffset`. Positive offset rotates the zero point CCW when viewed from the top of the encoder.

---

## After All Configuration — Verification Checklist

Run these checks in order. Fix each before moving to the next.

- [ ] `./gradlew build` — zero compilation errors
- [ ] `./gradlew test` — all unit tests pass
- [ ] All CAN devices appear in Phoenix Tuner X with correct IDs
- [ ] All four swerve wheels track straight forward (no crab-walking)
- [ ] Joystick forward → robot drives straight, no rotation drift
- [ ] Heading lock holds heading when rotation stick is released
- [ ] Drive motors do not trip breakers under hard acceleration
- [ ] Intake deploys and stows smoothly without slamming hard stops
- [ ] Intake roller collects balls reliably (not pushing them away)
- [ ] Flywheel spins to target RPS consistently within `kFlywheelToleranceRPS`
- [ ] Hood reaches commanded angle without oscillation or sag
- [ ] Turret rotates to commanded angle without oscillation
- [ ] Shoot at 3 m → balls land in HUB (efficiency / bias baseline check)
- [ ] Shoot at 6 m → balls land in HUB (far-bias check)
- [ ] Alliance zone lob → balls land in alliance zone
- [ ] Vision pose estimate matches actual field position during teleop
- [ ] Ball detection triggers when hopper is loaded, clears when empty
- [ ] Feeder/spindexer motors do not overheat during sustained shooting

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
| `kHoodMinMotorRotations` | 9 | 0.0 | rot |
| `kHoodMaxMotorRotations` | 9 | 0.0 | rot |
| `kTurretGearRatio` | 10 | 100.0 | — |
| `kTurretCCWLimitMotorRotations` | 10 | 0.0 | rot |
| `kTurretCWLimitMotorRotations` | 10 | 0.0 | rot |
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
| `kAllianceZoneFlywheelRPS` | 11 | 35.0 | rps |
| `kAllianceZoneHoodAngleDeg` | 11 | 40.0 | deg |
| `kAllianceZoneTurretAngleDeg` | 11 | 172.0 | deg |
| `kAllianceZoneFeederSpeed` | 11 | 0.60 | duty |
| `kPreFeedReverseDurationSeconds` | 11 | 0.12 | s |
| `kPreFeedReverseFeederSpeed` | 11 | -0.25 | duty |
| `kPreFeedReverseSpindexerSpeed` | 11 | -0.15 | duty |
| `kFeederMotorId` | 12 | 23 | CAN ID |
| `kBeamBreakDIOPort` | 12 | 0 | DIO port |
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
| `kIntakeExtendedMotorRotations` | 14 | -0.01293 | rot |
| `kRollerDiameterMeters` | 14 | 0.0508 | m |
| `kRollerGearRatio` | 14 | 5.0 | — |
| `kDeployP/I/D/S` | 14 | 40/0/1/0.12 | — |
| `kDeployMaxVelocity` | 14 | 4.0 | rot/s |
| `kDeployMaxAcceleration` | 14 | 8.0 | rot/s² |
| `kDeployCurrentLimit` | 14 | 30.0 | A |
| `kRollerCurrentLimit` | 14 | 40.0 | A |
| `kRollerIntakeSpeed` | 14 | 0.8 | duty |
| `kRollerOuttakeSpeed` | 14 | -0.6 | duty |
| `kDeployLeftGravityVoltages[5]` | 14 | {0,0.10,0.25,0.35,0.40} | V |
| `kDeployRightGravityVoltages[5]` | 14 | {0,0.15,0.35,0.50,0.60} | V |
| `kFrontLeftCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kFrontRightCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kBackLeftCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kBackRightCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kIntakeCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kSideCam X/Y/Z/Roll/Pitch/Yaw` | 15 | placeholders | m / rad |
| `kMultiTagStdDevs[3]` | 16 | {0.5,0.5,1.0} | m / rad |
| `kSingleTagStdDevs[3]` | 16 | {4.0,4.0,8.0} | m / rad |
| `kAggressiveMultiTagStdDevs[3]` | 16 | {0.1,0.1,0.2} | m / rad |
| `kBallChaseYawP/I/D` | 16 | 0.04/0/0.002 | — |
| `kDefaultSpindexerBaselineCurrent` | 17 | 3.0 | A |
| `kDefaultFeederBaselineCurrent` | 17 | 2.5 | A |
| `kSpindexerLoadedThreshold` | 17 | 2.0 | A |
| `kSpindexerEmptyThreshold` | 17 | 0.5 | A |
| `kSpindexerFullLoadDelta` | 17 | 8.0 | A |
| `kFeederBallPresentThreshold` | 17 | 3.0 | A |
| `kTrackWidthMeters` | 18 | 0.5779 | m |
| `kWheelBaseMeters` | 18 | 0.5779 | m |
| `kFrontLeftEncoderOffset` | 19 | 0.0 | rot |
| `kFrontRightEncoderOffset` | 19 | 0.0 | rot |
| `kBackLeftEncoderOffset` | 19 | 0.0 | rot |
| `kBackRightEncoderOffset` | 19 | 0.0 | rot |
