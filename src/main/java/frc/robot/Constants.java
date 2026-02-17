// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.CANBus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        // Joystick deadband
        public static final double kDeadband = 0.1;
    }

    // ==================== CAN BUS ====================
    public static final CANBus kCANivoreBus = new CANBus("CANivore");

    public static final class SwerveConstants {
        // ==================== GYRO ====================
        // Pigeon2 CAN ID
        public static final int kPigeonId = 13;

        // ==================== MK4i MODULE PHYSICAL CONSTANTS ====================
        // MK4i L2 gear ratios (change if using L1, L3, or L4)
        // L1 = 8.14:1, L2 = 6.75:1, L3 = 6.12:1, L4 = 5.14:1
        public static final double kDriveGearRatio = 6.75;  // L2
        public static final double kSteerGearRatio = 150.0 / 7.0; // ~21.43:1 for all MK4i variants

        // Wheel diameter (4 inches nominal)
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // ==================== CHASSIS DIMENSIONS ====================
        // Distance between centers of left and right modules (track width)
        // Distance between centers of front and back modules (wheelbase)
        // Adjust these to your robot's actual measurements!
        public static final double kTrackWidthMeters = Units.inchesToMeters(22.75);
        public static final double kWheelBaseMeters = Units.inchesToMeters(22.75);

        // Distance from robot center to the furthest module
        public static final double kDriveBaseRadiusMeters =
                Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

        // ==================== KINEMATICS ====================
        // Module positions relative to robot center (FL, FR, BL, BR)
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),   // Front Left
                new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),  // Front Right
                new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),  // Back Left
                new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)  // Back Right
        );

        // ==================== SPEED LIMITS ====================
        // Derived from Kraken X60 specs: 6000 RPM = 100 RPS free speed
        private static final double kKrakenFreeSpeedRPS = 100.0;
        public static final double kTheoreticalMaxSpeedMPS =
                (kKrakenFreeSpeedRPS / kDriveGearRatio) * kWheelCircumferenceMeters;
        // 95% of theoretical to account for real-world friction and voltage drop
        public static final double kMaxSpeedMetersPerSecond = kTheoreticalMaxSpeedMPS * 0.95;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; // ~1 rotation/s

        // ==================== DRIVE MOTOR PID / FF ====================
        // These values need to be tuned on your specific robot!
        public static final double kDriveP = 0.1;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveS = 0.0;   // Static friction feedforward
        public static final double kDriveV = 0.12;   // Velocity feedforward (V * rps = volts)
        public static final double kDriveA = 0.0;    // Acceleration feedforward

        // ==================== STEER MOTOR PID ====================
        public static final double kSteerP = 100.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 0.5;

        // ==================== AUTO TRAJECTORY FOLLOWING PID ====================
        // These are used by the Choreo trajectory follower to correct position error
        public static final double kAutoTranslationP = 10.0;
        public static final double kAutoTranslationI = 0.0;
        public static final double kAutoTranslationD = 0.0;
        public static final double kAutoRotationP = 7.5;
        public static final double kAutoRotationI = 0.0;
        public static final double kAutoRotationD = 0.0;

        // ==================== CURRENT LIMITS ====================
        public static final double kDriveCurrentLimit = 60.0;          // Amps (supply)
        public static final double kDriveStatorCurrentLimit = 80.0;    // Amps (stator — prevents wheel slip)
        public static final double kSteerCurrentLimit = 30.0;          // Amps

        // ==================== SLEW RATE LIMITING (smoother driver control) ====================
        public static final double kTranslationSlewRate = 6.0;    // m/s per second
        public static final double kRotationSlewRate = 8.0;        // rad/s per second

        // ==================== HEADING LOCK PID ====================
        // Holds current heading when driver releases rotation stick.
        // ProfiledPIDController with trapezoidal velocity constraints.
        public static final double kHeadingLockP = 5.0;
        public static final double kHeadingLockI = 0.0;
        public static final double kHeadingLockD = 0.3;
        public static final double kMaxAngularAccelRadiansPerSecondSq = 8.0;

        // ==================== SLOW MODE ====================
        // Multipliers applied when slow mode trigger is held.
        // Fine positioning near scoring positions or during defense.
        public static final double kSlowModeSpeedMultiplier = 0.35;
        public static final double kSlowModeRotMultiplier = 0.40;

        // ==================== ODOMETRY ====================
        public static final double kOdometryPeriodSeconds = 0.004;    // 250Hz
        public static final double kMainLoopPeriodSeconds = 0.02;     // 50Hz

        // ==================== SIGNAL UPDATE FREQUENCIES ====================
        public static final double kPigeonYawUpdateFreqHz = 100.0;
        public static final double kPigeonAngVelUpdateFreqHz = 100.0;
        public static final double kModulePositionUpdateFreqHz = 100.0;
        public static final double kModuleVelocityUpdateFreqHz = 50.0;

        // ==================== VOLTAGE LIMITS ====================
        public static final double kPeakForwardVoltage = 12.0;
        public static final double kPeakReverseVoltage = -12.0;

        // ==================== SLIP DETECTION ====================
        public static final double kSlipDetectionThreshold = 0.30;    // 30% discrepancy
        public static final double kSlipMinCommandedSpeed = 0.5;      // m/s
        public static final double kSlipMinActualSpeed = 0.1;         // m/s

        // ==================== MODULE DRIVE THRESHOLDS ====================
        public static final double kAntiJitterSpeedMPS = 0.05;       // hold last angle below this

        // ==================== HEADING LOCK TOLERANCE ====================
        public static final double kHeadingLockToleranceDeg = 1.5;

        // ==================== VISION ====================
        public static final double kMaxVisionMeasurementAgeSec = 0.100; // reject stale measurements

        // ==================== MODULE HEALTH ====================
        public static final double kMaxSignalAgeSeconds = 0.5;       // stale CAN signal threshold

        // ==================== SIMULATION ====================
        public static final double kDriveSimMOI = 0.025;             // kg*m^2 (wheel + gearbox)
        public static final double kSteerSimMOI = 0.004;             // kg*m^2 (module azimuth)

        // ==================== MODULE CAN IDs AND OFFSETS ====================
        // Front Left Module
        public static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftSteerMotorId = 2;
        public static final int kFrontLeftEncoderId = 3;
        public static final double kFrontLeftEncoderOffset = 0.0; // rotations — calibrate!

        // Front Right Module
        public static final int kFrontRightDriveMotorId = 4;
        public static final int kFrontRightSteerMotorId = 5;
        public static final int kFrontRightEncoderId = 6;
        public static final double kFrontRightEncoderOffset = 0.0; // rotations — calibrate!

        // Back Left Module
        public static final int kBackLeftDriveMotorId = 7;
        public static final int kBackLeftSteerMotorId = 8;
        public static final int kBackLeftEncoderId = 9;
        public static final double kBackLeftEncoderOffset = 0.0; // rotations — calibrate!

        // Back Right Module
        public static final int kBackRightDriveMotorId = 10;
        public static final int kBackRightSteerMotorId = 11;
        public static final int kBackRightEncoderId = 12;
        public static final double kBackRightEncoderOffset = 0.0; // rotations — calibrate!
    }

    // ==================== HUB — FIELD STRUCTURE CONSTANTS ====================
    // The HUB is the primary scoring target in 2026 REBUILT™.
    // It is a multi-stage rectangular prism located in the NEUTRAL ZONE.
    // FUEL (foam balls) are scored through a hexagonal opening at the top.
    // Scored FUEL is distributed back onto the field through 4 base exits.
    public static final class HubConstants {

        // ==================== PHYSICAL DIMENSIONS ====================
        /** Base footprint: 47in × 47in square on carpet */
        public static final double kBaseWidthMeters = Units.inchesToMeters(47.0);  // ~1.194m
        public static final double kBaseLengthMeters = Units.inchesToMeters(47.0); // ~1.194m

        /** Front edge of scoring opening: 72in above the floor */
        public static final double kScoringOpeningHeightMeters = Units.inchesToMeters(72.0); // ~1.829m

        /** Hexagonal scoring opening width: 41.7in */
        public static final double kScoringOpeningWidthMeters = Units.inchesToMeters(41.7); // ~1.059m

        /** Number of FUEL exits at the base of the HUB */
        public static final int kBaseExitCount = 4;

        // ==================== FIELD POSITIONS (alliance-aware) ====================
        // Field: 16.54m × 8.07m
        // Each HUB is centered along field width, 158.6in (~4.03m) from its alliance wall
        public static final double kDistanceFromAllianceWallMeters = Units.inchesToMeters(158.6); // ~4.028m
        public static final Translation2d kBlueHubCenter = new Translation2d(
                kDistanceFromAllianceWallMeters,
                AutoConstants.kFieldWidthMeters / 2.0);
        public static final Translation2d kRedHubCenter = new Translation2d(
                AutoConstants.kFieldLengthMeters - kDistanceFromAllianceWallMeters,
                AutoConstants.kFieldWidthMeters / 2.0);

        /** Returns the active alliance's HUB center position at runtime. */
        public static Translation2d getActiveHubPosition() {
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                return kRedHubCenter;
            }
            return kBlueHubCenter;
        }

        // ==================== APRILTAG MARKERS ====================
        // The HUB has 16 unique AprilTags (2 per face × 4 faces per HUB × 2 HUBs)
        // Tag centers are 44.25in (1.124m) off the floor.
        public static final double kAprilTagHeightMeters = Units.inchesToMeters(44.25); // 1.124m

        /** Blue alliance HUB AprilTag IDs (2 tags per face × 4 faces) */
        public static final int[] kBlueHubTagIds = { 2, 3, 4, 5, 8, 9, 10, 11 };
        /** Red alliance HUB AprilTag IDs (2 tags per face × 4 faces) */
        public static final int[] kRedHubTagIds  = { 18, 19, 20, 21, 24, 25, 26, 27 };
        /** All HUB AprilTag IDs across both alliances */
        public static final int[] kAllHubTagIds  = {
            2, 3, 4, 5, 8, 9, 10, 11,
            18, 19, 20, 21, 24, 25, 26, 27
        };

        /** Returns the HUB AprilTag IDs for the current alliance. */
        public static int[] getActiveHubTagIds() {
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                return kRedHubTagIds;
            }
            return kBlueHubTagIds;
        }

        /** Check if a given AprilTag ID belongs to any HUB. */
        public static boolean isHubTag(int tagId) {
            for (int id : kAllHubTagIds) {
                if (id == tagId) return true;
            }
            return false;
        }

        /** Check if a given AprilTag ID belongs to the current alliance's HUB. */
        public static boolean isOwnHubTag(int tagId) {
            for (int id : getActiveHubTagIds()) {
                if (id == tagId) return true;
            }
            return false;
        }

        // ==================== SCORING RULES (Game Manual Section 6.5, Table 6-4) ====================
        // Active HUB:   ALLIANCE-colored DMX light bars ON  → scored FUEL = 1 point
        // Inactive HUB: DMX light bars OFF                  → scored FUEL = 0 points
        //
        // During ALLIANCE SHIFTS in Teleop, active/inactive alternates every 25 seconds.
        // TRANSITION SHIFT (first 10s): both HUBs active.
        // END GAME (last 30s): both HUBs active.
        // The alliance that scored MORE FUEL in Autonomous starts with HUB
        // INACTIVE for Shift 1.
        //
        // FUEL scored continues to count for up to 3 seconds after HUB deactivates
        // (processing time grace period).
        /** Match points per FUEL scored while HUB is Active */
        public static final int kActiveHubPointsPerFuel = 1;
        /** Match points per FUEL scored while HUB is Inactive */
        public static final int kInactiveHubPointsPerFuel = 0;

        // ==================== TOWER SCORING (Game Manual Table 6-4) ====================
        /** LEVEL 1 climb: 15 pts in AUTO (max 2 robots), 10 pts in TELEOP */
        public static final int kLevel1AutoPoints = 15;
        public static final int kLevel1TeleopPoints = 10;
        /** LEVEL 2 climb: 20 pts (TELEOP only) */
        public static final int kLevel2TeleopPoints = 20;
        /** LEVEL 3 climb: 30 pts (TELEOP only) */
        public static final int kLevel3TeleopPoints = 30;

        // ==================== RANKING POINT THRESHOLDS (Game Manual Table 6-5) ====================
        // Regional / District Events thresholds (may increase at Championships)
        /** ENERGIZED RP: ≥ 100 FUEL scored in active HUBs during the match */
        public static final int kEnergizedRpThreshold = 100;
        /** SUPERCHARGED RP: ≥ 360 FUEL scored in active HUBs during the match */
        public static final int kSuperchargedRpThreshold = 360;
        /** TRAVERSAL RP: ≥ 50 total TOWER points during the match */
        public static final int kTraversalRpThreshold = 50;

        // ==================== MATCH FUEL STAGING (Game Manual Section 6.3.4) ====================
        /** Maximum FUEL preloaded per robot */
        public static final int kMaxPreloadPerRobot = 8;
        /** Total FUEL staged per match */
        public static final int kTotalFuelPerMatch = 504;
        /** FUEL per DEPOT */
        public static final int kFuelPerDepot = 24;
        /** FUEL per OUTPOST CHUTE */
        public static final int kFuelPerChute = 24;

        // ==================== ANTI-CAMPING (G408) ====================
        // ROBOTS may NOT catch FUEL directly from HUB exit chutes.
        // FUEL must touch the carpet or another object before gaining control.
        // Minimum standoff distance from HUB base exits for strategic positioning.
        public static final double kAntiCampStandoffMeters = 0.5;
    }

    public static final class ShooterConstants {
        // ==================== GAME PIECE — 2026 REBUILT "FUEL" ====================
        // 5.91-inch (15.0 cm) high-density foam ball
        public static final double kBallDiameterMeters = Units.inchesToMeters(5.91); // 0.150m
        public static final double kBallRadiusMeters = kBallDiameterMeters / 2.0;
        public static final double kBallMassKg = 0.215; // ~215g (range: 203-227g)
        public static final double kBallCompressionMeters = 0.020; // ~20mm (range: 15-25mm)

        // Effective ball diameter when compressed between flywheels
        public static final double kCompressedBallDiameter = kBallDiameterMeters - kBallCompressionMeters;

        // ==================== HUB / GOAL SPECIFICATIONS (delegated to HubConstants) ====================
        public static final double kHubHeightMeters = HubConstants.kScoringOpeningHeightMeters;
        public static final double kHubInnerDiameterMeters = HubConstants.kScoringOpeningWidthMeters;

        // ==================== HUB FIELD POSITIONS (alliance-aware) ====================
        public static final Translation2d kBlueHubPosition = HubConstants.kBlueHubCenter;
        public static final Translation2d kRedHubPosition  = HubConstants.kRedHubCenter;

        public static Translation2d getActiveHubPosition() {
            return HubConstants.getActiveHubPosition();
        }

        // Legacy constant — use getActiveHubPosition() instead
        public static final Translation2d kHubFieldPosition = kBlueHubPosition;

        // ==================== AUTO-AIM CONSTANTS ====================
        // Minimum distance where auto-aim will engage (closer than this = too close to shoot)
        public static final double kMinShootingDistanceMeters = 1.2;
        // Maximum distance where auto-aim will engage (farther = not worth shooting)
        public static final double kMaxShootingDistanceMeters = 6.5;
        // Turret angular offset — compensate for turret not being exactly centered on robot (degrees)
        // Positive = turret is offset counterclockwise from robot forward. Set to 0 if centered.
        public static final double kTurretMountOffsetDegrees = 0.0;

        // ==================== SHOOTER GEOMETRY (measured on robot) ====================
        // Height of ball exit point above the floor. Measure from carpet to the
        // point where the ball leaves the flywheel gap.
        // TODO: measure on real robot
        public static final double kShooterExitHeightMeters = Units.inchesToMeters(24.0);

        // ==================== BALL-FLYWHEEL EFFICIENCY ====================
        // Ratio of actual ball exit speed to flywheel surface speed.
        // Accounts for foam ball compression, slip, friction losses.
        // v_exit = surface_speed × efficiency
        // Tune this single value on the real robot until shots consistently
        // land in the HUB at multiple distances.
        public static final double kShooterEfficiencyFactor = 0.30;

        // ==================== REAL-WORLD BIAS CORRECTION (DISTANCE-DEPENDENT) ====================
        // Multiplier applied to the final computed velocity to bridge the gap
        // between theoretical projectile physics (vacuum, no drag, no spin)
        // and real-world behavior. Compensates for:
        //   - Air drag (increases with distance — longer flight = more slowdown)
        //   - Ball spin from flywheel contact (creates lift or draw)
        //   - Foam ball deformation and energy loss in flight
        //   - Environmental factors (humidity, worn balls, altitude)
        //
        // The bias is linearly interpolated between near and far values:
        //   bias(d) = nearBias + (farBias − nearBias) × (d − minDist) / (maxDist − minDist)
        //
        // Close shots (2m): short flight time → minimal drag → bias ≈ 1.0
        // Far shots  (7m): long  flight time → significant drag → bias > 1.0
        //
        // 1.00 = pure theory (no correction)
        // >1.0 = real world needs MORE power than theory (typical: drag losses)
        // <1.0 = real world needs LESS power than theory (unlikely)
        //
        // Tuning:
        //   1. Start both at 1.00 (pure physics)
        //   2. Shoot at kMinShootingDistanceMeters — if short, increase Near
        //   3. Shoot at kMaxShootingDistanceMeters — if short, increase Far
        //   4. Verify mid-range shots land without re-tuning (interpolation handles it)
        public static final double kBiasFactorNear = 1.00; // at kMinShootingDistanceMeters
        public static final double kBiasFactorFar  = 1.00; // at kMaxShootingDistanceMeters

        // ==================== ARC / BASKETBALL CONSTRAINTS ====================
        // Minimum height (meters) the ball must peak ABOVE the Hub opening.
        // Like basketball: ball must arc up and come DOWN into the basket.
        // Higher = safer arc (swish), lower = flatter trajectory (more bounce-out risk).
        // 0.3m (~12 inches) gives a clean descending arc at all distances.
        public static final double kMinApexClearanceMeters = 0.30;

        // ==================== TABLE GENERATION ====================
        // Step size for physics-computed lookup table entries.
        // Smaller = more entries = smoother interpolation, but diminishing returns.
        public static final double kTableDistanceStepMeters = 0.5;

        // Whether shoot-while-moving compensation is enabled.
        // Set to false during initial tuning, enable once the static aim is dialed in.
        public static final boolean kShootWhileMovingEnabled = true;

        // ==================== ALLIANCE ZONE DUMP ====================
        // Fixed shooter parameters for dumping FUEL from neutral zone into alliance zone.
        // No auto-aim — turret faces backward, hood and flywheel set to a fixed lob shot.
        // TODO: tune these values on real robot to land balls in the alliance zone
        public static final double kAllianceZoneFlywheelRPS = 35.0;   // low-power lob
        public static final double kAllianceZoneHoodAngleDeg = 40.0;  // steep arc
        public static final double kAllianceZoneTurretAngleDeg = 172.0; // face backward (toward own alliance zone, buffered from ±175° limit)
        public static final double kAllianceZoneFeederSpeed = 0.60;   // moderate feed

        // ==================== TURRET WRAPAROUND ====================
        // When the turret is within this many degrees of either limit, the drivetrain
        // should auto-rotate to bring the target back toward turret center.
        // This prevents the turret from hitting hard limits during fast rotations.
        // Uses hysteresis: activates at kTurretWraparoundThresholdDeg,
        // deactivates at kTurretWraparoundReleaseThresholdDeg.
        public static final double kTurretWraparoundThresholdDeg = 150.0;
        public static final double kTurretWraparoundReleaseThresholdDeg = 140.0;
        // Max additional rotation speed (rad/s) the wraparound hint can add.
        // Scales proportionally from 0 at the release threshold to max at the hard limit.
        public static final double kTurretWraparoundMaxHintRadPerSec = 1.5;

        // ==================== PRE-FEED REVERSE ====================
        // Before feeding into the shooter, briefly reverse feeder + spindexer
        // at low speed to create ball clearance and prevent jams.
        // Like pulling the bolt back before firing — creates space so balls
        // feed cleanly one at a time instead of jamming against each other.
        public static final double kPreFeedReverseDurationSeconds = 0.12; // 120ms reverse pulse
        public static final double kPreFeedReverseFeederSpeed = -0.25;    // slow reverse (gentler than unjam)
        public static final double kPreFeedReverseSpindexerSpeed = -0.15; // slow reverse

        // ==================== ENDGAME ====================
        // Time remaining in Teleop (seconds) to trigger "endgame dump" mode.
        // In the last seconds, dump all remaining FUEL regardless of Hub state.
        public static final double kEndgameDumpTimeSeconds = 10.0;

        // ==================== MOTOR CAN IDs ====================
        public static final int kFlywheelMotorId = 20;   // Kraken — drives both top and bottom flywheels
        public static final int kHoodMotorId = 21;        // Kraken — adjusts hood angle
        public static final int kTurretMotorId = 22;      // Kraken — rotates turret for aiming

        // ==================== FLYWHEEL PHYSICAL CONSTANTS ====================
        // Bottom flywheel: 4-inch diameter wheels + 2 iron inertia wheels (0.7kg each)
        public static final double kBottomFlywheelDiameterMeters = Units.inchesToMeters(4.0); // 0.1016m
        public static final double kBottomFlywheelRadiusMeters = kBottomFlywheelDiameterMeters / 2.0;
        public static final double kBottomFlywheelCircumferenceMeters = kBottomFlywheelDiameterMeters * Math.PI;
        public static final double kIronWheelMassKg = 0.7;  // each iron inertia wheel
        public static final int kIronWheelCount = 2;

        // Top (hood) flywheel: 2-inch diameter wheels, 2 rows
        public static final double kTopFlywheelDiameterMeters = Units.inchesToMeters(2.0); // 0.0508m
        public static final double kTopFlywheelRadiusMeters = kTopFlywheelDiameterMeters / 2.0;
        public static final double kTopFlywheelCircumferenceMeters = kTopFlywheelDiameterMeters * Math.PI;

        // Surface speed equalization gear ratio:
        // Both flywheels are driven by ONE motor, geared so surface speed is equal.
        // Surface speed = RPM * circumference
        // For equal surface speed: RPM_top * C_top = RPM_bottom * C_bottom
        // RPM_top / RPM_bottom = C_bottom / C_top = 4" / 2" = 2.0
        // So the top flywheel spins 2x faster than the bottom flywheel.
        // This means the gear ratio from motor to bottom is X, and motor to top is 2X.
        // Motor to bottom flywheel: 2:1 (2 motor rotations per 1 flywheel rotation)
        public static final double kFlywheelGearRatio = 2.0;
        // Bottom to top flywheel: 1:2 (top spins 2x faster for equal surface speed)
        public static final double kTopToBottomGearRatio = 2.0;

        // ==================== FLYWHEEL PID / FF ====================
        // Velocity PID for flywheel speed control
        public static final double kFlywheelP = 0.5;
        public static final double kFlywheelI = 0.0;
        public static final double kFlywheelD = 0.0;
        public static final double kFlywheelS = 0.0;    // Static friction
        public static final double kFlywheelV = 0.12;   // Velocity FF (volts per rps)
        public static final double kFlywheelA = 0.0;    // Acceleration FF

        // Acceptable velocity error before we consider flywheel "at speed"
        public static final double kFlywheelToleranceRPS = 2.0; // rotations per second

        // Idle pre-spin speed when out of shooting range (keeps flywheel warm for faster spin-up)
        public static final double kIdleFlywheelRPS = 15.0;

        // Maximum flywheel RPS used to normalize shooter speed for feeder/spindexer scaling.
        // Feeder and spindexer duty cycles are computed as:
        //   duty = ratio × (targetFlywheelRPS / kMaxFlywheelRPS)
        // This value should be >= the highest RPS in the shooting table.
        // TODO: set from actual shooting table max after physics tuning
        public static final double kMaxFlywheelRPS = 90.0;

        // ==================== HOOD CONSTANTS ====================
        // Hood gear ratio: 2:1 (2 motor rotations per mechanism rotation)
        public static final double kHoodGearRatio = 2.0;

        // Hood angle range (measured from horizontal / parallel to ground)
        public static final double kHoodMinAngleDegrees = 27.5;
        public static final double kHoodMaxAngleDegrees = 42.5;

        // TODO: move hood to min (most open), read Kraken motor encoder value
        private static final double kHoodMinMotorRotations = 0.0; // placeholder
        // TODO: move hood to max (most closed), read Kraken motor encoder value
        private static final double kHoodMaxMotorRotations = 0.0; // placeholder

        // Hood positions in mechanism rotations (for motor controller)
        public static final double kHoodMinRotations = kHoodMinMotorRotations / kHoodGearRatio;
        public static final double kHoodMaxRotations = kHoodMaxMotorRotations / kHoodGearRatio;

        // ==================== EXIT ANGLE MAPPING ====================
        // The hood mechanical angle maps to a ball exit angle.
        // Ball exits roughly perpendicular to the hood surface, so:
        //   exit_angle_from_horizontal = 90° − hood_angle
        // When hood is at minimum (open): ball exits steep (high angle).
        // When hood is at maximum (closed): ball exits flat (low angle).
        public static final double kExitAngleAtMinHood = 90.0 - kHoodMinAngleDegrees;
        public static final double kExitAngleAtMaxHood = 90.0 - kHoodMaxAngleDegrees;

        // Hood PID (position control)
        public static final double kHoodP = 8.0;
        public static final double kHoodI = 0.0;
        public static final double kHoodD = 0.2;
        public static final double kHoodG = 0.15; // Gravity feedforward (volts)

        // Hood tolerance
        public static final double kHoodToleranceDegrees = 0.5;
        public static final double kHoodToleranceRotations = kHoodToleranceDegrees / 360.0;

        // ==================== TURRET CONSTANTS ====================
        // Turret gear ratio: motor rotations per mechanism rotation
        // TODO: measure and set actual gear ratio
        public static final double kTurretGearRatio = 100.0; // placeholder

        // Turret rotation limits (degrees from center / forward)
        // Narrowed from ±180° to ±175° to prevent oscillation at the wrap boundary.
        public static final double kTurretMinAngleDegrees = -175.0;
        public static final double kTurretMaxAngleDegrees = 175.0;

        // TODO: move turret to CCW hard stop, read Kraken motor encoder value
        private static final double kTurretCCWLimitMotorRotations = 0.0; // placeholder
        // TODO: move turret to CW hard stop, read Kraken motor encoder value
        private static final double kTurretCWLimitMotorRotations = 0.0; // placeholder

        // Turret positions in mechanism rotations (for motor controller)
        // Derived from the two hard stop measurements and gear ratio
        public static final double kTurretMinRotations =
                kTurretCCWLimitMotorRotations / kTurretGearRatio;
        public static final double kTurretMaxRotations =
                kTurretCWLimitMotorRotations / kTurretGearRatio;

        // Turret PID (position control)
        public static final double kTurretP = 30.0;
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 0.5;

        // Turret tolerance
        public static final double kTurretToleranceDegrees = 1.0;
        public static final double kTurretToleranceRotations = kTurretToleranceDegrees / 360.0;

        // ==================== TURRET / HOOD PHYSICAL DIMENSIONS ====================
        /** Height of turret rotation axis above ground (meters). ~18 inches. */
        public static final double kTurretHeightMeters = 0.45;
        /** Height of hood pivot above ground (meters). Slightly above turret. */
        public static final double kHoodHeightMeters = 0.50;

        // ==================== FLYWHEEL RECOVERY ====================
        /** Cooldown time after a ball exits to let flywheel recover speed (seconds). */
        public static final double kFlywheelRecoverySeconds = 0.15;

        // ==================== RUNTIME HEALTH ====================
        /** Consecutive fault cycles before marking unhealthy (~1s at 50Hz). */
        public static final int kFaultCycleThreshold = 50;

        // ==================== SHOOTING LOOKUP TABLE ====================
        // Generated at robot init by ShooterPhysics from projectile motion equations.
        // No hardcoded values — everything derived from physical constants above.
        // See ShooterPhysics.java for the computation.
    }

    public static final class IntakeConstants {
        // ==================== MOTOR CAN IDs ====================
        // 2 Kraken X60 deploy motors (left + right, mirror-inverted)
        public static final int kLeftDeployMotorId = 24;
        public static final int kRightDeployMotorId = 25;
        // 1 Kraken X60 roller motor (collects balls)
        public static final int kRollerMotorId = 26;

        // ==================== DEPLOY CONSTANTS ====================
        // Deploy gear ratio: motor rotations per mechanism rotation
        public static final double kDeployGearRatio = 4.0;

        // Measured raw Kraken motor encoder readings (motor rotations)
        private static final double kStowedMotorRotations = 0.0; // measured at stowed position
        private static final double kExtendedMotorRotations = -0.01293; // measured at full deploy

        // Deploy positions in mechanism rotations (0 = stowed inside bumpers)
        // Negative = deployed outward past the bumper
        public static final double kDeployStowedRotations = kStowedMotorRotations / kDeployGearRatio;
        public static final double kDeployExtendedRotations =
                kExtendedMotorRotations / kDeployGearRatio;
        public static final double kDeployHoverRotations =
                kDeployExtendedRotations / 2.0;

        // Deploy PID (position control — gravity handled via lookup tables, NOT Arm_Cosine)
        public static final double kDeployP = 40.0;
        public static final double kDeployI = 0.0;
        public static final double kDeployD = 1.0;
        public static final double kDeployS = 0.12; // Static friction FF (volts) — minimum voltage to overcome friction
        // TODO: tune with SysId or by slowly increasing voltage until the arm just starts to move

        // ==================== DEPLOY GRAVITY FEEDFORWARD LOOKUP ====================
        // The passive linkage (free flap) creates a non-linear torque profile:
        //   - Stowed: linkage is folded, CG is near pivot → low gravity torque
        //   - Mid-deploy: linkage starts opening → torque ramps up
        //   - Full deploy: linkage fully extended → maximum gravity torque
        // A simple cosine doesn't capture this, so we use position → voltage tables.
        // The right side carries more weight (roller/linkage mass is off-center).
        //
        // Format: { position (mechanism rotations), voltage (V) }
        // Positions are negative (0 = stowed, kDeployExtendedRotations = full deploy).
        // TODO: tune all values on the real robot using AdvantageScope current plots
        public static final double[][] kDeployLeftGravityTable = {
            { 0.0,                                  0.0  },  // stowed — vertical, ~zero torque
            { kDeployExtendedRotations * 0.25,      0.10 },  // 25% deploy — linkage still folded
            { kDeployExtendedRotations * 0.50,      0.25 },  // 50% deploy — linkage opening
            { kDeployExtendedRotations * 0.75,      0.35 },  // 75% deploy — linkage mostly open
            { kDeployExtendedRotations,              0.40 },  // full deploy — linkage extended, max torque
        };

        public static final double[][] kDeployRightGravityTable = {
            { 0.0,                                  0.0  },  // stowed — vertical, ~zero torque
            { kDeployExtendedRotations * 0.25,      0.15 },  // 25% deploy — linkage still folded
            { kDeployExtendedRotations * 0.50,      0.35 },  // 50% deploy — linkage opening, extra weight kicks in
            { kDeployExtendedRotations * 0.75,      0.50 },  // 75% deploy — linkage mostly open
            { kDeployExtendedRotations,              0.60 },  // full deploy — linkage extended, max torque + extra weight
        };

        // Deploy tolerance — ~10% of full deploy travel
        // Travel = |kDeployExtendedRotations| ≈ 0.00323 mechanism rotations
        public static final double kDeployToleranceRotations = 0.0003;

        // Deploy current limit (amps) — lower because it's just swinging an arm
        public static final double kDeployCurrentLimit = 30.0;

        // Deploy stall detection
        public static final double kDeployStallCurrentThreshold = 25.0; // Amps
        public static final int kDeployStallCycleThreshold = 25;         // ~0.5s

        // Deploy motion magic (smooth trapezoidal motion)
        public static final double kDeployMaxVelocity = 4.0;     // rotations/sec
        public static final double kDeployMaxAcceleration = 8.0; // rotations/sec²

        // ==================== ROLLER CONSTANTS ====================
        // Roller speed (duty cycle) — how fast to spin the intake wheels
        public static final double kRollerIntakeSpeed = 0.8;   // collecting balls inward
        public static final double kRollerOuttakeSpeed = -0.6; // ejecting balls outward
        public static final double kRollerCurrentLimit = 40.0;

        // ==================== ROLLER PHYSICAL DIMENSIONS ====================
        // Needed to compute roller surface speed for chassis speed limiting.
        // Roller surface speed MUST exceed chassis speed while intaking,
        // otherwise we push balls away instead of collecting them.
        public static final double kRollerDiameterMeters = Units.inchesToMeters(2.0); // TODO: measure on robot
        public static final double kRollerGearRatio = 5.0; // motor rotations per roller rotation — TODO: verify

        // Kraken X60 free speed ≈ 6000 RPM = 100 RPS
        private static final double kKrakenFreeSpeedRPS = 100.0;

        // Computed roller surface speed at intake duty cycle (m/s)
        // = (motorRPS × dutyCycle / gearRatio) × (π × diameter)
        public static final double kRollerSurfaceSpeedMps =
                (kKrakenFreeSpeedRPS * kRollerIntakeSpeed / kRollerGearRatio)
                * (Math.PI * kRollerDiameterMeters);

        // ==================== INTAKE CHASSIS SPEED LIMIT ====================
        // While intaking, cap robot translation speed below roller surface speed.
        // If chassis moves faster than rollers spin, balls get pushed away.
        // Set to 85% of roller surface speed for reliable ball collection.
        public static final double kMaxChassisSpeedWhileIntaking =
                kRollerSurfaceSpeedMps * 0.85;
    }

    public static final class SpindexerConstants {
        // ==================== MOTOR CAN ID ====================
        public static final int kSpindexerMotorId = 27;

        // ==================== SPINDEXER CONSTANTS ====================
        // The spindexer is a rotating platform / indexer that moves balls
        // from the intake hopper toward the feeder (which feeds the shooter).
        // It spins continuously at a set speed while intaking or shooting.
        //
        // SPEED HIERARCHY (CRITICAL — prevents jams):
        //   Shooter flywheel >> Feeder > Spindexer > Intake roller (when feeding)
        //   Each stage must pull balls faster than the previous stage pushes them.
        //   If a downstream stage is slower, balls pile up and jam.
        public static final double kSpindexerIntakeSpeed = 0.4;  // slow spin while collecting
        // Feed speed is now computed dynamically as a fraction of the FEEDER speed:
        //   spindexer_duty = kSpindexerToFeederRatio × feeder_duty
        // This creates a chain: Shooter → Feeder → Spindexer
        // The hierarchy is automatically guaranteed as long as ratio < 1.0.
        public static final double kSpindexerToFeederRatio = 0.65;  // spindexer runs at 65% of feeder speed
        public static final double kSpindexerReverseSpeed = -0.3; // unjam / reverse
        public static final double kSpindexerCurrentLimit = 30.0;

        // ==================== STALL / JAM DETECTION ====================
        public static final double kSpindexerStallCurrentThreshold = 25.0;
        public static final double kSpindexerStallVelocityThreshold = 0.3;
        public static final int kSpindexerStallCycleThreshold = 15;
        public static final double kSpindexerUnjamDuration = 0.3;
        public static final double kSpindexerUnjamSpeed = -0.5;
    }

    public static final class FeederConstants {
        // ==================== MOTOR CAN ID ====================
        public static final int kFeederMotorId = 23;

        // ==================== BEAM BREAK SENSOR ====================
        // DIO port for the beam break sensor at the feeder exit (before flywheels).
        // This is the ONLY beam break on the robot — used to detect balls leaving
        // the magazine and decrement ball count.
        public static final int kBeamBreakDIOPort = 0; // TODO: set to actual DIO port

        // ==================== FEEDER CONSTANTS ====================
        public static final double kFeederCurrentLimit = 30.0;

        // ==================== STALL / JAM DETECTION ====================
        public static final double kFeederStallCurrentThreshold = 25.0; // Amps — jam detected above this
        public static final double kFeederStallVelocityThreshold = 0.5; // RPS — considered stalled below this
        public static final int kFeederStallCycleThreshold = 10;        // consecutive cycles before jam declared
        public static final double kFeederUnjamReverseDuration = 0.25;  // seconds to reverse when jammed
        public static final double kFeederUnjamReverseSpeed = -0.5;     // duty cycle for unjam reverse

        // ==================== FEEDER SPEED (ratio of shooter) ====================
        // Feeder speed is derived directly from the shooter's target flywheel RPS:
        //   feeder_duty = kFeederSpeedRatio × (targetFlywheelRPS / kMaxFlywheelRPS)
        //
        // No separate interpolation table needed — the shooter already does the
        // distance-based lookup. The feeder and spindexer form a chain:
        //   Shooter RPS → Feeder duty → Spindexer duty
        //
        // SPEED HIERARCHY (CRITICAL — prevents jams):
        //   Shooter flywheel >> Feeder > Spindexer
        //   Feeder = kFeederSpeedRatio × normalized shooter RPS
        //   Spindexer = kSpindexerToFeederRatio × feeder duty (always < feeder)
        //   Flywheel surface speed is always >> feeder surface speed by design
        //   (flywheel runs velocity PID at high RPS, feeder is duty-cycle capped).
        public static final double kFeederSpeedRatio = 0.75;
    }

    public static final class VisionConstants {
        // ==================== FIELD LAYOUT ====================
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // ==================== HUB APRILTAG VISION TARGETING ====================
        // The HUB AprilTags are at 44.25in (1.124m) height — used for
        // camera pitch calculation and distance estimation when locking on.
        public static final double kHubTagHeightMeters = HubConstants.kAprilTagHeightMeters;

        // ==================== CAMERA NAMES ====================
        // These MUST match the camera nicknames set in the PhotonVision UI
        // 4 AprilTag cameras at corners + 1 intake camera + 1 side camera
        public static final String kFrontLeftCamName = "cam_front_left";
        public static final String kFrontRightCamName = "cam_front_right";
        public static final String kBackLeftCamName = "cam_back_left";
        public static final String kBackRightCamName = "cam_back_right";
        public static final String kIntakeCamName = "cam_intake";    // below slapdown intake, ball detection
        public static final String kSideCamName = "cam_side";        // side-facing camera

        // ==================== ROBOT DIMENSIONS FOR CAMERA MOUNTING ====================
        // Half-track / half-wheelbase — cameras are mounted at the corners
        private static final double kHalfTrack = SwerveConstants.kTrackWidthMeters / 2.0;
        private static final double kHalfBase = SwerveConstants.kWheelBaseMeters / 2.0;

        // Camera mount height (approximate — all AprilTag cameras at same height)
        // Adjust this to the actual mounting height on your robot!
        private static final double kAprilTagCamHeightMeters = Units.inchesToMeters(12.0);

        // Cameras tilt slightly upward to see AprilTags on the field walls/towers
        // Positive pitch = looking up (in WPILib camera convention, negative pitch = tilt up)
        private static final double kAprilTagCamPitchRadians = Units.degreesToRadians(-15.0); // 15° upward tilt

        // ==================== CAMERA FOV ====================
        // Each camera covers 80° horizontal FOV
        public static final double kCameraHFovDegrees = 80.0;

        // ==================== APRILTAG CAMERA TRANSFORMS ====================
        // Each camera is at a 45° corner of the robot, facing outward at 45° from the robot's sides.
        // Transform3d: Translation3d(x_forward, y_left, z_up) + Rotation3d(roll, pitch, yaw)
        // Yaw: 45° for FL, -45° for FR, 135° for BL, -135° for BR (each bisects the corner)

        // Front-Left camera: front-left corner, facing 45° left of forward
        public static final Transform3d kRobotToFrontLeftCam = new Transform3d(
                new Translation3d(kHalfBase, kHalfTrack, kAprilTagCamHeightMeters),
                new Rotation3d(0, kAprilTagCamPitchRadians, Units.degreesToRadians(45.0)));

        // Front-Right camera: front-right corner, facing 45° right of forward
        public static final Transform3d kRobotToFrontRightCam = new Transform3d(
                new Translation3d(kHalfBase, -kHalfTrack, kAprilTagCamHeightMeters),
                new Rotation3d(0, kAprilTagCamPitchRadians, Units.degreesToRadians(-45.0)));

        // Back-Left camera: back-left corner, facing 135° left of forward (45° from back)
        public static final Transform3d kRobotToBackLeftCam = new Transform3d(
                new Translation3d(-kHalfBase, kHalfTrack, kAprilTagCamHeightMeters),
                new Rotation3d(0, kAprilTagCamPitchRadians, Units.degreesToRadians(135.0)));

        // Back-Right camera: back-right corner, facing 135° right of forward (45° from back)
        public static final Transform3d kRobotToBackRightCam = new Transform3d(
                new Translation3d(-kHalfBase, -kHalfTrack, kAprilTagCamHeightMeters),
                new Rotation3d(0, kAprilTagCamPitchRadians, Units.degreesToRadians(-135.0)));

        // ==================== NON-APRILTAG CAMERAS ====================
        // Intake camera: mounted below the slapdown intake, looking down at the ground for ball detection
        // This camera does NOT do AprilTag detection — it runs object detection (ML pipeline)
        private static final double kIntakeCamHeightMeters = Units.inchesToMeters(6.0);
        public static final Transform3d kRobotToIntakeCam = new Transform3d(
                new Translation3d(kHalfBase + Units.inchesToMeters(2.0), 0.0, kIntakeCamHeightMeters), // 2in past front
                new Rotation3d(0, Units.degreesToRadians(45.0), 0.0)); // 45° downward, facing forward

        // Side camera: mounted on one side of the robot
        // Used for additional awareness (trench, game pieces, etc.)
        private static final double kSideCamHeightMeters = Units.inchesToMeters(10.0);
        public static final Transform3d kRobotToSideCam = new Transform3d(
                new Translation3d(0.0, kHalfTrack, kSideCamHeightMeters), // left side, centered lengthwise
                new Rotation3d(0, Units.degreesToRadians(-5.0), Units.degreesToRadians(90.0))); // facing left, slight upward tilt

        // ==================== ALL APRILTAG CAMERAS (for iteration) ====================
        public static final String[] kAprilTagCamNames = {
                kFrontLeftCamName, kFrontRightCamName, kBackLeftCamName, kBackRightCamName
        };

        public static final Transform3d[] kAprilTagCamTransforms = {
                kRobotToFrontLeftCam, kRobotToFrontRightCam, kRobotToBackLeftCam, kRobotToBackRightCam
        };

        // ==================== POSE ESTIMATION STANDARD DEVIATIONS ====================
        // How much we trust vision measurements vs. odometry
        // Lower values = more trust in vision. (x meters, y meters, heading radians)
        // These are STARTING values — tune on your robot!

        // When we see multiple AprilTags, the estimate is very reliable
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);

        // Single tag is less reliable — higher standard deviations
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);

        // ==================== FILTERING THRESHOLDS ====================
        // Reject pose estimates that are too far from current pose (likely noise / wrong tag)
        public static final double kMaxPoseAmbiguity = 0.2; // reject high-ambiguity single-tag results
        public static final double kMaxPoseJumpMeters = 2.0; // reject estimates that jump more than 2m
        public static final double kMaxTagDistanceMeters = 5.5; // ignore tags further than this

        // ==================== VISION-CORRECTED ODOMETRY ====================
        // When AprilTags are visible, tighten std devs to aggressively correct odometry.
        // When no tags are visible, odometry relies solely on wheel encoders + gyro (drifts).
        // Aggressive multi-tag trust ensures odometry snaps back quickly when tags reappear.
        public static final Matrix<N3, N1> kAggressiveMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);

        // Maximum acceptable distance discrepancy between odometry-based distance to Hub
        // and vision-measured distance to an AprilTag on the Hub (meters).
        // If the discrepancy exceeds this, the vision system flags a distance validation warning.
        public static final double kMaxDistanceDiscrepancyMeters = 1.0;

        // ==================== VISION INTAKE (BALL CHASE) ====================
        /** PID gains for yaw correction when chasing a ball. */
        public static final double kBallChaseYawP = 0.04;
        public static final double kBallChaseYawI = 0.0;
        public static final double kBallChaseYawD = 0.002;
        /** Yaw tolerance for ball chase PID (degrees). */
        public static final double kBallChaseYawTolerance = 2.0;
        /** Ball area threshold — above this, ball is close enough to collect. */
        public static final double kBallCloseEnoughArea = 15.0;
        /** Max forward speed when chasing a ball (m/s). */
        public static final double kBallChaseMaxSpeed = 2.0;
        /** Min forward speed when close to the ball (m/s). */
        public static final double kBallChaseMinSpeed = 0.5;
        /** Debounce: wait this long before spin-searching after losing a ball (seconds). */
        public static final double kBallLostDebounceSeconds = 0.5;

        // ==================== FIELD BOUNDARY VALIDATION ====================
        /** Margin outside the field for pose rejection (meters). */
        public static final double kFieldBoundaryMargin = 1.0;
    }

    // ==================== LED CONSTANTS ====================
    public static final class LEDConstants {
        /** PWM port for the addressable LED strip. */
        public static final int kLedPort = 0;
        /** Number of LEDs on the strip. */
        public static final int kLedCount = 60;
    }

    // ==================== AUTONOMOUS CONSTANTS ====================
    public static final class AutoConstants {

        // ==================== FIELD DIMENSIONS (Game Manual Section 5.2) ====================
        /** Field length: 651.2 inches. */
        public static final double kFieldLengthMeters = Units.inchesToMeters(651.2);
        /** Field width: 317.7 inches. */
        public static final double kFieldWidthMeters = Units.inchesToMeters(317.7);

        // ==================== AUTO PERIOD (Game Manual Section 6.4) ====================
        /** Autonomous period: first 20 seconds of the match. Both HUBs active. */
        public static final double kAutoDurationSeconds = 20.0;

        // ==================== MATCH TIMING SUMMARY (Game Manual Table 6-2) ====================
        // AUTO:            20s   (both HUBs active)
        // TRANSITION SHIFT: 10s  (both HUBs active)
        // SHIFT 1-4:       4×25s (alternating active/inactive)
        // END GAME:         30s  (both HUBs active)
        // Total match:      160s (2:40)

        // ==================== SHOOTING TIMING ====================
        /** Max time to wait for shooter lock-on (flywheel + turret + hood at setpoint). */
        public static final double kShooterLockTimeoutSeconds = 1.0;

        /** Time to feed all 8 preloaded FUEL through the shooter (seconds). */
        public static final double kPreloadShootSeconds = 4.0;

        /** Time to feed a collected batch of FUEL through the shooter (seconds). */
        public static final double kCollectedShootSeconds = 3.0;

        /** Max time for vision-assisted ball chase before giving up and continuing auto. */
        public static final double kVisionChaseTimeoutSeconds = 2.0;

        // ==================== AUTO ROBUSTNESS TIMEOUTS ====================
        /** Maximum time for any single ChoreoLib trajectory before aborting. */
        public static final double kMaxTrajectoryTimeoutSeconds = 6.0;
        /** Maximum time for a shoot sequence (wait-for-lock + feed) before continuing. */
        public static final double kMaxShootTimeoutSeconds = 3.0;
        /** Maximum time for vision-assisted ball collection in auto. */
        public static final double kMaxVisionCollectTimeoutSeconds = 4.0;

        // ==================== PATHPLANNER ON-THE-FLY PATHFINDING ====================
        public static final class PathfindingConstants {

            // ---- Path constraints ----
            /** Maximum velocity during pathfinding (m/s). Conservative for auto accuracy. */
            public static final double kMaxVelocity = 3.0;
            /** Maximum acceleration during pathfinding (m/s²). */
            public static final double kMaxAcceleration = 2.5;
            /** Maximum angular velocity during pathfinding (rad/s). */
            public static final double kMaxAngularVelocity = Math.toRadians(540.0);
            /** Maximum angular acceleration during pathfinding (rad/s²). */
            public static final double kMaxAngularAcceleration = Math.toRadians(720.0);

            // ---- Pathfinding timeouts ----
            /** Max time for a pathfind command before it's considered stuck (seconds). */
            public static final double kPathfindTimeoutSeconds = 6.0;
            /** Shorter timeout for nearby movements like shooting range (seconds). */
            public static final double kShortPathfindTimeoutSeconds = 4.0;

            // ---- Trench geometry (Game Manual §5.6) ----
            // Trenches extend from guardrail to bump on both sides of the field.
            // Blue alliance origin: (0,0) is at the blue alliance wall, near-guardrail corner.
            // Field: 16.54m × 8.07m

            // The trench structure sits on both guardrail sides.
            // Near-guardrail trenches (Y close to 0 and Y close to 8.07m)
            /** Trench overall width: 65.65in = 1.668m */
            public static final double kTrenchWidthMeters = Units.inchesToMeters(65.65);
            /** Trench depth (field-lengthwise): 47.0in = 1.194m */
            public static final double kTrenchDepthMeters = Units.inchesToMeters(47.0);
            /** Passable clearance under trench arm: 22.25in = 0.5652m */
            public static final double kTrenchClearanceMeters = Units.inchesToMeters(22.25);
            /** Passable width under trench arm: 50.34in = 1.279m */
            public static final double kTrenchPassableWidthMeters = Units.inchesToMeters(50.34);

            // Trench Y-axis center positions (perpendicular to alliance wall)
            // Near-side trenches sit against the guardrails (Y ≈ 0.6m and Y ≈ 7.47m)
            /** Y-center of the near-guardrail trench (blue-left side). */
            public static final double kTrenchNearGuardrailY = kTrenchWidthMeters / 2.0; // ~0.834m
            /** Y-center of the far-guardrail trench (blue-right side). */
            public static final double kTrenchFarGuardrailY = kFieldWidthMeters - kTrenchWidthMeters / 2.0; // ~7.236m

            // Trench X-axis span: from the bump to the guardrail, across the alliance/neutral boundary.
            // Each alliance has 2 trenches (one per guardrail side). They span from the bump 
            // (which is at ~4.03m from alliance wall) extending the trench depth further into the neutral zone.
            // Blue alliance side: X ≈ kDistanceFromAllianceWall - bumpDepth/2 through the bump region
            // Approximate X-span for blue-side trench underpass center
            /** X-center of the blue alliance trench underpass. */
            public static final double kBlueTrenchCenterX = HubConstants.kDistanceFromAllianceWallMeters; // ~4.03m
            /** X-center of the red alliance trench underpass. */
            public static final double kRedTrenchCenterX = kFieldLengthMeters - HubConstants.kDistanceFromAllianceWallMeters; // ~12.51m

            // ---- Depot positions (Game Manual §5.7) ----
            // Depots are along the alliance wall, 42in wide × 27in deep.
            // Blue depot is on the near-guardrail side of the blue alliance wall.
            /** Blue depot center position (near alliance wall, near-guardrail side). */
            public static final Translation2d kBlueDepotCenter = new Translation2d(
                    Units.inchesToMeters(27.0 / 2.0), // ~0.34m from wall
                    Units.inchesToMeters(42.0 / 2.0)); // ~0.53m from guardrail
            /** Red depot center position (mirrored). */
            public static final Translation2d kRedDepotCenter = new Translation2d(
                    kFieldLengthMeters - Units.inchesToMeters(27.0 / 2.0),
                    kFieldWidthMeters - Units.inchesToMeters(42.0 / 2.0));

            // ---- Key autonomous poses (blue alliance, mirrored for red) ----
            /** Optimal shooting distance from hub center (meters). */
            public static final double kOptimalShootingDistanceMeters = 2.5;
            /** Neutral zone center for collection. */
            public static final Translation2d kNeutralZoneCenter = new Translation2d(
                    kFieldLengthMeters / 2.0, kFieldWidthMeters / 2.0);
            /** Neutral zone near-trench collection point (near-guardrail side). */
            public static final Translation2d kNeutralZoneNearTrench = new Translation2d(
                    kFieldLengthMeters / 2.0, kTrenchNearGuardrailY);
            /** Neutral zone far-trench collection point (far-guardrail side). */
            public static final Translation2d kNeutralZoneFarTrench = new Translation2d(
                    kFieldLengthMeters / 2.0, kTrenchFarGuardrailY);
        }
    }

    // ==================== BALL PRESENCE ESTIMATION (CURRENT-BASED) ====================
    /**
     * Constants for sensorless ball detection using motor current analysis.
     *
     * <p>When a motor runs with balls loaded, friction increases → current rises
     * above the empty-mechanism baseline. The delta reveals ball presence.
     *
     * <p>All thresholds are in AMPS and represent the current ABOVE baseline.
     * Tune these on the real robot by comparing empty vs loaded current readings.
     */
    public static final class BallDetectionConstants {
        // ---- Voltage compensation ----
        // Baseline currents are calibrated at nominal battery voltage.
        // During a match, voltage sag causes motors to draw more current
        // for the same load. We normalize readings back to this voltage.
        public static final double kNominalCalibrationVoltage = 12.0;

        // ---- Sliding window size for current smoothing ----
        // At 50Hz update rate, 10 samples = 200ms averaging window.
        // Larger = more stable, slower response. Smaller = noisier, faster response.
        public static final int kCurrentHistorySize = 10;

        // ---- Default baselines (used before calibration) ----
        // These are rough estimates — calibrate() overwrites them during health check.
        // TODO: measure on real robot with empty hopper
        public static final double kDefaultSpindexerBaselineCurrent = 3.0; // amps when spinning empty
        public static final double kDefaultFeederBaselineCurrent = 2.5;    // amps when spinning empty

        // ---- Spindexer hopper detection thresholds ----
        // Delta above baseline needed to detect balls in hopper.
        // kSpindexerLoadedThreshold: above this → hopper has balls (positive detection)
        // kSpindexerEmptyThreshold:  below this → hopper is empty (negative detection)
        // Gap between them = hysteresis band to prevent flickering.
        // TODO: tune on real robot — load 1 ball, measure delta, set thresholds
        public static final double kSpindexerLoadedThreshold = 2.0;  // amps above baseline
        public static final double kSpindexerEmptyThreshold = 0.5;   // amps above baseline

        // Delta when hopper is completely full (8 preloaded balls).
        // Used to normalize load level to 0.0–1.0 range.
        // TODO: measure with full hopper on real robot
        public static final double kSpindexerFullLoadDelta = 8.0;    // amps above baseline

        // ---- Feeder ball detection threshold ----
        // Current spike when a ball is being actively pushed into flywheels.
        // TODO: tune on real robot — shoot single balls, observe feeder current
        public static final double kFeederBallPresentThreshold = 3.0; // amps above baseline
    }
}
