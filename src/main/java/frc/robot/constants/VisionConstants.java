package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.RobotConfig;

public final class VisionConstants {
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
    public static final String kFrontLeftCamName  = "cam_front_left";
    public static final String kFrontRightCamName = "cam_front_right";
    public static final String kBackLeftCamName   = "cam_back_left";
    public static final String kBackRightCamName  = "cam_back_right";
    public static final String kIntakeCamName     = "cam_intake";    // below slapdown intake, ball detection
    public static final String kSideCamName       = "cam_side";      // side-facing camera

    // ==================== CAMERA FOV ====================
    // Each camera covers 80° horizontal FOV
    public static final double kCameraHFovDegrees = 80.0;

    // ==================== APRILTAG CAMERA TRANSFORMS ====================
    // Each camera has an independently configured 6-DOF pose (x, y, z, roll, pitch, yaw).
    // All values come from RobotConfig — edit there for each robot deployment.
    // Transform3d: Translation3d(x_forward, y_left, z_up) + Rotation3d(roll, pitch, yaw)

    // Front-Left camera
    public static final Transform3d kRobotToFrontLeftCam = new Transform3d(
            new Translation3d(RobotConfig.kFrontLeftCamX, RobotConfig.kFrontLeftCamY, RobotConfig.kFrontLeftCamZ),
            new Rotation3d(RobotConfig.kFrontLeftCamRoll, RobotConfig.kFrontLeftCamPitch, RobotConfig.kFrontLeftCamYaw));

    // Front-Right camera
    public static final Transform3d kRobotToFrontRightCam = new Transform3d(
            new Translation3d(RobotConfig.kFrontRightCamX, RobotConfig.kFrontRightCamY, RobotConfig.kFrontRightCamZ),
            new Rotation3d(RobotConfig.kFrontRightCamRoll, RobotConfig.kFrontRightCamPitch, RobotConfig.kFrontRightCamYaw));

    // Back-Left camera
    public static final Transform3d kRobotToBackLeftCam = new Transform3d(
            new Translation3d(RobotConfig.kBackLeftCamX, RobotConfig.kBackLeftCamY, RobotConfig.kBackLeftCamZ),
            new Rotation3d(RobotConfig.kBackLeftCamRoll, RobotConfig.kBackLeftCamPitch, RobotConfig.kBackLeftCamYaw));

    // Back-Right camera
    public static final Transform3d kRobotToBackRightCam = new Transform3d(
            new Translation3d(RobotConfig.kBackRightCamX, RobotConfig.kBackRightCamY, RobotConfig.kBackRightCamZ),
            new Rotation3d(RobotConfig.kBackRightCamRoll, RobotConfig.kBackRightCamPitch, RobotConfig.kBackRightCamYaw));

    // ==================== NON-APRILTAG CAMERAS ====================

    // Intake camera — tilted downward to see balls on the ground.
    // Does NOT do AprilTag detection — runs object-detection (ML) pipeline.
    public static final Transform3d kRobotToIntakeCam = new Transform3d(
            new Translation3d(RobotConfig.kIntakeCamX, RobotConfig.kIntakeCamY, RobotConfig.kIntakeCamZ),
            new Rotation3d(RobotConfig.kIntakeCamRoll, RobotConfig.kIntakeCamPitch, RobotConfig.kIntakeCamYaw));

    // Side camera — lateral awareness.
    public static final Transform3d kRobotToSideCam = new Transform3d(
            new Translation3d(RobotConfig.kSideCamX, RobotConfig.kSideCamY, RobotConfig.kSideCamZ),
            new Rotation3d(RobotConfig.kSideCamRoll, RobotConfig.kSideCamPitch, RobotConfig.kSideCamYaw));

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
    // Raw values set in RobotConfig.java — edit there for new robot deployments.

    // When we see multiple AprilTags, the estimate is very reliable
    public static final Matrix<N3, N1> kMultiTagStdDevs =
            VecBuilder.fill(RobotConfig.kMultiTagStdDevs[0],
                            RobotConfig.kMultiTagStdDevs[1],
                            RobotConfig.kMultiTagStdDevs[2]);

    // Single tag is less reliable — higher standard deviations
    public static final Matrix<N3, N1> kSingleTagStdDevs =
            VecBuilder.fill(RobotConfig.kSingleTagStdDevs[0],
                            RobotConfig.kSingleTagStdDevs[1],
                            RobotConfig.kSingleTagStdDevs[2]);

    // ==================== FILTERING THRESHOLDS ====================
    // Reject pose estimates that are too far from current pose (likely noise / wrong tag)
    public static final double kMaxPoseAmbiguity     = 0.2;  // reject high-ambiguity single-tag results
    public static final double kMaxPoseJumpMeters    = 2.0;  // reject estimates that jump more than 2m
    public static final double kMaxTagDistanceMeters = 5.5;  // ignore tags further than this

    // ==================== VISION-CORRECTED ODOMETRY ====================
    // When AprilTags are visible, tighten std devs to aggressively correct odometry.
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final Matrix<N3, N1> kAggressiveMultiTagStdDevs =
            VecBuilder.fill(RobotConfig.kAggressiveMultiTagStdDevs[0],
                            RobotConfig.kAggressiveMultiTagStdDevs[1],
                            RobotConfig.kAggressiveMultiTagStdDevs[2]);

    // Maximum acceptable distance discrepancy between odometry-based distance to Hub
    // and vision-measured distance to an AprilTag on the Hub (meters).
    public static final double kMaxDistanceDiscrepancyMeters = 1.0;

    // ==================== VISION INTAKE (BALL CHASE) ====================
    /** PID gains for yaw correction when chasing a ball. */
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kBallChaseYawP = RobotConfig.kBallChaseYawP;
    public static final double kBallChaseYawI = RobotConfig.kBallChaseYawI;
    public static final double kBallChaseYawD = RobotConfig.kBallChaseYawD;
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
