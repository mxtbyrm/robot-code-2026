package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

/**
 * Multi-camera vision subsystem using PhotonVision for AprilTag-based pose estimation
 * and object detection.
 * 
 * Camera layout (6 cameras total):
 * - 4 AprilTag cameras at robot corners, each at 45° angle, 80° FOV, slightly tilted up
 *   → Combined 360° AprilTag coverage with overlap at each quadrant
 * - 1 intake camera under the slapdown intake, facing downward for ball/object detection
 * - 1 side camera for additional situational awareness (trench navigation, etc.)
 * 
 * AprilTag pose estimates are fused into the drivetrain's SwerveDrivePoseEstimator
 * via a consumer callback. Standard deviations are dynamically adjusted based on
 * tag count and average distance.
 */
public class Vision extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Vision instance;

    public static void initialize(VisionMeasurementConsumer measurementConsumer,
                                  java.util.function.Supplier<Pose2d> currentPoseSupplier) {
        if (instance != null) throw new IllegalStateException("Vision already initialized.");
        instance = new Vision(measurementConsumer, currentPoseSupplier);
    }

    public static Vision getInstance() {
        if (instance == null) throw new IllegalStateException("Vision not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== FUNCTIONAL INTERFACE ====================
    /**
     * Consumer for vision pose estimates. The drivetrain passes its
     * addVisionMeasurement method matching this signature.
     */
    @FunctionalInterface
    public static interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    // ==================== APRILTAG CAMERAS ====================
    private final PhotonCamera[] aprilTagCameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private final Matrix<N3, N1>[] curStdDevs;

    // ==================== NON-APRILTAG CAMERAS ====================
    private final PhotonCamera intakeCamera;
    private final PhotonCamera sideCamera;

    // ==================== CONSUMER ====================
    private final VisionMeasurementConsumer measurementConsumer;

    // ==================== POSE-JUMP REJECTION ====================
    private final java.util.function.Supplier<Pose2d> currentPoseSupplier;
    private int poseJumpRejectionCount = 0;

    // ==================== TELEMETRY ====================
    private final StructPublisher<Pose2d>[] visionPosePublishers;

    // ==================== CAMERA RECONNECTION ====================
    private final double[] lastReconnectAttempt;
    private static final double kReconnectCooldownSeconds = 2.0;

    // ==================== PER-CAMERA TELEMETRY ====================
    private final int[] perCamTotalEstimates;
    private final int[] perCamAccepted;
    private final int[] perCamRejected;

    // ==================== TIMESTAMP VALIDATION ====================
    private int staleTimestampRejections = 0;
    private static final double kMaxTimestampAge = 0.5;

    // ==================== OBJECT DETECTION STATE ====================
    private boolean ballDetected = false;
    private double ballYaw = 0.0;
    private double ballPitch = 0.0;
    private double ballArea = 0.0;

    // ==================== HUB APRILTAG TRACKING ====================
    /** Number of HUB AprilTags currently visible across all cameras */
    private int visibleHubTagCount = 0;
    /** True if any HUB AprilTag from our alliance is currently visible */
    private boolean ownHubVisible = false;

    // ==================== VISION-CORRECTED ODOMETRY ====================
    /** True if any AprilTag was seen this cycle (any camera). */
    private boolean anyTagSeenThisCycle = false;
    /** Distance to closest HUB tag measured directly via camera (meters). -1 if none visible. */
    private double directHubDistanceMeters = -1.0;
    /** Distance discrepancy between odometry and vision (meters). 0 if no comparison available. */
    private double distanceDiscrepancy = 0.0;

    /**
     * Creates the vision subsystem with all 6 cameras.
     *
     * @param measurementConsumer A function that accepts (Pose2d, timestamp, stdDevs)
     *                           — typically swerveDrive::addVisionMeasurement
     * @param currentPoseSupplier Supplies the current odometry pose for pose-jump rejection
     */
    @SuppressWarnings("unchecked")
    private Vision(VisionMeasurementConsumer measurementConsumer,
                   java.util.function.Supplier<Pose2d> currentPoseSupplier) {
        this.measurementConsumer = measurementConsumer;
        this.currentPoseSupplier = currentPoseSupplier;

        // ==================== INITIALIZE APRILTAG CAMERAS ====================
        int numCams = VisionConstants.kAprilTagCamNames.length;
        aprilTagCameras = new PhotonCamera[numCams];
        poseEstimators = new PhotonPoseEstimator[numCams];
        curStdDevs = new Matrix[numCams];
        visionPosePublishers = new StructPublisher[numCams];
        lastReconnectAttempt = new double[numCams];
        perCamTotalEstimates = new int[numCams];
        perCamAccepted = new int[numCams];
        perCamRejected = new int[numCams];

        for (int i = 0; i < numCams; i++) {
            String camName = VisionConstants.kAprilTagCamNames[i];
            Transform3d robotToCam = VisionConstants.kAprilTagCamTransforms[i];

            aprilTagCameras[i] = new PhotonCamera(camName);
            poseEstimators[i] = new PhotonPoseEstimator(
                    VisionConstants.kTagLayout,
                    robotToCam);
            curStdDevs[i] = VisionConstants.kSingleTagStdDevs;

            // NT publisher for each camera's pose estimate (for AdvantageScope visualization)
            visionPosePublishers[i] = NetworkTableInstance.getDefault()
                    .getStructTopic("Vision/" + camName + "/EstPose", Pose2d.struct)
                    .publish();
        }

        // ==================== INITIALIZE NON-APRILTAG CAMERAS ====================
        intakeCamera = new PhotonCamera(VisionConstants.kIntakeCamName);
        sideCamera = new PhotonCamera(VisionConstants.kSideCamName);
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // Reset tracking for this cycle
        visibleHubTagCount = 0;
        ownHubVisible = false;
        anyTagSeenThisCycle = false;
        directHubDistanceMeters = -1.0;
        distanceDiscrepancy = 0.0;

        // Attempt reconnection of disconnected cameras
        attemptCameraReconnection();

        // Process all AprilTag cameras
        processAprilTagCameras();

        // Process intake camera for ball detection
        processIntakeCamera();

        // Health: at least 1 AprilTag camera connected
        RobotState.getInstance().setVisionHealthy(getConnectedAprilTagCameraCount() >= 1);

        // Publish telemetry
        publishTelemetry();
    }

    // ==================== CAMERA RECONNECTION ====================

    /**
     * Attempts to reconnect any disconnected AprilTag cameras.
     * Only tries once every kReconnectCooldownSeconds per camera.
     */
    private void attemptCameraReconnection() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        for (int i = 0; i < aprilTagCameras.length; i++) {
            if (!aprilTagCameras[i].isConnected()
                    && (now - lastReconnectAttempt[i]) > kReconnectCooldownSeconds) {
                lastReconnectAttempt[i] = now;
                aprilTagCameras[i].setDriverMode(false); // trigger reconnect attempt
                Logger.recordOutput("Vision/" + VisionConstants.kAprilTagCamNames[i] + "/ReconnectAttempt", true);
            }
        }
    }

    // ==================== APRILTAG PROCESSING ====================

    /**
     * Processes all 4 AprilTag cameras, estimates robot pose from each,
     * and feeds valid estimates into the drivetrain pose estimator.
     */
    private void processAprilTagCameras() {
        for (int i = 0; i < aprilTagCameras.length; i++) {
            processOneAprilTagCamera(i);
        }
    }

    /**
     * Processes a single AprilTag camera: reads all unread results,
     * estimates pose, filters bad estimates, and sends to the consumer.
     */
    private void processOneAprilTagCamera(int camIndex) {
        PhotonCamera camera = aprilTagCameras[camIndex];
        PhotonPoseEstimator estimator = poseEstimators[camIndex];

        for (var result : camera.getAllUnreadResults()) {
            // Skip if no targets
            if (!result.hasTargets()) continue;

            // Before single-tag fallback, check ambiguity
            if (shouldRejectSingleTagResult(result)) continue;

            // Try coprocessor multi-tag first, fall back to lowest-ambiguity single-tag
            Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEst.isEmpty()) continue;

            EstimatedRobotPose est = visionEst.get();
            perCamTotalEstimates[camIndex]++;

            // Timestamp validation: reject stale or future timestamps
            double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            if (Math.abs(now - est.timestampSeconds) > kMaxTimestampAge) {
                staleTimestampRejections++;
                perCamRejected[camIndex]++;
                Logger.recordOutput("Vision/" + VisionConstants.kAprilTagCamNames[camIndex] + "/StaleTimestamp", true);
                continue;
            }

            // Validate the estimate
            if (!isEstimateValid(est)) {
                perCamRejected[camIndex]++;
                continue;
            }

            // Pose-jump rejection: reject if estimate is too far from current odometry pose
            Pose2d currentPose = currentPoseSupplier.get();
            double jump = est.estimatedPose.toPose2d().getTranslation()
                    .getDistance(currentPose.getTranslation());
            if (jump > VisionConstants.kMaxPoseJumpMeters) {
                poseJumpRejectionCount++;
                perCamRejected[camIndex]++;
                continue;
            }

            // Update standard deviations based on tag count and distance
            updateEstimationStdDevs(camIndex, visionEst, result.getTargets());

            // Track HUB AprilTags for shooter awareness
            for (var tgt : result.getTargets()) {
                int tagId = tgt.getFiducialId();
                if (HubConstants.isHubTag(tagId)) {
                    visibleHubTagCount++;
                    if (HubConstants.isOwnHubTag(tagId)) {
                        ownHubVisible = true;
                        // Measure direct distance to HUB tag from camera
                        // Use the best target's transform for distance
                        double tagDist = tgt.getBestCameraToTarget().getTranslation().getNorm();
                        if (directHubDistanceMeters < 0 || tagDist < directHubDistanceMeters) {
                            directHubDistanceMeters = tagDist;
                        }
                    }
                }
            }

            // Mark that we saw at least one tag this cycle
            anyTagSeenThisCycle = true;

            // Publish pose for this camera
            visionPosePublishers[camIndex].set(est.estimatedPose.toPose2d());

            // Track accepted estimate
            perCamAccepted[camIndex]++;

            // Send to drivetrain pose estimator
            measurementConsumer.accept(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    curStdDevs[camIndex]);
        }
    }

    /**
     * Checks if a single-tag result should be rejected due to high ambiguity.
     */
    private boolean shouldRejectSingleTagResult(PhotonPipelineResult result) {
        if (result.getTargets().size() != 1) return false;
        PhotonTrackedTarget target = result.getBestTarget();
        return target.getPoseAmbiguity() > VisionConstants.kMaxPoseAmbiguity;
    }

    /**
     * Validates an estimated pose — rejects estimates that are clearly wrong.
     */
    private boolean isEstimateValid(EstimatedRobotPose est) {
        Pose3d pose = est.estimatedPose;

        // Reject if pose is below the floor or unreasonably high
        if (pose.getZ() < -0.5 || pose.getZ() > 1.5) return false;

        // Reject if pose is outside the field boundaries (with margin)
        double margin = VisionConstants.kFieldBoundaryMargin;
        double x = pose.getX();
        double y = pose.getY();
        if (x < -margin || x > AutoConstants.kFieldLengthMeters + margin
                || y < -margin || y > AutoConstants.kFieldWidthMeters + margin) return false;

        return true;
    }

    /**
     * Dynamically adjusts standard deviations based on number of visible tags
     * and their average distance. More tags + closer = lower std devs = more trust.
     */
    private void updateEstimationStdDevs(
            int camIndex,
            Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            curStdDevs[camIndex] = VisionConstants.kSingleTagStdDevs;
            return;
        }

        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Calculate number of known tags and average distance
        for (var tgt : targets) {
            var tagPose = poseEstimators[camIndex].getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs[camIndex] = VisionConstants.kSingleTagStdDevs;
            return;
        }

        avgDist /= numTags;

        // Use tighter std devs when multiple tags are visible
        if (numTags > 1) {
            // Aggressive vision correction: when multiple tags are visible,
            // heavily trust vision to snap odometry back to correct position.
            // This corrects any drift that accumulated while tags weren't visible.
            estStdDevs = VisionConstants.kAggressiveMultiTagStdDevs;
        }

        // If single tag and too far away, reject entirely
        if (numTags == 1 && avgDist > VisionConstants.kMaxTagDistanceMeters) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            // Scale std devs up with distance squared (further = less trust)
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30.0));
        }

        curStdDevs[camIndex] = estStdDevs;
    }

    // ==================== INTAKE CAMERA (OBJECT DETECTION) ====================

    /**
     * Processes the intake camera for ball detection using PhotonVision's
     * object detection pipeline. The intake camera should be configured
     * with an ML object detection pipeline in the PhotonVision UI.
     */
    private void processIntakeCamera() {
        var results = intakeCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            ballDetected = false;
            return;
        }
        // Use the most recent result
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            var bestTarget = result.getBestTarget();
            ballDetected = true;
            ballYaw = bestTarget.getYaw();
            ballPitch = bestTarget.getPitch();
            ballArea = bestTarget.getArea();
        } else {
            ballDetected = false;
            ballYaw = 0.0;
            ballPitch = 0.0;
            ballArea = 0.0;
        }
    }

    // ==================== INTAKE CAMERA GETTERS ====================

    /** @return true if the intake camera currently sees a ball */
    public boolean isBallDetected() {
        return ballDetected;
    }

    /** @return yaw angle to detected ball (positive = left), degrees */
    public double getBallYaw() {
        return ballYaw;
    }

    /** @return pitch angle to detected ball (positive = up), degrees */
    public double getBallPitch() {
        return ballPitch;
    }

    /** @return area of detected ball as percent of frame (0-100) — larger = closer */
    public double getBallArea() {
        return ballArea;
    }

    // ==================== SIDE CAMERA ====================

    /**
     * Gets the latest result from the side camera.
     * Can be used for trench detection, obstacle avoidance, or game piece spotting.
     * The specific pipeline (AprilTag, color, ML) should be configured in the PhotonVision UI.
     */
    public PhotonPipelineResult getSideCameraResult() {
        var results = sideCamera.getAllUnreadResults();
        if (results.isEmpty()) return new PhotonPipelineResult();
        return results.get(results.size() - 1);
    }

    /** @return true if the side camera sees any target */
    public boolean sideHasTargets() {
        return getSideCameraResult().hasTargets();
    }

    // ==================== CAMERA STATUS ====================
    /**
     * Gets the latest raw result from the intake camera.
     * Can be used to detect large objects (robots) ahead for obstacle avoidance.
     * Note: For ball-specific data, prefer {@link #isBallDetected()}, {@link #getBallYaw()}, etc.
     */
    public PhotonPipelineResult getIntakeCameraResult() {
        var results = intakeCamera.getAllUnreadResults();
        if (results.isEmpty()) return new PhotonPipelineResult();
        return results.get(results.size() - 1);
    }

    // ==================== CAMERA STATUS ====================

    /**
     * @return true if at least one AprilTag camera is connected and reporting
     */
    public boolean isAnyAprilTagCameraConnected() {
        for (PhotonCamera cam : aprilTagCameras) {
            if (cam.isConnected()) return true;
        }
        return false;
    }

    /**
     * @return number of AprilTag cameras currently connected
     */
    public int getConnectedAprilTagCameraCount() {
        int count = 0;
        for (PhotonCamera cam : aprilTagCameras) {
            if (cam.isConnected()) count++;
        }
        return count;
    }

    /**
     * @return true if the intake camera is connected
     */
    public boolean isIntakeCameraConnected() {
        return intakeCamera.isConnected();
    }

    // ==================== HUB APRILTAG GETTERS ====================

    /** @return number of HUB AprilTags visible across all cameras this cycle */
    public int getVisibleHubTagCount() {
        return visibleHubTagCount;
    }

    /** @return true if any AprilTag on our alliance's HUB is currently visible */
    public boolean isOwnHubVisible() {
        return ownHubVisible;
    }

    /** @return true if any AprilTag was seen this cycle from any camera */
    public boolean isAnyTagSeen() {
        return anyTagSeenThisCycle;
    }

    /**
     * @return direct camera-measured distance to closest own HUB AprilTag, or -1 if none visible.
     *         Use for cross-checking odometry-based distance.
     */
    public double getDirectHubDistanceMeters() {
        return directHubDistanceMeters;
    }

    /**
     * Validates the odometry-based distance to Hub against the vision-measured distance.
     * Call from Superstructure or shooter periodic to detect odometry drift.
     *
     * @param odometryDistance Distance to Hub computed from odometry pose (meters).
     * @return true if distances agree (or no vision data available), false if discrepancy detected.
     */
    public boolean validateDistance(double odometryDistance) {
        if (directHubDistanceMeters < 0) {
            // No vision measurement available — can't validate
            distanceDiscrepancy = 0.0;
            return true;
        }
        distanceDiscrepancy = Math.abs(odometryDistance - directHubDistanceMeters);
        boolean valid = distanceDiscrepancy < VisionConstants.kMaxDistanceDiscrepancyMeters;
        if (!valid) {
            DriverStation.reportWarning(
                    "[Vision] Distance discrepancy: odom=" + String.format("%.2f", odometryDistance)
                            + "m, vision=" + String.format("%.2f", directHubDistanceMeters)
                            + "m, delta=" + String.format("%.2f", distanceDiscrepancy) + "m", false);
        }
        return valid;
    }

    // ==================== TELEMETRY ====================

    private void publishTelemetry() {
        // Telemetry (AdvantageKit only)
        Logger.recordOutput("Vision/ConnectedATCameras", getConnectedAprilTagCameraCount());
        Logger.recordOutput("Vision/IntakeCamConnected", isIntakeCameraConnected());
        Logger.recordOutput("Vision/BallDetected", ballDetected);
        Logger.recordOutput("Vision/BallYaw", ballYaw);
        Logger.recordOutput("Vision/BallArea", ballArea);
        Logger.recordOutput("Vision/PoseJumpRejections", poseJumpRejectionCount);
        Logger.recordOutput("Vision/StaleTimestampRejections", staleTimestampRejections);
        Logger.recordOutput("Vision/VisibleHubTags", visibleHubTagCount);
        Logger.recordOutput("Vision/OwnHubVisible", ownHubVisible);
        Logger.recordOutput("Vision/AnyTagSeen", anyTagSeenThisCycle);
        Logger.recordOutput("Vision/DirectHubDistance", directHubDistanceMeters);
        Logger.recordOutput("Vision/DistanceDiscrepancy", distanceDiscrepancy);

        // Per-camera connection status and accept rate
        for (int i = 0; i < aprilTagCameras.length; i++) {
            String camName = VisionConstants.kAprilTagCamNames[i];
            Logger.recordOutput("Vision/" + camName + "/Connected", aprilTagCameras[i].isConnected());
            Logger.recordOutput("Vision/" + camName + "/TotalEstimates", perCamTotalEstimates[i]);
            Logger.recordOutput("Vision/" + camName + "/Accepted", perCamAccepted[i]);
            Logger.recordOutput("Vision/" + camName + "/Rejected", perCamRejected[i]);
            double acceptRate = perCamTotalEstimates[i] > 0
                    ? (double) perCamAccepted[i] / perCamTotalEstimates[i]
                    : 0.0;
            Logger.recordOutput("Vision/" + camName + "/AcceptRate", acceptRate);
        }
    }
}
