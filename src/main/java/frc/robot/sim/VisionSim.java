package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * PhotonLib vision simulation for all four AprilTag corner cameras.
 *
 * <h2>How it works:</h2>
 * {@link VisionSystemSim} tracks the simulated robot pose (from {@link SwerveDrive})
 * and computes what each camera would see given the field's AprilTag layout.
 * Each frame, it updates the underlying {@link PhotonCamera} objects — the same
 * objects that {@link frc.robot.subsystems.Vision} reads — so all vision filtering,
 * std-dev logic, and odometry fusion work identically in sim and on the real robot.
 *
 * <h2>Field layout:</h2>
 * Uses {@link AprilTagFields#kDefaultField} as a stand-in for the 2026 REBUILT field.
 * Once the official 2026 game field image/layout is released and integrated into WPILib,
 * replace {@code AprilTagFields.kDefaultField} with {@code AprilTagFields.k2026REBUILT}
 * (or whatever the correct enum constant is).
 *
 * <h2>Camera properties:</h2>
 * All four corner cameras use the same resolution and FOV as configured in
 * {@link VisionConstants#kCameraHFovDegrees}. Pixel noise is added to realistically
 * model pose estimation uncertainty — tighten or loosen {@code kAvgErrorPx} to
 * simulate better or worse calibrated cameras.
 */
public class VisionSim {

    // ==================== CAMERA NOISE MODEL ====================
    // Simulates lens distortion and calibration imperfection.
    // Increase kAvgErrorPx to simulate a poorly calibrated camera.
    private static final double kAvgErrorPx    = 0.25; // average pixel error
    private static final double kErrorStdDevPx = 0.08; // standard deviation

    // Camera resolution (pixels). Should match the physical camera setting in PhotonVision UI.
    private static final int kResolutionWidth  = 1280;
    private static final int kResolutionHeight = 800;

    // ==================== STATE ====================

    private final VisionSystemSim visionSystemSim;
    private final PhotonCameraSim[] cameraSims;
    private boolean initialised = false;

    // ==================== CONSTRUCTOR ====================

    public VisionSim() {
        visionSystemSim = new VisionSystemSim("main");

        // Load the field's AprilTag layout.
        // TODO: Replace kDefaultField with the official 2026 REBUILT field layout once
        //       it is available in WPILib (AprilTagFields.k2026Reefscape or equivalent).
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            visionSystemSim.addAprilTags(layout);
        } catch (Exception e) {
            DriverStation.reportWarning(
                    "[VisionSim] Could not load AprilTag field layout: " + e.getMessage(), false);
            layout = null;
        }

        // Create one PhotonCameraSim per AprilTag camera, using the same camera names
        // that Vision.java references so the simulated data flows into the real pipeline.
        cameraSims = new PhotonCameraSim[VisionConstants.kAprilTagCamNames.length];

        for (int i = 0; i < VisionConstants.kAprilTagCamNames.length; i++) {
            SimCameraProperties props = buildCameraProperties();

            PhotonCamera camera = new PhotonCamera(VisionConstants.kAprilTagCamNames[i]);
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);

            // Enable wireframe overlay in the VisionSystemSim debug window for easy debugging.
            cameraSim.enableDrawWireframe(true);

            // Register the camera with the vision system at the robot-relative transform.
            visionSystemSim.addCamera(cameraSim, VisionConstants.kAprilTagCamTransforms[i]);

            cameraSims[i] = cameraSim;
        }

        if (layout != null) {
            initialised = true;
        }
    }

    // ==================== UPDATE ====================

    /**
     * Advances the vision simulation using the current robot pose from odometry.
     * Must be called once per robot periodic loop (from {@link RobotSimulator#update}).
     *
     * <p>After this call, each {@link PhotonCamera} will contain fresh simulated results
     * that {@link frc.robot.subsystems.Vision} will read and fuse with odometry.
     */
    public void update() {
        if (!initialised) return;

        Pose2d robotPose = SwerveDrive.getInstance().getPose();
        visionSystemSim.update(robotPose);

        logTelemetry(robotPose);
    }

    // ==================== CAMERA PROPERTIES ====================

    /**
     * Builds camera intrinsic properties matching the physical cameras.
     * Diagonal FOV is derived from the horizontal FOV in {@link VisionConstants}.
     */
    private SimCameraProperties buildCameraProperties() {
        SimCameraProperties props = new SimCameraProperties();

        props.setCalibration(kResolutionWidth, kResolutionHeight,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(
                        VisionConstants.kCameraHFovDegrees));

        // Add realistic pixel noise to simulate a real (imperfect) camera.
        props.setCalibError(kAvgErrorPx, kErrorStdDevPx);

        // 30 fps — matches the typical PhotonVision pipeline frame rate.
        props.setFPS(30.0);

        // Latency: average 35 ms with 5 ms std dev (typical USB camera pipeline latency).
        props.setAvgLatencyMs(35.0);
        props.setLatencyStdDevMs(5.0);

        return props;
    }

    // ==================== TELEMETRY ====================

    private void logTelemetry(Pose2d robotPose) {
        Logger.recordOutput("Sim/Vision/RobotPoseForSim", robotPose);
        Logger.recordOutput("Sim/Vision/Initialised", initialised);

        // Log active camera count. Avoid reading the result buffer here — Vision.java
        // consumes results via getAllUnreadResults() in its own periodic loop.
        Logger.recordOutput("Sim/Vision/ActiveCameras", cameraSims.length);
    }

    // ==================== DEBUG FIELD ====================

    /**
     * Returns the {@link edu.wpi.first.wpilibj.smartdashboard.Field2d} from the vision system
     * for overlaying camera views on the SmartDashboard field widget.
     *
     * @return Field2d with camera overlays, or {@code null} if not initialised
     */
    public edu.wpi.first.wpilibj.smartdashboard.Field2d getDebugField() {
        return visionSystemSim.getDebugField();
    }
}
