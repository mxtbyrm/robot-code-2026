package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.constants.AutoConstants;
import frc.robot.sim.RobotSimulator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Superstructure;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Einstein-grade Robot class with AdvantageKit logging, data logging,
 * RobotState integration, simulation support, and match lifecycle management.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Created in simulationInit() — null on the real robot.
  private RobotSimulator m_robotSimulator;

  public Robot() {
    // ==================== ADVANTAGEKIT LOGGING ====================
    // Record metadata for log file identification
    Logger.recordMetadata("ProjectName", "Robot2026");
    Logger.recordMetadata("BuildDate", "2026-02-09");

    if (isReal()) {
      // Real robot: log to USB stick + publish to NetworkTables for live dashboards
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Simulation: only publish to NetworkTables (no USB logging needed)
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger — must be called before any Logger.recordOutput()
    Logger.start();

    // Start WPILib data logging — all SmartDashboard + DriverStation data goes to USB
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Initialize RobotState singleton (runs as a subsystem — periodic() auto-called)
    RobotState.getInstance();

    // Initialize HubStateTracker (tracks Active/Inactive HUB shifts)
    HubStateTracker.getInstance();

    // Instantiate RobotContainer — subsystems, bindings, auto chooser
    m_robotContainer = new RobotContainer();

    // Dashboard defaults for auto-win override (pit crew can set before match)
    SmartDashboard.putBoolean("Auto/Use Override", false);
    SmartDashboard.putBoolean("Auto/Won Auto Override", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Reset timing stats for clean match data
    RobotState.getInstance().resetTimingStats();

    // Reset ball shot counter for the new match
    Superstructure.getInstance().resetBallsShotCount();

    // Reset HUB state tracker for autonomous
    HubStateTracker.getInstance().onAutoInit();

    // Deploy intake arm at autonomous start — stays deployed throughout auto.
    // Operator takes over arm control (toggle) in teleop.
    Superstructure.getInstance().deployIntakeArm();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Reset timing stats at teleop start
    RobotState.getInstance().resetTimingStats();

    // Determine if we won Autonomous for HUB shift pattern.
    // PRIMARY: FMS Game Data ('R'/'B') is read automatically by HubStateTracker.
    // The data arrives ~3s after Auto ends via DriverStation.getGameSpecificMessage().
    // FALLBACK: if FMS is attached but Game Data hasn't arrived yet, use
    // dashboard override or heuristic based on auto ball count.
    // IN PRACTICE (no FMS): skip entirely — HUB stays ACTIVE so drivers
    // and operators can always shoot without shift restrictions.
    if (DriverStation.isFMSAttached() && !HubStateTracker.getInstance().isGameDataReceived()) {
      boolean dashboardEnabled = SmartDashboard.getBoolean("Auto/Use Override", false);
      boolean dashboardOverride = SmartDashboard.getBoolean("Auto/Won Auto Override", false);

      boolean wonAuto;
      if (dashboardEnabled) {
        wonAuto = dashboardOverride;
      } else {
        // Heuristic: if we shot ≥6 balls in auto, assume we won
        int autoShots = Superstructure.getInstance().getBallsShotCount();
        wonAuto = autoShots >= 6;
        SmartDashboard.putNumber("Auto/Balls Shot In Auto", autoShots);
      }

      HubStateTracker.getInstance().setWonAutonomous(wonAuto);
      SmartDashboard.putBoolean("Auto/Won Auto (resolved)", wonAuto);
    }

    // Start HUB Alliance Shift tracking
    HubStateTracker.getInstance().onTeleopInit();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    // All subsystem singletons are already initialised by RobotContainer at this point.

    // Place robot at a valid blue-alliance starting position so VisionSim detects
    // AprilTags from a realistic position and doesn't fuse wildly wrong pose estimates.
    // 1.5m from the blue wall, centered in Y on the field.
    SwerveDrive.getInstance().resetPose(
        new Pose2d(1.5, AutoConstants.kFieldWidthMeters / 2.0, new Rotation2d()));

    m_robotSimulator = new RobotSimulator();
  }

  @Override
  public void simulationPeriodic() {
    if (m_robotSimulator != null) {
      m_robotSimulator.update(getPeriod());
    }
  }
}
