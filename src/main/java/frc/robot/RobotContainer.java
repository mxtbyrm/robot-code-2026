// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.OnTheFlyAutos;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.VisionIntakeCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * Einstein-grade RobotContainer with:
 * <ul>
 *   <li>Dual-controller setup (driver + operator)</li>
 *   <li>Superstructure state machine for mechanism coordination</li>
 *   <li>LED feedback for driver communication</li>
 *   <li>Heading lock, slow mode, snap-to-angle</li>
 *   <li>Alliance-aware autonomous</li>
 *   <li>Pre-match system check</li>
 * </ul>
 */
public class RobotContainer {
    // ==================== SUBSYSTEMS (initialized as singletons) ====================

    // ==================== CONTROLLERS ====================
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController operatorController =
            new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // ==================== COMMANDS ====================
    private final SwerveDriveCommand driveCommand;

    // ==================== AUTONOMOUS ====================
    private final AutoFactory autoFactory;
    private final Autos autos;
    private final OnTheFlyAutos onTheFlyAutos;
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // ==================== SUBSYSTEM INITIALIZATION (singleton order matters) ====================
        SwerveDrive.initialize();
        Intake.initialize();
        Spindexer.initialize();
        Shooter.initialize(SwerveDrive.getInstance()::getPose, SwerveDrive.getInstance()::getFieldRelativeSpeeds);
        Feeder.initialize(Shooter.getInstance()::isReadyToShoot, Shooter.getInstance()::getTargetFlywheelRPS);
        Vision.initialize(SwerveDrive.getInstance()::addVisionMeasurement, SwerveDrive.getInstance()::getPose);
        Superstructure.initialize();
        LEDs.initialize();

        // ==================== CHOREO AUTO FACTORY ====================
        // Alliance-aware: ChoreoLib automatically mirrors trajectories for Red alliance
        autoFactory = new AutoFactory(
                SwerveDrive.getInstance()::getPose,
                SwerveDrive.getInstance()::resetPose,
                SwerveDrive.getInstance()::followTrajectory,
                true, // enable alliance flipping
                SwerveDrive.getInstance()
        );

        // ==================== AUTOS ====================
        autos = new Autos(SwerveDrive.getInstance(), Shooter.getInstance(), Feeder.getInstance(),
                Intake.getInstance(), Spindexer.getInstance(), Superstructure.getInstance(),
                Vision.getInstance(), autoFactory,
                () -> VisionIntakeCommand.forAuto(SwerveDrive.getInstance(), Vision.getInstance()));

        // ==================== ON-THE-FLY AUTOS (PathPlanner pathfinding) ====================
        onTheFlyAutos = new OnTheFlyAutos(SwerveDrive.getInstance(), Shooter.getInstance(),
                Superstructure.getInstance(),
                () -> VisionIntakeCommand.forAuto(SwerveDrive.getInstance(), Vision.getInstance()));

        // ==================== AUTO CHOOSER ====================
        // Combined chooser: PathPlanner routines first, Choreo fallbacks below
        autoChooser = new SendableChooser<>();

        // Default = safe fallback
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // ---- PathPlanner on-the-fly routines (primary) ----
        autoChooser.addOption("[PP] Preload & Park",           onTheFlyAutos.preloadAndPark());
        autoChooser.addOption("[PP] Preload & Collect Center", onTheFlyAutos.preloadAndCollectCenter());
        autoChooser.addOption("[PP] Preload & Collect Side",   onTheFlyAutos.preloadAndCollectSide());
        autoChooser.addOption("[PP] Preload & Collect Depot",  onTheFlyAutos.preloadAndCollectDepot());
        autoChooser.addOption("[PP] Double Trench Score",      onTheFlyAutos.doubleTrenchScore());
        autoChooser.addOption("[PP] Aggressive Sweep",         onTheFlyAutos.aggressiveSweep());
        autoChooser.addOption("[PP] Defensive Preload",        onTheFlyAutos.defensivePreload());

        // ---- Choreo fallback routines (deprecated but kept for safety) ----
        autoChooser.addOption("[Choreo] Preload & Park",       autos.preloadAndPark().cmd());
        autoChooser.addOption("[Choreo] Center 2-Cycle",       autos.centerTwoCycle().cmd());
        autoChooser.addOption("[Choreo] Left 2-Cycle",         autos.leftTwoCycle().cmd());
        autoChooser.addOption("[Choreo] Right 2-Cycle",        autos.rightTwoCycle().cmd());
        autoChooser.addOption("[Choreo] Center 3-Cycle",       autos.centerThreeCycle().cmd());
        autoChooser.addOption("[Choreo] Shoot While Moving",   autos.shootWhileMoving().cmd());
        autoChooser.addOption("[Choreo] Depot Cycle",          autos.depotCycle().cmd());

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // ==================== DEFAULT COMMAND (with heading lock + slow mode) ====================
        driveCommand = new SwerveDriveCommand(
                SwerveDrive.getInstance(),
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> driverController.getRightTriggerAxis() > 0.3, // right trigger = slow mode
                Superstructure.getInstance()::isIntaking // cap chassis speed below roller speed
        );
        driveCommand.setShooterReference(Shooter.getInstance()); // turret wraparound assist
        SwerveDrive.getInstance().setDefaultCommand(driveCommand);


        // Configure button bindings
        configureDriverBindings();
        configureOperatorBindings();

        // ==================== PRE-MATCH SYSTEM CHECK (visible on dashboard) ====================
        SmartDashboard.putData("System Check", systemCheckCommand());
    }

    /**
     * Pre-match system check command. Run from dashboard before every match.
     * Actuates each subsystem and validates responses — reports pass/fail per system.
     */
    public Command systemCheckCommand() {
        // Track per-subsystem results
        final boolean[] results = new boolean[6]; // swerve, intake, spindexer, feeder, shooter, vision
        final double[] baselines = new double[2]; // [0] = spindexer empty current, [1] = feeder empty current

        return Commands.sequence(
                Commands.print("[SYSTEM CHECK] Starting..."),

                // ---- Swerve: drive briefly, verify modules still healthy ----
                Commands.runOnce(() -> SwerveDrive.getInstance().drive(0.1, 0, 0, false), SwerveDrive.getInstance()),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> {
                    SwerveDrive.getInstance().stop();
                    results[0] = RobotState.getInstance().isAllHealthy(); // checks swerve health flag
                    SmartDashboard.putBoolean("SystemCheck/Swerve", results[0]);
                    if (!results[0]) CommandScheduler.getInstance().schedule(Commands.print("[SYSTEM CHECK] FAIL: Swerve"));
                }, SwerveDrive.getInstance()),

                // ---- Intake: deploy, verify position reached ----
                Commands.runOnce(() -> Intake.getInstance().deploy(), Intake.getInstance()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    results[1] = Intake.getInstance().isDeployed();
                    SmartDashboard.putBoolean("SystemCheck/Intake Deploy", results[1]);
                    if (!results[1]) CommandScheduler.getInstance().schedule(Commands.print("[SYSTEM CHECK] FAIL: Intake did not reach deploy position"));
                    Intake.getInstance().stow();
                }, Intake.getInstance()),
                Commands.waitSeconds(0.5),

                // ---- Spindexer: run briefly, verify not jammed, record baseline current ----
                Commands.runOnce(() -> Spindexer.getInstance().runIntakeSpeed(), Spindexer.getInstance()),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> {
                    // Record empty spindexer current as baseline for ball detection
                    baselines[0] = Spindexer.getInstance().getCurrent();
                    results[2] = RobotState.getInstance().isAllHealthy();
                    SmartDashboard.putBoolean("SystemCheck/Spindexer", results[2]);
                    SmartDashboard.putNumber("SystemCheck/Spindexer Baseline A", baselines[0]);
                    Spindexer.getInstance().stop();
                }, Spindexer.getInstance()),

                // ---- Feeder: run briefly, record baseline, calibrate ball estimator ----
                Commands.runOnce(() -> Feeder.getInstance().requestFeed(), Feeder.getInstance()),
                Commands.waitSeconds(0.15),
                Commands.runOnce(() -> {
                    baselines[1] = Feeder.getInstance().getCurrent();
                    results[3] = RobotState.getInstance().isAllHealthy();
                    SmartDashboard.putBoolean("SystemCheck/Feeder", results[3]);
                    SmartDashboard.putNumber("SystemCheck/Feeder Baseline A", baselines[1]);
                    Feeder.getInstance().cancelFeed();

                    // Calibrate ball presence estimator with both empty baselines
                    BallPresenceEstimator.getInstance().calibrate(baselines[0], baselines[1]);
                }, Feeder.getInstance()),

                // ---- Shooter: spin flywheel briefly, verify healthy ----
                Commands.runOnce(() -> Shooter.getInstance().enableTracking(), Shooter.getInstance()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    results[4] = Shooter.getInstance().isHealthy();
                    SmartDashboard.putBoolean("SystemCheck/Shooter", results[4]);
                    if (!results[4]) CommandScheduler.getInstance().schedule(Commands.print("[SYSTEM CHECK] FAIL: Shooter"));
                    Shooter.getInstance().disableTracking();
                }, Shooter.getInstance()),

                // ---- Vision: check camera connectivity ----
                Commands.runOnce(() -> {
                    int camCount = Vision.getInstance().getConnectedAprilTagCameraCount();
                    results[5] = camCount >= 1;
                    SmartDashboard.putNumber("SystemCheck/Vision Cameras", camCount);
                    SmartDashboard.putBoolean("SystemCheck/Vision", results[5]);
                    if (!results[5]) CommandScheduler.getInstance().schedule(Commands.print("[SYSTEM CHECK] FAIL: No AprilTag cameras connected"));
                }),

                // ---- Summary ----
                Commands.runOnce(() -> {
                    boolean allPassed = results[0] && results[1] && results[2] && results[3] && results[4] && results[5];
                    SmartDashboard.putBoolean("SystemCheck/ALL PASSED", allPassed);
                    String status = allPassed ? "ALL PASSED" : "FAILURES DETECTED — check dashboard";
                    System.out.println("[SYSTEM CHECK] " + status);
                }),
                Commands.print("[SYSTEM CHECK] Complete.")
        ).withName("System Check");
    }

    // ==================== DRIVER BINDINGS ====================
    //
    // Driver responsibilities: DRIVING ONLY
    // The driver focuses purely on field navigation and positioning.
    //
    // Layout:
    //   Left stick     → Translation (X/Y)
    //   Right stick    → Rotation
    //   Right trigger  → SLOW MODE (hold for fine positioning)
    //   X button       → X-LOCK wheels
    //   Y button       → VISION BALL CHASE (auto-drives toward ball + runs intake rollers)
    //   START          → Zero gyro
    //   D-pad          → Snap-to-angle (0°, 90°, 180°, 270°)

    private void configureDriverBindings() {
        // Zero gyro — press START to reset field-relative forward direction
        driverController.start().onTrue(
                Commands.runOnce(() -> SwerveDrive.getInstance().zeroGyro()).ignoringDisable(true));

        // X-lock wheels — hold X to lock wheels in X formation (defense)
        driverController.x().whileTrue(
                Commands.run(() -> SwerveDrive.getInstance().lockWheels(), SwerveDrive.getInstance()));

        // ==================== SNAP-TO-ANGLE (D-PAD) ====================
        // Snap robot heading to cardinal directions for precise alignment
        driverController.povUp().onTrue(
                Commands.runOnce(() -> driveCommand.snapToAngle(0)));      // Face away from DS
        driverController.povRight().onTrue(
                Commands.runOnce(() -> driveCommand.snapToAngle(-90)));    // Face right
        driverController.povDown().onTrue(
                Commands.runOnce(() -> driveCommand.snapToAngle(180)));    // Face DS
        driverController.povLeft().onTrue(
                Commands.runOnce(() -> driveCommand.snapToAngle(90)));     // Face left

        // Y button: Vision-assisted ball chase (while holding, auto-drives to nearest ball + runs rollers)
        // Operator should deploy intake arm first via A toggle. This only controls chassis + rollers.
        driverController.y().whileTrue(
                Commands.parallel(
                        new VisionIntakeCommand(SwerveDrive.getInstance(), Vision.getInstance()),
                        Superstructure.getInstance().intakeRollerCommand()
                )
        );
    }

    // ==================== OPERATOR BINDINGS ====================
    //
    // Operator responsibilities: ALL MECHANISM CONTROL
    // The operator manages intake, shooting, outtake, unjam, and special modes.
    //
    // Layout:
    //   Right trigger  → SHOOT (Hub tracking + feed)
    //   Left trigger   → INTAKE ROLLERS (hold to spin rollers + spindexer)
    //   A button       → INTAKE ARM TOGGLE (press to deploy, press again to stow)
    //   Right bumper   → ALLIANCE ZONE DUMP (fixed lob shot)
    //   Left bumper    → OUTTAKE (eject balls)
    //   B button       → UNJAM (reverse everything)
    //   START          → Emergency stop all mechanisms
    //   BACK           → Re-enable after emergency stop

    private void configureOperatorBindings() {
        // ==================== PRIMARY ACTIONS ====================

        // Right trigger: SHOOT — hold to enable Hub tracking + feed.
        // Turret handles aiming independently; drivetrain stays under driver control.
        // On release: tracking off, feeder stops.
        operatorController.rightTrigger(0.3).whileTrue(
                Superstructure.getInstance().shootCommand()
                        .alongWith(Commands.startEnd(
                                () -> Shooter.getInstance().enableTracking(),
                                () -> Shooter.getInstance().disableTracking()
                        ))
        );

        // A button: INTAKE ARM TOGGLE — press to deploy, press again to stow.
        // Arm stays in position until toggled again, independent of rollers.
        operatorController.a().onTrue(
                Commands.runOnce(() -> Superstructure.getInstance().toggleIntakeArm()));

        // Left trigger: INTAKE ROLLERS — hold to spin intake roller + spindexer.
        // Arm must be deployed separately via A button first.
        operatorController.leftTrigger(0.3).whileTrue(Superstructure.getInstance().intakeRollerCommand());

        // Right bumper: ALLIANCE ZONE DUMP — hold to lob FUEL from neutral zone
        // into alliance zone with fixed shooter parameters (no auto-aim).
        // On release: alliance zone mode off, back to idle.
        operatorController.rightBumper().whileTrue(
                Superstructure.getInstance().shootAllianceCommand()
                        .alongWith(Commands.startEnd(
                                () -> Shooter.getInstance().enableAllianceZoneMode(),
                                () -> Shooter.getInstance().disableAllianceZoneMode()
                        ))
        );

        // Left bumper: OUTTAKE — hold to deploy arm + reverse rollers to eject balls.
        operatorController.leftBumper().whileTrue(Superstructure.getInstance().outtakeCommand());

        // B button: UNJAM — hold to reverse everything to clear a jam.
        operatorController.b().whileTrue(Superstructure.getInstance().unjamCommand());

        // ==================== EMERGENCY CONTROLS ====================
        // START: emergency stop all mechanisms
        operatorController.start().onTrue(
                Commands.runOnce(() -> Superstructure.getInstance().requestDisable()));

        // BACK: re-enable after emergency stop
        operatorController.back().onTrue(
                Commands.runOnce(() -> Superstructure.getInstance().requestIdle()));

        // ==================== MANUAL OVERRIDES ====================
        // D-pad up/down: hood angle trim ±2° (additive to auto-aim)
        operatorController.povUp().onTrue(
                Commands.runOnce(() -> Shooter.getInstance().adjustHoodTrim(2.0)));
        operatorController.povDown().onTrue(
                Commands.runOnce(() -> Shooter.getInstance().adjustHoodTrim(-2.0)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
