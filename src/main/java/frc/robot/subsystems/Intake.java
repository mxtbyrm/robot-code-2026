package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;
import frc.robot.RobotState;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Slapdown intake subsystem for FRC 2026 REBUILT.
 *
 * <h2>Mechanical layout:</h2>
 * <ul>
 *   <li><b>2× Kraken X60 deploy motors</b> (left + right, mirror-inverted) —
 *       swing the intake arm outside the bumper perimeter to collect balls
 *       off the ground. Right motor follows left motor (inverted follower).</li>
 *   <li><b>1× Kraken X60 roller motor</b> — spins the intake rollers to
 *       grab balls and pull them inward.</li>
 * </ul>
 *
 * <h2>Ball path:</h2>
 * Ground → <b>Roller</b> → Spindexer hopper → Feeder → Flywheels → Hub
 *
 * <h2>Command workflow:</h2>
 * <ul>
 *   <li>{@code intakeCommand()} — deploys arm + spins roller.
 *       Hold to collect, release to stow. Bind alongside Spindexer.intakeCommand().</li>
 *   <li>{@code outtakeCommand()} — deploys arm + reverses roller to eject.</li>
 * </ul>
 */
public class Intake extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Intake instance;

    public static void initialize() {
        if (instance != null) throw new IllegalStateException("Intake already initialized.");
        instance = new Intake();
    }

    public static Intake getInstance() {
        if (instance == null) throw new IllegalStateException("Intake not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== MOTORS ====================
    private final TalonFX leftDeployMotor;
    private final TalonFX rightDeployMotor;
    private final TalonFX rollerMotor;

    // ==================== CONTROL REQUESTS ====================
    // withUpdateFreqHz(50): Phoenix 6 auto-resends control at 100 Hz by default. 50 Hz matches
    // the main loop and halves CAN traffic on the RoboRIO bus. Same fix applied to all three motors.
    private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0).withUpdateFreqHz(50);
    private final VoltageOut rightDeployRequest = new VoltageOut(0).withUpdateFreqHz(50);
    private final DutyCycleOut rollerRequest = new DutyCycleOut(0).withUpdateFreqHz(50);

    // ==================== GRAVITY FEEDFORWARD LOOKUP TABLES ====================
    // Non-linear gravity compensation for the slapdown linkage.
    // The passive flap changes effective torque at each arm position.
    private final InterpolatingDoubleTreeMap leftGravityTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rightGravityTable = new InterpolatingDoubleTreeMap();

    // ==================== STATE ====================
    private DeployState deployState = DeployState.STOWED;

    // ==================== HEALTH / CURRENT MONITORING ====================
    private final StatusSignal<Current> leftDeployCurrentSignal;
    private final StatusSignal<Current> rightDeployCurrentSignal;
    private final StatusSignal<Angle> deployPositionSignal;
    private final StatusSignal<Voltage> leftDeployOutputSignal;
    private int deployStallCycleCount = 0;
    private boolean healthy = true;
    private static final int kMaxConfigRetries = 5;

    public enum DeployState {
        STOWED,
        DEPLOYED,
        HOVER
    }

    private Intake() {
        leftDeployMotor = new TalonFX(IntakeConstants.kLeftDeployMotorId);
        rightDeployMotor = new TalonFX(IntakeConstants.kRightDeployMotorId);
        rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId);

        // Populate gravity feedforward lookup tables
        for (double[] entry : IntakeConstants.kDeployLeftGravityTable) {
            leftGravityTable.put(entry[0], entry[1]);
        }
        for (double[] entry : IntakeConstants.kDeployRightGravityTable) {
            rightGravityTable.put(entry[0], entry[1]);
        }

        configureDeployMotors();
        configureRollerMotor();

        // Cache deploy current for stall detection (10Hz is sufficient for stall counting)
        leftDeployCurrentSignal = leftDeployMotor.getStatorCurrent();
        leftDeployCurrentSignal.setUpdateFrequency(10);
        rightDeployCurrentSignal = rightDeployMotor.getStatorCurrent();
        rightDeployCurrentSignal.setUpdateFrequency(10);

        // Cache deploy position signal — avoid synchronous CAN read in getDeployPosition()
        // 30Hz — deploy arm is slow but needs reasonable feedback for Motion Magic.
        deployPositionSignal = leftDeployMotor.getPosition();
        deployPositionSignal.setUpdateFrequency(30);

        // Cache left motor's closed-loop output so we can mirror it to the right motor
        // 50Hz matches the robot loop rate
        leftDeployOutputSignal = leftDeployMotor.getMotorVoltage();
        leftDeployOutputSignal.setUpdateFrequency(50);

        leftDeployMotor.optimizeBusUtilization();
        rightDeployMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    /** Retry loop for config. */
    private void applyConfig(Supplier<StatusCode> apply, String device) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxConfigRetries; i++) {
            status = apply.get();
            if (status.isOK()) return;
        }
        healthy = false;
        DriverStation.reportError("[Intake] " + device + " config FAILED: " + status, false);
    }

    // ==================== MOTOR CONFIGURATION ====================

    private void configureDeployMotors() {
        // ---- Left deploy motor (LEADER) ----
        // The left motor runs PID + Motion Magic. Gravity compensation is NOT
        // handled by CTRE's built-in kG (Arm_Cosine doesn't match our slapdown
        // linkage torque profile). Instead, we inject gravity feedforward via
        // the lookup table as an arbitrary feedforward voltage each cycle.
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = IntakeConstants.kDeployP;
        config.Slot0.kI = IntakeConstants.kDeployI;
        config.Slot0.kD = IntakeConstants.kDeployD;
        config.Slot0.kS = IntakeConstants.kDeployS;
        config.Slot0.kG = 0.0; // gravity handled externally via lookup table

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kDeployCurrentLimit;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = IntakeConstants.kDeployGearRatio;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.kDeployStowedRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.kDeployExtendedRotations;

        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = IntakeConstants.kDeployMaxVelocity;
        mm.MotionMagicAcceleration = IntakeConstants.kDeployMaxAcceleration;

        applyConfig(() -> leftDeployMotor.getConfigurator().apply(config), "Left Deploy");
        leftDeployMotor.setPosition(IntakeConstants.kDeployStowedRotations);

        // ---- Right deploy motor (VOLTAGE MIRROR + EXTRA GRAVITY) ----
        // Not a follower — each cycle we read the left motor's output voltage
        // and send it to the right motor with an extra gravity offset.
        // This gives asymmetric gravity compensation without two fighting PIDs.
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kDeployCurrentLimit;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // opposed to left
        applyConfig(() -> rightDeployMotor.getConfigurator().apply(rightConfig), "Right Deploy");
        rightDeployMotor.setPosition(IntakeConstants.kDeployStowedRotations);
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // coast so balls don't get stuck
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        applyConfig(() -> rollerMotor.getConfigurator().apply(config), "Roller");
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // ---- Refresh cached status signals ONCE per cycle ----
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(
                leftDeployCurrentSignal, rightDeployCurrentSignal,
                deployPositionSignal, leftDeployOutputSignal);

        double armPosition = deployPositionSignal.getValueAsDouble();

        // ---- Update left motor gravity FF (runs every cycle so FF tracks arm position) ----
        double leftGravityFF = leftGravityTable.get(armPosition);
        leftDeployMotor.setControl(deployRequest.withFeedForward(leftGravityFF));

        // ---- Right motor: mirror left PID output + right-side gravity ----
        // leftOutputVolts includes PID + kS + left gravity FF.
        // Subtract left gravity FF → leaves PID + kS (kS carries over naturally).
        // Add right gravity FF → right motor gets same PID + kS + its own gravity.
        double leftOutputVolts = leftDeployOutputSignal.getValueAsDouble();
        double rightGravityFF = rightGravityTable.get(armPosition);
        double pidOutput = leftOutputVolts - leftGravityFF;
        rightDeployMotor.setControl(
                rightDeployRequest.withOutput(pidOutput + rightGravityFF));

        double deployCurrent = Math.max(
                leftDeployCurrentSignal.getValueAsDouble(),
                rightDeployCurrentSignal.getValueAsDouble());
        if (deployState != DeployState.STOWED && !isDeployAtTarget()) {
            if (deployCurrent > IntakeConstants.kDeployStallCurrentThreshold) {
                deployStallCycleCount++;
            } else {
                deployStallCycleCount = Math.max(0, deployStallCycleCount - 1);
            }

            if (deployStallCycleCount >= IntakeConstants.kDeployStallCycleThreshold) {
                DriverStation.reportWarning(
                        "[Intake] Deploy stall detected! Current: " + deployCurrent + "A", false);
                deployStallCycleCount = 0; // reset — don't spam
            }
        } else {
            deployStallCycleCount = 0;
        }

        // Telemetry (AdvantageKit only)
        Logger.recordOutput("Intake/DeployState", deployState.name());
        Logger.recordOutput("Intake/DeployPosition", getDeployPosition());
        Logger.recordOutput("Intake/DeployAtTarget", isDeployAtTarget());
        Logger.recordOutput("Intake/RollerOutput", rollerMotor.get());
        Logger.recordOutput("Intake/LeftDeployCurrent", leftDeployCurrentSignal.getValueAsDouble());
        Logger.recordOutput("Intake/RightDeployCurrent", rightDeployCurrentSignal.getValueAsDouble());
        Logger.recordOutput("Intake/LeftGravityFF", leftGravityFF);
        Logger.recordOutput("Intake/RightGravityFF", rightGravityFF);
        Logger.recordOutput("Intake/PIDOutput", pidOutput);

        RobotState.getInstance().setIntakeHealthy(healthy);
    }

    // ==================== DEPLOY CONTROL ====================

    /** Deploy the intake arm past the bumpers to collect balls. */
    public void deploy() {
        deployState = DeployState.DEPLOYED;
        setDeployTarget(IntakeConstants.kDeployExtendedRotations);
    }

    /** Retract the intake arm inside the bumpers. */
    public void stow() {
        deployState = DeployState.STOWED;
        setDeployTarget(IntakeConstants.kDeployStowedRotations);
    }

    /** Partial deploy — arm just past bumper line (hover position). */
    public void hover() {
        deployState = DeployState.HOVER;
        setDeployTarget(IntakeConstants.kDeployHoverRotations);
    }

    /** Send position target to the left motor with gravity FF from lookup table. */
    private void setDeployTarget(double positionRotations) {
        double currentPos = deployPositionSignal.getValueAsDouble();
        double leftGravityFF = leftGravityTable.get(currentPos);
        leftDeployMotor.setControl(
                deployRequest.withPosition(positionRotations).withFeedForward(leftGravityFF));
    }

    /** @return current deploy arm position in mechanism rotations */
    public double getDeployPosition() {
        return deployPositionSignal.getValueAsDouble();
    }

    /** @return true if deploy arm is at the target position */
    public boolean isDeployAtTarget() {
        double target;
        switch (deployState) {
            case DEPLOYED: target = IntakeConstants.kDeployExtendedRotations; break;
            case HOVER:    target = IntakeConstants.kDeployHoverRotations; break;
            default:       target = IntakeConstants.kDeployStowedRotations; break;
        }
        return Math.abs(getDeployPosition() - target) < IntakeConstants.kDeployToleranceRotations;
    }

    /** @return true if intake is stowed inside bumpers */
    public boolean isStowed() {
        return deployState == DeployState.STOWED && isDeployAtTarget();
    }

    /** @return true if intake is fully deployed */
    public boolean isDeployed() {
        return deployState == DeployState.DEPLOYED && isDeployAtTarget();
    }

    // ==================== ROLLER CONTROL ====================

    /** Run the roller inward to collect balls. */
    public void runRollerIntake() {
        rollerMotor.setControl(rollerRequest.withOutput(IntakeConstants.kRollerIntakeSpeed));
    }

    /** Run the roller outward to eject balls. */
    public void runRollerOuttake() {
        rollerMotor.setControl(rollerRequest.withOutput(IntakeConstants.kRollerOuttakeSpeed));
    }

    /** Stop the roller. */
    public void stopRoller() {
        rollerMotor.setControl(rollerRequest.withOutput(0));
    }

    // ==================== STOP ALL ====================

    /** Stop all intake motors and stow the arm. */
    public void stopAll() {
        stow();
        stopRoller();
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * <b>THE teleop intake command.</b> Bind to a button with {@code whileTrue()}.
     * Run in parallel with {@code Spindexer.intakeCommand()}.
     *
     * <p>While held:
     * <ul>
     *   <li>Deploys the arm past the bumpers</li>
     *   <li>Spins the roller inward to grab balls</li>
     * </ul>
     *
     * <p>On release:
     * <ul>
     *   <li>Stows the arm back inside bumpers</li>
     *   <li>Stops roller</li>
     * </ul>
     */
    public Command intakeCommand() {
        return Commands.startEnd(
                () -> {
                    deploy();
                    runRollerIntake();
                },
                this::stopAll,
                this
        ).withName("Intake");
    }

    /**
     * Outtake — deploys arm and reverses roller to eject balls.
     * Hold to eject, release to stow.
     */
    public Command outtakeCommand() {
        return Commands.startEnd(
                () -> {
                    deploy();
                    runRollerOuttake();
                },
                this::stopAll,
                this
        ).withName("Outtake");
    }

    /**
     * Unjam — reverses the roller briefly to clear a jam.
     * Hold to reverse, release to stop. Bind alongside Spindexer.reverseCommand().
     */
    public Command unjamCommand() {
        return Commands.startEnd(
                this::runRollerOuttake,
                this::stopRoller,
                this
        ).withName("Unjam");
    }

    /**
     * Autonomous intake — deploys and collects until interrupted.
     * Run in parallel with {@code Spindexer.intakeCommand()}.
     */
    public Command autoIntakeCommand() {
        return intakeCommand().withName("Auto Intake");
    }

    /**
     * Stow command — retracts the arm. Finishes when stowed.
     */
    public Command stowCommand() {
        return Commands.runOnce(this::stow, this)
                .andThen(Commands.waitUntil(this::isStowed))
                .withName("Stow");
    }

    // ==================== SIMULATION SUPPORT ====================

    /** Exposes left deploy TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getLeftDeploySimState() {
        return leftDeployMotor.getSimState();
    }

    /** Exposes right deploy TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getRightDeploySimState() {
        return rightDeployMotor.getSimState();
    }

    /** Exposes roller TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getRollerSimState() {
        return rollerMotor.getSimState();
    }

    /** Returns whether the roller is currently spinning (for ball simulation). */
    public boolean isRollerRunning() {
        return Math.abs(rollerMotor.getDutyCycle().getValueAsDouble()) > 0.05;
    }

    // ==================== SYSID CHARACTERIZATION ====================

    /** Apply raw voltage to the deploy motor (for SysId). */
    public void setDeployVoltage(double volts) {
        leftDeployMotor.setVoltage(volts);
    }

    /** Deploy position in mechanism rotations (for SysId logging). */
    public double getDeployPositionRotations() {
        return deployPositionSignal.getValueAsDouble();
    }

    /** Deploy velocity in mechanism rotations/sec (for SysId logging). */
    public double getDeployVelocityRPS() {
        return leftDeployMotor.getVelocity().getValueAsDouble();
    }
}
