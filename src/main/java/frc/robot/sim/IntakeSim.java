package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

/**
 * Physics simulation for the Intake subsystem (deploy arm + roller).
 *
 * <h2>Motor models:</h2>
 * <ul>
 *   <li><b>Deploy arm</b> — {@link SingleJointedArmSim}: slapdown arm with gravity.
 *       Both left and right deploy motors are driven by the same physics model — they move
 *       the same arm. Left motor is the position-controlled master; right mirrors it.
 *       Gravity is simulated so gravity feedforward tables can be tuned in sim.
 *   <li><b>Roller</b>    — {@link DCMotorSim}: small cylinder, mostly for current feedback.
 * </ul>
 *
 * <h2>Arm angle convention:</h2>
 * {@link SingleJointedArmSim} uses angles from the positive X axis (horizontal), positive CCW.
 * We model the arm as starting near vertical (90°, stowed) and swinging down to approximately
 * 0° (deployed, horizontal). The Phoenix 6 encoder zero at stowed maps to 90° in the sim.
 */
public class IntakeSim {

    // ==================== MOTOR MODEL ====================

    private static final DCMotor kKrakenX60Foc = DCMotor.getKrakenX60Foc(1);

    // ==================== PHYSICS CONSTANTS ====================

    // Deploy arm: ~1.0 kg total (arm bar + roller + bracket), 0.4 m from pivot to roller.
    // J_arm = 1/3 * m * L²  →  1/3 * 1.0 * 0.4² = 0.0533 kg·m²
    private static final double kDeployArmMassKg  = 1.0;
    private static final double kDeployArmLengthM = 0.40;

    // Roller: small cylinder ~0.2 kg, 2-inch radius.
    // J_disk = 0.5 * m * r²  →  0.5 * 0.2 * 0.0254² = 0.0000647 kg·m²
    private static final double kRollerMOI = 0.0001; // kg·m²

    // Arm angle range:
    //   STOWED  ≈ 90° from horizontal (arm pointing upward)
    //   DEPLOYED ≈ 0° from horizontal (arm pointing forward/down to floor)
    // SingleJointedArmSim convention: 0 = horizontal, +90° = straight up.
    private static final double kDeployStowedAngleRad   = Math.PI / 2.0; // 90°
    private static final double kDeployDeployedAngleRad = 0.0;           // 0° (floor contact)

    // ==================== SIMULATION OBJECTS ====================

    /** Deploy arm physics (left = position master). */
    private final SingleJointedArmSim deployArmSim;

    /** Roller physics. */
    private final DCMotorSim rollerSim;

    // ==================== PHOENIX 6 SIM STATES ====================

    private final TalonFXSimState leftDeploySimState;
    private final TalonFXSimState rightDeploySimState;
    private final TalonFXSimState rollerSimState;

    // ==================== STATE TRACKING ====================

    /** Current arm angle in radians (from SingleJointedArmSim). */
    private double armAngleRad = kDeployStowedAngleRad;

    // ==================== CONSTRUCTOR ====================

    public IntakeSim() {
        Intake intake = Intake.getInstance();

        // Deploy arm: starts stowed (vertical, 90°), can swing down to 0° (deployed).
        // simulateGravity=true: tests that gravity FF tables hold the arm at each position.
        deployArmSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                IntakeConstants.kDeployGearRatio,
                SingleJointedArmSim.estimateMOI(kDeployArmLengthM, kDeployArmMassKg),
                kDeployArmLengthM,
                kDeployDeployedAngleRad,
                kDeployStowedAngleRad,
                true,                    // simulate gravity
                kDeployStowedAngleRad);  // start stowed

        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kKrakenX60Foc, kRollerMOI, IntakeConstants.kRollerGearRatio),
                kKrakenX60Foc);

        leftDeploySimState  = intake.getLeftDeploySimState();
        rightDeploySimState = intake.getRightDeploySimState();
        rollerSimState      = intake.getRollerSimState();
    }

    // ==================== UPDATE ====================

    /**
     * Advances intake physics by {@code dtSeconds}.
     * Both left and right deploy motors feed from the same arm physics model.
     *
     * @param dtSeconds loop period in seconds
     */
    public void update(double dtSeconds) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        updateDeployArm(batteryVoltage, dtSeconds);
        updateRoller(batteryVoltage, dtSeconds);

        logTelemetry();
    }

    // ==================== DEPLOY ARM ====================

    private void updateDeployArm(double batteryVoltage, double dt) {
        // Both motors drive the same arm — use left motor voltage (position master).
        leftDeploySimState.setSupplyVoltage(batteryVoltage);
        rightDeploySimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = leftDeploySimState.getMotorVoltage();
        deployArmSim.setInputVoltage(motorVoltage);
        deployArmSim.update(dt);

        armAngleRad = deployArmSim.getAngleRads();
        double velocityRadPerSec = deployArmSim.getVelocityRadPerSec();

        // Map arm angle to Phoenix 6 mechanism position.
        // The real code sets position = 0 at stowed and uses a negative extended position.
        // delta_from_stow (rad) = armAngleRad - kDeployStowedAngleRad  (negative when deployed)
        // mechanism_position (rot) = delta / (2π)
        // rotor_position (rot) = mechanism × gearRatio
        double deltaFromStowRad = armAngleRad - kDeployStowedAngleRad;
        double mechPositionRot  = deltaFromStowRad / (2.0 * Math.PI);
        double mechVelocityRPS  = velocityRadPerSec / (2.0 * Math.PI);

        double rotorPosition = mechPositionRot * IntakeConstants.kDeployGearRatio;
        double rotorVelocity = mechVelocityRPS  * IntakeConstants.kDeployGearRatio;

        leftDeploySimState.setRawRotorPosition(rotorPosition);
        leftDeploySimState.setRotorVelocity(rotorVelocity);

        // Right motor mirrors left (same arm, same position).
        rightDeploySimState.setRawRotorPosition(rotorPosition);
        rightDeploySimState.setRotorVelocity(rotorVelocity);
    }

    // ==================== ROLLER ====================

    private void updateRoller(double batteryVoltage, double dt) {
        rollerSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = rollerSimState.getMotorVoltage();
        rollerSim.setInputVoltage(motorVoltage);
        rollerSim.update(dt);

        double mechPositionRot = rollerSim.getAngularPositionRad() / (2.0 * Math.PI);
        double mechVelocityRPS = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        rollerSimState.setRawRotorPosition(mechPositionRot * IntakeConstants.kRollerGearRatio);
        rollerSimState.setRotorVelocity(mechVelocityRPS * IntakeConstants.kRollerGearRatio);
    }

    // ==================== TELEMETRY ====================

    private void logTelemetry() {
        Logger.recordOutput("Sim/Intake/ArmAngleDeg", Units.radiansToDegrees(armAngleRad));
        Logger.recordOutput("Sim/Intake/ArmCurrentA", deployArmSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Intake/RollerVelocityRPS",
                rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
        Logger.recordOutput("Sim/Intake/RollerCurrentA", rollerSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Intake/IsDeployed", isDeployed());
        Logger.recordOutput("Sim/Intake/RollerRunning", isRollerRunning());
    }

    // ==================== ACCESSORS FOR BALL SIMULATION ====================

    /**
     * @return true when the intake arm is sufficiently deployed to collect balls.
     *         Threshold: arm within 15° of the deployed (floor) position.
     */
    public boolean isDeployed() {
        return armAngleRad < Units.degreesToRadians(15.0);
    }

    /**
     * @return true when the roller is spinning fast enough to collect balls.
     *         Uses the roller surface speed via {@link Intake#isRollerRunning()}.
     */
    public boolean isRollerRunning() {
        return Math.abs(rollerSim.getAngularVelocityRadPerSec()) > 0.5;
    }
}
