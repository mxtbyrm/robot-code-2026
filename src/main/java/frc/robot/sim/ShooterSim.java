package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

/**
 * Physics simulation for the Shooter subsystem (Flywheel + Hood + Turret).
 *
 * <h2>Motor models:</h2>
 * <ul>
 *   <li><b>Flywheel</b>  — {@link DCMotorSim}: two heavy spinning wheels, high inertia.
 *   <li><b>Hood</b>      — {@link SingleJointedArmSim}: short arm tilting under gravity.
 *       Gravity is simulated so that {@code kHoodG} (Arm_Cosine FF) can be tuned in sim.
 *   <li><b>Turret</b>    — {@link DCMotorSim}: horizontal rotation, no gravity.
 *       Spring cable compensation ({@code kTurretSpringFeedForwardVPerDeg}) is NOT modelled
 *       because the spring restoring torque is a real-hardware phenomenon that would require
 *       a custom spring constant. In sim the turret holds perfectly; tune spring FF on robot.
 * </ul>
 *
 * <h2>Phoenix 6 integration:</h2>
 * Each {@link DCMotorSim}/{@link SingleJointedArmSim} feeds back into the motor's
 * {@link TalonFXSimState} so that all Phoenix 6 closed-loop control (PID, Motion Magic,
 * feedforward) sees realistic position and velocity signals.
 */
public class ShooterSim {

    // ==================== MOTOR MODEL ====================

    private static final DCMotor kKrakenX60Foc = DCMotor.getKrakenX60Foc(1);

    // ==================== PHYSICS CONSTANTS ====================

    // Flywheel: two ~700 g iron wheels at 4-inch (0.0508 m) radius.
    // J_disk = 0.5 * m * r²  →  per wheel = 0.5 * 0.70 * 0.0508² = 0.000903 kg·m²
    // Two wheels: J ≈ 0.0018 kg·m²   (at mechanism shaft, before gear ratio)
    private static final double kFlywheelMOI = 0.0018; // kg·m²

    // Hood: short tilting arm, ~200 g effective link mass, ~15 cm moment arm.
    // J_arm = 1/3 * m * L²  →  1/3 * 0.20 * 0.15² = 0.0015 kg·m²
    private static final double kHoodArmMassKg   = 0.20;
    private static final double kHoodArmLengthM  = 0.15;

    // Turret: rotating platform ~1.5 kg total, ~15 cm average radius.
    // J_disk = 0.5 * m * r²  →  0.5 * 1.5 * 0.15² = 0.01688 kg·m²
    private static final double kTurretMOI = 0.017; // kg·m²

    // Hood angle limits converted to radians (for SingleJointedArmSim).
    // The arm sim treats angle 0 = horizontal, positive = upward.
    // We model the hood angle directly in degrees-from-reference, mapped to radians.
    private static final double kHoodMinRad =
            Units.degreesToRadians(ShooterConstants.kHoodMinAngleDegrees);
    private static final double kHoodMaxRad =
            Units.degreesToRadians(ShooterConstants.kHoodMaxAngleDegrees);

    // Turret soft limits in radians (used to clamp turret sim position).
    private static final double kTurretMinRad =
            Units.degreesToRadians(ShooterConstants.kTurretMinAngleDegrees);
    private static final double kTurretMaxRad =
            Units.degreesToRadians(ShooterConstants.kTurretMaxAngleDegrees);

    // ==================== SIMULATION OBJECTS ====================

    /** Flywheel physics: high inertia spinning wheel pair. */
    private final DCMotorSim flywheelSim;

    /**
     * Hood physics: short arm tilting under gravity. Simulates the {@code GravityType.Arm_Cosine}
     * feedforward behaviour so operators can tune {@code kHoodG} against the sim.
     */
    private final SingleJointedArmSim hoodSim;

    /** Turret physics: horizontal rotation plate. */
    private final DCMotorSim turretSim;

    // ==================== PHOENIX 6 SIM STATES ====================

    private final TalonFXSimState flywheelSimState;
    private final TalonFXSimState hoodSimState;
    private final TalonFXSimState turretSimState;

    // ==================== STATE TRACKING ====================

    /** Accumulated turret angle in radians (integrated from velocity, not wrapped). */
    private double turretAngleRad = 0.0;

    // ==================== CONSTRUCTOR ====================

    public ShooterSim() {
        Shooter shooter = Shooter.getInstance();

        // -- Flywheel (DCMotorSim) --
        // gearing = kFlywheelGearRatio (motor rotations per mechanism rotation)
        flywheelSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kKrakenX60Foc, kFlywheelMOI, ShooterConstants.kFlywheelGearRatio),
                kKrakenX60Foc);

        // -- Hood (SingleJointedArmSim) --
        // simulateGravity=true: tests that kHoodG feedforward holds position.
        // Starting angle = minimum (most open / lowest PID setpoint at power-on).
        hoodSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                ShooterConstants.kHoodGearRatio,
                SingleJointedArmSim.estimateMOI(kHoodArmLengthM, kHoodArmMassKg),
                kHoodArmLengthM,
                kHoodMinRad,
                kHoodMaxRad,
                true,   // simulate gravity
                kHoodMinRad);

        // -- Turret (DCMotorSim) --
        // Horizontal rotation — no gravity effect modelled (spring FF tuned on real robot).
        turretSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kKrakenX60Foc, kTurretMOI, ShooterConstants.kTurretGearRatio),
                kKrakenX60Foc);

        // -- Phoenix 6 sim states --
        flywheelSimState = shooter.getFlywheelSimState();
        hoodSimState      = shooter.getHoodSimState();
        turretSimState    = shooter.getTurretSimState();

        // Initialize hood rotor position to match the real setPosition() call in Shooter constructor.
        // Real code: hoodMotor.setPosition(kHoodMinRotations)  → rotor at kHoodMinRotations * gearRatio
        hoodSimState.setRawRotorPosition(
                ShooterConstants.kHoodMinRotations * ShooterConstants.kHoodGearRatio);

        // Turret starts centered (0°), matching real code's setPosition(0).
        turretSimState.setRawRotorPosition(0.0);
        flywheelSimState.setRawRotorPosition(0.0);
    }

    // ==================== UPDATE ====================

    /**
     * Advances all shooter physics by {@code dtSeconds} and writes the results back to
     * the Phoenix 6 sim states so PID / velocity closed-loop control sees correct feedback.
     *
     * <p>Called from {@link RobotSimulator#update(double)}.
     *
     * @param dtSeconds loop period in seconds (typically 0.02)
     */
    public void update(double dtSeconds) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        updateFlywheel(batteryVoltage, dtSeconds);
        updateHood(batteryVoltage, dtSeconds);
        updateTurret(batteryVoltage, dtSeconds);

        logTelemetry();
    }

    // ==================== FLYWHEEL ====================

    private void updateFlywheel(double batteryVoltage, double dt) {
        flywheelSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = flywheelSimState.getMotorVoltage();
        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(dt);

        // Rotor position and velocity (at motor shaft = mechanism × gear ratio)
        double mechPositionRot = flywheelSim.getAngularPositionRad() / (2.0 * Math.PI);
        double mechVelocityRPS = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        flywheelSimState.setRawRotorPosition(mechPositionRot * ShooterConstants.kFlywheelGearRatio);
        flywheelSimState.setRotorVelocity(mechVelocityRPS * ShooterConstants.kFlywheelGearRatio);
    }

    // ==================== HOOD ====================

    private void updateHood(double batteryVoltage, double dt) {
        hoodSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = hoodSimState.getMotorVoltage();
        hoodSim.setInputVoltage(motorVoltage);
        hoodSim.update(dt);

        // Hood angle from SingleJointedArmSim is in radians from the minimum angle.
        // Map back to motor rotations for Phoenix 6.
        // angle_delta_from_min = hoodSim.getAngleRads() - kHoodMinRad
        // mechanism_delta_rot  = angle_delta / (2π)
        // rotor_position       = (kHoodMinRotations + mech_delta) * gearRatio
        double angleRad      = hoodSim.getAngleRads();
        double angleDeltaRad = angleRad - kHoodMinRad;
        double mechPositionRot = ShooterConstants.kHoodMinRotations
                                 + angleDeltaRad / (2.0 * Math.PI);
        double mechVelocityRPS = hoodSim.getVelocityRadPerSec() / (2.0 * Math.PI);

        hoodSimState.setRawRotorPosition(mechPositionRot * ShooterConstants.kHoodGearRatio);
        hoodSimState.setRotorVelocity(mechVelocityRPS * ShooterConstants.kHoodGearRatio);
    }

    // ==================== TURRET ====================

    private void updateTurret(double batteryVoltage, double dt) {
        turretSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = turretSimState.getMotorVoltage();
        turretSim.setInputVoltage(motorVoltage);
        turretSim.update(dt);

        // Integrate velocity to track absolute turret angle (clamped at soft limits).
        double velocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
        turretAngleRad += velocityRadPerSec * dt;
        turretAngleRad = Math.max(kTurretMinRad, Math.min(kTurretMaxRad, turretAngleRad));

        // Convert to motor rotations (rotor = mechanism × gear ratio).
        double mechPositionRot = turretAngleRad / (2.0 * Math.PI);
        double mechVelocityRPS = velocityRadPerSec / (2.0 * Math.PI);

        turretSimState.setRawRotorPosition(mechPositionRot * ShooterConstants.kTurretGearRatio);
        turretSimState.setRotorVelocity(mechVelocityRPS * ShooterConstants.kTurretGearRatio);
    }

    // ==================== TELEMETRY ====================

    private void logTelemetry() {
        Logger.recordOutput("Sim/Shooter/FlywheelVelocityRPS",
                flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
        Logger.recordOutput("Sim/Shooter/FlywheelCurrentA",
                flywheelSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Shooter/HoodAngleDeg",
                Units.radiansToDegrees(hoodSim.getAngleRads()));
        Logger.recordOutput("Sim/Shooter/HoodCurrentA",
                hoodSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Shooter/TurretAngleDeg",
                Units.radiansToDegrees(turretAngleRad));
        Logger.recordOutput("Sim/Shooter/TurretCurrentA",
                turretSim.getCurrentDrawAmps());
    }

    // ==================== ACCESSORS FOR BALL SIMULATION ====================

    /** @return simulated flywheel velocity in RPS (mechanism shaft). */
    public double getFlywheelVelocityRPS() {
        return flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    }

    /** @return true when the flywheel is within tolerance of any non-zero target. */
    public boolean isFlywheelApproxAtSpeed() {
        // Proxy: if flywheel is spinning fast enough to shoot (>= idle pre-spin threshold)
        return getFlywheelVelocityRPS() >= ShooterConstants.kIdleFlywheelRPS;
    }
}
