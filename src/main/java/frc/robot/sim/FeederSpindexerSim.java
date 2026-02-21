package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

import org.littletonrobotics.junction.Logger;

/**
 * Physics simulation for the Feeder and Spindexer subsystems, plus the
 * feeder exit beam-break sensor.
 *
 * <h2>Motor models:</h2>
 * <ul>
 *   <li><b>Feeder</b>    — {@link DCMotorSim}: lightweight transport roller.
 *   <li><b>Spindexer</b> — {@link DCMotorSim}: rotating indexer plate.
 * </ul>
 *
 * <h2>Beam break simulation:</h2>
 * The feeder exit beam break ({@link edu.wpi.first.wpilibj.DigitalInput} on DIO port
 * {@code kBeamBreakDIOPort}) is driven by a {@link DIOSim}.
 * {@link RobotSimulator} controls the beam-break state based on simulated ball position:
 * <ul>
 *   <li>{@code setBeamBreakTripped(true)} → ball is at feeder exit (interrupting the beam)
 *   <li>{@code setBeamBreakTripped(false)} → no ball at sensor
 * </ul>
 *
 * <h2>Active-low sensor note:</h2>
 * The real beam-break sensor is typically active-low (output LOW when beam is broken).
 * {@link DIOSim#setValue(boolean)} with {@code false} simulates a broken beam (ball present).
 */
public class FeederSpindexerSim {

    // ==================== MOTOR MODEL ====================

    private static final DCMotor kKrakenX60Foc = DCMotor.getKrakenX60Foc(1);

    // ==================== PHYSICS CONSTANTS ====================

    // Feeder: small transport roller, ~0.1 kg spinning mass at ~3 cm radius.
    // J_disk = 0.5 * m * r²  →  0.5 * 0.1 * 0.03² = 0.000045 kg·m²
    // Use a slightly larger value to account for roller + belt/chain inertia.
    private static final double kFeederMOI = 0.0005; // kg·m²

    // Spindexer: rotating indexer plate, ~0.5 kg at ~10 cm average radius.
    // J_disk = 0.5 * m * r²  →  0.5 * 0.5 * 0.1² = 0.0025 kg·m²
    private static final double kSpindexerMOI = 0.003; // kg·m²

    // Gear ratios (feeder and spindexer drive their rollers/plates directly — 1:1 assumed).
    private static final double kFeederGearRatio    = 1.0;
    private static final double kSpindexerGearRatio = 1.0;

    // ==================== SIMULATION OBJECTS ====================

    private final DCMotorSim feederSim;
    private final DCMotorSim spindexerSim;

    // Beam-break GPIO simulation — controls the DIO signal seen by the robot code.
    private final DIOSim beamBreakSim;

    // ==================== PHOENIX 6 SIM STATES ====================

    private final TalonFXSimState feederSimState;
    private final TalonFXSimState spindexerSimState;

    // ==================== CONSTRUCTOR ====================

    public FeederSpindexerSim() {
        Feeder    feeder    = Feeder.getInstance();
        Spindexer spindexer = Spindexer.getInstance();

        feederSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kKrakenX60Foc, kFeederMOI, kFeederGearRatio),
                kKrakenX60Foc);

        spindexerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(kKrakenX60Foc, kSpindexerMOI, kSpindexerGearRatio),
                kKrakenX60Foc);

        // Beam break: create DIOSim from the subsystem's DigitalInput object.
        // Initial state: beam intact (no ball), DIO reads HIGH (true).
        beamBreakSim = new DIOSim(feeder.getBeamBreakInput());
        beamBreakSim.setValue(true); // true = beam intact, false = beam broken

        feederSimState    = feeder.getFeederSimState();
        spindexerSimState = spindexer.getSpindexerSimState();
    }

    // ==================== UPDATE ====================

    /**
     * Advances feeder and spindexer physics by {@code dtSeconds}.
     *
     * @param dtSeconds loop period in seconds
     */
    public void update(double dtSeconds) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        updateFeeder(batteryVoltage, dtSeconds);
        updateSpindexer(batteryVoltage, dtSeconds);

        logTelemetry();
    }

    // ==================== FEEDER ====================

    private void updateFeeder(double batteryVoltage, double dt) {
        feederSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = feederSimState.getMotorVoltage();
        feederSim.setInputVoltage(motorVoltage);
        feederSim.update(dt);

        double mechPositionRot = feederSim.getAngularPositionRad() / (2.0 * Math.PI);
        double mechVelocityRPS = feederSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        feederSimState.setRawRotorPosition(mechPositionRot * kFeederGearRatio);
        feederSimState.setRotorVelocity(mechVelocityRPS * kFeederGearRatio);
    }

    // ==================== SPINDEXER ====================

    private void updateSpindexer(double batteryVoltage, double dt) {
        spindexerSimState.setSupplyVoltage(batteryVoltage);

        double motorVoltage = spindexerSimState.getMotorVoltage();
        spindexerSim.setInputVoltage(motorVoltage);
        spindexerSim.update(dt);

        double mechPositionRot = spindexerSim.getAngularPositionRad() / (2.0 * Math.PI);
        double mechVelocityRPS = spindexerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        spindexerSimState.setRawRotorPosition(mechPositionRot * kSpindexerGearRatio);
        spindexerSimState.setRotorVelocity(mechVelocityRPS * kSpindexerGearRatio);
    }

    // ==================== BEAM BREAK CONTROL ====================

    /**
     * Sets the simulated state of the feeder exit beam-break sensor.
     * Called by {@link RobotSimulator} based on simulated ball position.
     *
     * <p>Sensor is <b>active-low</b>: {@code setValue(false)} = beam broken = ball present.
     *
     * @param tripped {@code true} if a ball is interrupting the beam (ball at feeder exit)
     */
    public void setBeamBreakTripped(boolean tripped) {
        // DIO active-low: false = broken beam = ball present
        beamBreakSim.setValue(!tripped);
    }

    // ==================== ACCESSORS FOR BALL SIMULATION ====================

    /** @return true when the feeder motor is spinning (transporting a ball). */
    public boolean isFeederRunning() {
        return Math.abs(feederSim.getAngularVelocityRadPerSec()) > 0.5;
    }

    /** @return true when the spindexer motor is spinning (indexing balls). */
    public boolean isSpindexerRunning() {
        return Math.abs(spindexerSim.getAngularVelocityRadPerSec()) > 0.5;
    }

    /** @return feeder current draw in amps (for ball presence estimation calibration). */
    public double getFeederCurrentAmps() {
        return feederSim.getCurrentDrawAmps();
    }

    /** @return spindexer current draw in amps. */
    public double getSpindexerCurrentAmps() {
        return spindexerSim.getCurrentDrawAmps();
    }

    // ==================== TELEMETRY ====================

    private void logTelemetry() {
        Logger.recordOutput("Sim/Feeder/VelocityRPS",
                feederSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
        Logger.recordOutput("Sim/Feeder/CurrentA", feederSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Feeder/IsRunning", isFeederRunning());
        Logger.recordOutput("Sim/Spindexer/VelocityRPS",
                spindexerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
        Logger.recordOutput("Sim/Spindexer/CurrentA", spindexerSim.getCurrentDrawAmps());
        Logger.recordOutput("Sim/Spindexer/IsRunning", isSpindexerRunning());
    }
}
