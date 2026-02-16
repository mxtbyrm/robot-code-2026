package frc.robot.subsystems;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants.SwerveConstants;

/**
 * Physics simulation for the swerve drivetrain.
 *
 * <p>Uses WPILib {@link DCMotorSim} for each module's drive and steer motors,
 * and Phoenix6 {@link TalonFXSimState} / {@link Pigeon2SimState} to feed
 * simulated sensor values back into the hardware abstraction layer.
 *
 * <p>This allows full autonomous testing in simulation — Choreo trajectories,
 * pose estimation, and vision fusion all work without hardware.
 */
public class SwerveDriveSim {

    // ==================== SIM MOTORS ====================
    private final DCMotorSim[] driveSims = new DCMotorSim[4];
    private final DCMotorSim[] steerSims = new DCMotorSim[4];

    // ==================== REFERENCES ====================
    private final SwerveModule[] modules;
    private final Pigeon2 pigeon;
    private final SwerveDriveKinematics kinematics;

    // Simulated gyro heading (integrated from chassis speeds)
    private double simHeadingRad = 0.0;

    // Kraken X60 motor model
    private static final DCMotor kKrakenX60 = DCMotor.getKrakenX60(1);

    // Moment of inertia estimates (from SwerveConstants)
    private static final double kDriveMOI = SwerveConstants.kDriveSimMOI;
    private static final double kSteerMOI = SwerveConstants.kSteerSimMOI;

    public SwerveDriveSim(SwerveModule[] modules, Pigeon2 pigeon) {
        this.modules = modules;
        this.pigeon = pigeon;
        this.kinematics = SwerveConstants.kSwerveKinematics;

        for (int i = 0; i < 4; i++) {
            driveSims[i] = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(kKrakenX60, kDriveMOI, SwerveConstants.kDriveGearRatio),
                    kKrakenX60);
            steerSims[i] = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(kKrakenX60, kSteerMOI, SwerveConstants.kSteerGearRatio),
                    kKrakenX60);
        }
    }

    /**
     * Update the simulation physics. Call this every robot period (20ms) from
     * {@code simulationPeriodic()}.
     *
     * @param dtSeconds Time step in seconds (typically 0.02)
     */
    public void update(double dtSeconds) {
        double batteryVoltage = RobotController.getBatteryVoltage();

        SwerveModuleState[] simStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            // Get the Phoenix6 sim states for each motor
            TalonFXSimState driveSimState = modules[i].getDriveMotorSimState();
            TalonFXSimState steerSimState = modules[i].getSteerMotorSimState();

            // Set supply voltage
            driveSimState.setSupplyVoltage(batteryVoltage);
            steerSimState.setSupplyVoltage(batteryVoltage);

            // Get the applied voltage from the motor controllers
            double driveVoltage = driveSimState.getMotorVoltage();
            double steerVoltage = steerSimState.getMotorVoltage();

            // Feed voltage into physics simulation
            driveSims[i].setInputVoltage(driveVoltage);
            steerSims[i].setInputVoltage(steerVoltage);

            // Step physics forward
            driveSims[i].update(dtSeconds);
            steerSims[i].update(dtSeconds);

            // Feed simulated sensor values back into Phoenix6
            // Drive motor: position in rotations, velocity in rotations/sec
            driveSimState.setRawRotorPosition(
                    driveSims[i].getAngularPositionRotations() * SwerveConstants.kDriveGearRatio);
            driveSimState.setRotorVelocity(
                    driveSims[i].getAngularVelocityRPM() / 60.0 * SwerveConstants.kDriveGearRatio);

            // Steer motor: position in rotations, velocity in rotations/sec
            steerSimState.setRawRotorPosition(
                    steerSims[i].getAngularPositionRotations() * SwerveConstants.kSteerGearRatio);
            steerSimState.setRotorVelocity(
                    steerSims[i].getAngularVelocityRPM() / 60.0 * SwerveConstants.kSteerGearRatio);

            // Build simulated module state for gyro integration
            double driveVelocityMPS = driveSims[i].getAngularVelocityRPM() / 60.0
                    * SwerveConstants.kWheelCircumferenceMeters;
            Rotation2d steerAngle = Rotation2d.fromRotations(
                    steerSims[i].getAngularPositionRotations());
            simStates[i] = new SwerveModuleState(driveVelocityMPS, steerAngle);
        }

        // Integrate chassis speeds to update simulated gyro heading
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(simStates);
        simHeadingRad += chassisSpeeds.omegaRadiansPerSecond * dtSeconds;

        // Update Pigeon2 sim state
        Pigeon2SimState pigeonSim = pigeon.getSimState();
        pigeonSim.setSupplyVoltage(batteryVoltage);
        pigeonSim.setRawYaw(Math.toDegrees(simHeadingRad));
    }
}
