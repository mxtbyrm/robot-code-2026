package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

/**
 * SysId characterization routines for all subsystems with tunable PID/FF.
 *
 * <p>Usage: bind the quasistatic and dynamic commands to buttons on a test controller.
 * Run each direction (forward/reverse) and export the log to WPILib's SysId tool.
 *
 * <p>Subsystems covered:
 * <ul>
 *   <li><b>Swerve drive</b> — all modules pointed forward, characterize drive motors (kS, kV, kA)</li>
 *   <li><b>Shooter flywheel</b> — velocity system (kS, kV, kA)</li>
 *   <li><b>Shooter hood</b> — position system with gravity (kG, kP, kD)</li>
 *   <li><b>Shooter turret</b> — position system (kP, kD)</li>
 *   <li><b>Intake deploy</b> — position system with gravity (kG, kP, kD)</li>
 * </ul>
 */
public class SysIdCommands {

    private SysIdCommands() {} // utility class

    // ==================== SWERVE DRIVE ====================

    public static SysIdRoutine createDriveRoutine(SwerveDrive drive) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(1.0),
                        Volts.of(7.0),
                        Seconds.of(10.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> drive.runDriveCharacterization(voltage.in(Volts)),
                        log -> {
                            SwerveModule[] modules = drive.getModules();
                            for (int i = 0; i < modules.length; i++) {
                                log.motor("drive-" + modules[i].getName())
                                        .voltage(Volts.of(drive.getSysIdDriveVolts()))
                                        .linearPosition(Meters.of(
                                                modules[i].getDrivePositionRotations()
                                                        * SwerveConstants.kWheelCircumferenceMeters))
                                        .linearVelocity(MetersPerSecond.of(
                                                modules[i].getDriveVelocityRPS()
                                                        * SwerveConstants.kWheelCircumferenceMeters));
                            }
                        },
                        drive
                )
        );
    }

    // ==================== SWERVE STEER ====================

    public static SysIdRoutine createSteerRoutine(SwerveDrive drive) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.5),
                        Volts.of(3.0),
                        Seconds.of(5.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> drive.runSteerCharacterization(voltage.in(Volts)),
                        log -> {
                            // Signs are already consistent: positive commanded voltage →
                            // CW motor (CW_Positive) → CCW azimuth (gearbox reverses) →
                            // positive RemoteCANcoder reading (CCW_Positive).
                            // No negation needed.
                            SwerveModule[] modules = drive.getModules();
                            for (int i = 0; i < modules.length; i++) {
                                log.motor("steer-" + modules[i].getName())
                                        .voltage(Volts.of(drive.getSysIdSteerVolts()))
                                        .angularPosition(Rotations.of(
                                                modules[i].getSteerPositionRotations()))
                                        .angularVelocity(RotationsPerSecond.of(
                                                modules[i].getSteerVelocityRPS()));
                            }
                        },
                        drive
                )
        );
    }

    // ==================== SHOOTER FLYWHEEL ====================

    public static SysIdRoutine createFlywheelRoutine(Shooter shooter) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.5),  // slower ramp — flywheel spins up fast
                        Volts.of(6.0),
                        Seconds.of(8.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> shooter.setFlywheelVoltage(voltage.in(Volts)),
                        log -> log.motor("flywheel")
                                .voltage(Volts.of(shooter.getFlywheelMotorVoltage()))
                                .angularPosition(Rotations.of(0)) // not tracking position
                                .angularVelocity(RotationsPerSecond.of(
                                        shooter.getFlywheelVelocityRPS())),
                        shooter
                )
        );
    }

    // ==================== SHOOTER HOOD ====================

    public static SysIdRoutine createHoodRoutine(Shooter shooter) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.25),  // very slow — small travel range
                        Volts.of(2.0),               // low step — don't slam the hood
                        Seconds.of(5.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> shooter.setHoodVoltage(voltage.in(Volts)),
                        log -> log.motor("hood")
                                .voltage(Volts.of(shooter.getHoodMotorVoltage()))
                                .angularPosition(Rotations.of(
                                        shooter.getHoodPositionRotations()))
                                .angularVelocity(RotationsPerSecond.of(
                                        shooter.getHoodVelocityRPS())),
                        shooter
                )
        );
    }

    // ==================== SHOOTER TURRET ====================

    public static SysIdRoutine createTurretRoutine(Shooter shooter) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.5),
                        Volts.of(4.0),
                        Seconds.of(8.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> shooter.setTurretVoltage(voltage.in(Volts)),
                        log -> log.motor("turret")
                                .voltage(Volts.of(shooter.getTurretMotorVoltage()))
                                .angularPosition(Rotations.of(
                                        shooter.getTurretPositionRotations()))
                                .angularVelocity(RotationsPerSecond.of(
                                        shooter.getTurretVelocityRPS())),
                        shooter
                )
        );
    }

    // ==================== INTAKE DEPLOY ====================

    public static SysIdRoutine createIntakeDeployRoutine(Intake intake) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(0.25),  // slow — small travel range
                        Volts.of(2.0),               // low step — don't slam the arm
                        Seconds.of(5.0)
                ),
                new SysIdRoutine.Mechanism(
                        voltage -> intake.setDeployVoltage(voltage.in(Volts)),
                        log -> log.motor("intake-deploy")
                                .voltage(Volts.of(intake.getDeployMotorVoltage()))
                                .angularPosition(Rotations.of(
                                        intake.getDeployPositionRotations()))
                                .angularVelocity(RotationsPerSecond.of(
                                        intake.getDeployVelocityRPS())),
                        intake
                )
        );
    }

    // ==================== COMMAND HELPERS ====================

    public static Command quasistaticForward(SysIdRoutine routine) {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public static Command quasistaticReverse(SysIdRoutine routine) {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public static Command dynamicForward(SysIdRoutine routine) {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public static Command dynamicReverse(SysIdRoutine routine) {
        return routine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
