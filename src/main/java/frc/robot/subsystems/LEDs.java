package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.LEDConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure.SuperState;

import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem for Einstein-grade driver communication.
 *
 * <h2>Why this matters:</h2>
 * The driver can't read SmartDashboard mid-match. LEDs instantly
 * communicate robot state: ready to shoot, intaking, jammed, low battery.
 * Every Einstein team has this.
 *
 * <h2>Priority system (highest → lowest):</h2>
 * <ol>
 *   <li>FAULT / JAM — red flash (driver must react immediately)</li>
 *   <li>LOW BATTERY — orange pulse (pit crew needs to know)</li>
 *   <li>READY TO SHOOT — solid green (driver can press shoot)</li>
 *   <li>INTAKING — blue chase pattern</li>
 *   <li>SHOOTING — green strobe</li>
 *   <li>DISABLED — alliance color breathe</li>
 *   <li>IDLE — alliance color solid</li>
 * </ol>
 */
public class LEDs extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static LEDs instance;

    public static void initialize() {
        if (instance != null) throw new IllegalStateException("LEDs already initialized.");
        instance = new LEDs();
    }

    public static LEDs getInstance() {
        if (instance == null) throw new IllegalStateException("LEDs not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== HARDWARE ====================
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int ledCount;

    // ==================== STATE SUPPLIERS ====================
    private final Superstructure superstructure;
    private final Shooter shooter;
    private final Vision vision;

    // ==================== ANIMATION ====================
    private int chaseOffset = 0;

    // ==================== TELEMETRY ====================
    /** Current LED pattern name — logged each cycle for AdvantageScope replay. */
    private String currentPattern = "IDLE";

    private static final int kLedPort = LEDConstants.kLedPort;
    private static final int kLedCount = LEDConstants.kLedCount;

    private LEDs() {
        this.superstructure = Superstructure.getInstance();
        this.shooter = Shooter.getInstance();
        this.vision = Vision.getInstance();
        this.ledCount = kLedCount;

        led = new AddressableLED(kLedPort);
        buffer = new AddressableLEDBuffer(kLedCount);
        led.setLength(kLedCount);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        RobotState rs = RobotState.getInstance();

        // ---- Priority 1: Fault / Jam ----
        if (!rs.isAllHealthy()) {
            flash(Color.kRed, now, 5.0); // fast red flash
            applyAndLog("FAULT");
            return;
        }

        // ---- Priority 2: Low battery ----
        if (rs.isLowBattery()) {
            pulse(Color.kOrangeRed, now, 2.0);
            applyAndLog("LOW_BATTERY");
            return;
        }

        // ---- Priority 1.5: Vision cameras disconnected ----
        if (vision.getConnectedAprilTagCameraCount() < 2) {
            alternating(Color.kCyan, Color.kRed, now, 3.0); // cyan/red alternating
            applyAndLog("VISION_DISCONNECTED");
            return;
        }

        // ---- Priority 2.3: Turret at hard limit (MUST rotate NOW) ----
        if (shooter.isTurretAtHardLimit()) {
            strobe(Color.kOrange, now, 12.0); // rapid amber strobe
            applyAndLog("TURRET_HARD_LIMIT");
            return;
        }

        // ---- Priority 2.5: Brownout predicted (imminent) ----
        if (rs.isBrownoutPredicted()) {
            strobe(Color.kOrange, now, 8.0); // fast orange strobe = brownout imminent
            applyAndLog("BROWNOUT_PREDICTED");
            return;
        }

        // ---- Priority 2.7: Turret near limit (driver should rotate) ----
        if (shooter.isTurretNearLimit()) {
            pulse(Color.kOrange, now, 4.0); // amber pulse
            applyAndLog("TURRET_NEAR_LIMIT");
            return;
        }

        // ---- Priority 3+: State-dependent ----
        if (DriverStation.isDisabled()) {
            // Alliance color breathe
            Color allianceColor = getAllianceColor();
            breathe(allianceColor, now, 1.5);
            currentPattern = "DISABLED_BREATHE";
        } else {
            SuperState state = superstructure.getCurrentState();

            switch (state) {
                case SHOOTING:
                case SHOOTING_ALLIANCE:
                    if (shooter.isReadyToShoot()) {
                        if (superstructure.getCurrentState() == SuperState.SHOOTING_ALLIANCE) {
                            strobe(Color.kPurple, now, 10.0); // purple strobe = alliance zone dump
                            currentPattern = "SHOOTING_ALLIANCE";
                        } else {
                            strobe(Color.kGreen, now, 10.0); // green strobe = locked on + feeding
                            currentPattern = "SHOOTING_LOCKED";
                        }
                    } else {
                        pulse(Color.kYellow, now, 3.0); // yellow pulse = waiting for lock
                        currentPattern = "SHOOTING_ACQUIRING";
                    }
                    break;

                case INTAKING:
                    if (vision.isBallDetected()) {
                        flash(Color.kCyan, now, 6.0); // cyan flash = vision sees ball
                        currentPattern = "INTAKING_BALL_VISIBLE";
                    } else {
                        chase(Color.kBlue, Color.kBlack, now);
                        currentPattern = "INTAKING";
                    }
                    break;

                case OUTTAKING:
                    chase(Color.kOrange, Color.kBlack, now);
                    currentPattern = "OUTTAKING";
                    break;

                case UNJAMMING:
                    flash(Color.kRed, now, 3.0);
                    currentPattern = "UNJAMMING";
                    break;

                case IDLE:
                default:
                    if (shooter.isTrackingEnabled() && shooter.isReadyToShoot() && shooter.isInShootingRange()) {
                        solid(Color.kGreen); // tracking + locked = ready to shoot
                        currentPattern = "IDLE_READY";
                    } else if (shooter.isTrackingEnabled() && !shooter.isReadyToShoot()) {
                        pulse(Color.kMagenta, now, 3.0); // tracking on but not locked yet
                        currentPattern = "IDLE_ACQUIRING";
                    } else if (shooter.isInShootingRange()) {
                        pulse(Color.kGreen, now, 2.0); // in range, not tracking
                        currentPattern = "IDLE_IN_RANGE";
                    } else {
                        solid(getAllianceColor()); // alliance color solid
                        currentPattern = "IDLE";
                    }
                    break;
            }
        }

        applyAndLog(currentPattern);
    }

    /**
     * Applies the LED buffer to the hardware strip and logs the current pattern name
     * + first LED RGB color to AdvantageKit for replay visibility.
     */
    private void applyAndLog(String patternName) {
        this.currentPattern = patternName;
        led.setData(buffer);
        Color firstLED = buffer.getLED(0);
        Logger.recordOutput("LEDs/Pattern", patternName);
        Logger.recordOutput("LEDs/ColorR", (int) (firstLED.red   * 255));
        Logger.recordOutput("LEDs/ColorG", (int) (firstLED.green * 255));
        Logger.recordOutput("LEDs/ColorB", (int) (firstLED.blue  * 255));
    }

    // ==================== PATTERNS ====================

    private void solid(Color color) {
        for (int i = 0; i < ledCount; i++) {
            buffer.setLED(i, color);
        }
    }

    private void flash(Color color, double now, double hz) {
        boolean on = ((int) (now * hz * 2)) % 2 == 0;
        solid(on ? color : Color.kBlack);
    }

    private void pulse(Color color, double now, double hz) {
        double brightness = (Math.sin(now * hz * 2 * Math.PI) + 1.0) / 2.0;
        Color dimmed = new Color(
                color.red * brightness,
                color.green * brightness,
                color.blue * brightness);
        solid(dimmed);
    }

    private void breathe(Color color, double now, double hz) {
        // Triangle wave for smoother breathing
        double phase = (now * hz) % 1.0;
        double brightness = phase < 0.5 ? phase * 2.0 : 2.0 - phase * 2.0;
        brightness = brightness * 0.8 + 0.05; // never fully off
        Color dimmed = new Color(
                color.red * brightness,
                color.green * brightness,
                color.blue * brightness);
        solid(dimmed);
    }

    private void strobe(Color color, double now, double hz) {
        // Alternating segments flash — visually distinct from full-strip flash()
        int segmentSize = 3;
        boolean phase = ((int) (now * hz * 2)) % 2 == 0;
        for (int i = 0; i < ledCount; i++) {
            boolean inEvenSegment = (i / segmentSize) % 2 == 0;
            boolean on = inEvenSegment == phase;
            buffer.setLED(i, on ? color : Color.kBlack);
        }
    }

    private void chase(Color foreground, Color background, double now) {
        int chaseLength = 5;
        chaseOffset = ((int) (now * 20)) % ledCount; // 20 LEDs/sec speed

        for (int i = 0; i < ledCount; i++) {
            if ((i + chaseOffset) % (chaseLength * 2) < chaseLength) {
                buffer.setLED(i, foreground);
            } else {
                buffer.setLED(i, background);
            }
        }
    }

    private void alternating(Color color1, Color color2, double now, double hz) {
        boolean phase = ((int) (now * hz * 2)) % 2 == 0;
        for (int i = 0; i < ledCount; i++) {
            boolean even = (i % 2 == 0);
            buffer.setLED(i, (even == phase) ? color1 : color2);
        }
    }

    // ==================== HELPERS ====================

    private Color getAllianceColor() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red
                    ? Color.kRed : Color.kBlue;
        }
        return Color.kGold; // no alliance = gold
    }
}
