package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.yise.Parameters;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Spindexer — positional-servo version (pose-driven for silos)
 *
 * This subsystem controls the spindexer servo that rotates the robot’s
 * silo mechanism to specific positions.
 *
 * SILO POSITIONS:
 *  - Silo1 -> 0.0
 *  - Silo2 -> 0.5
 *  - Silo3 -> 1.0
 *
 * Other behavior preserved:
 * - sensor reading / color detection
 * - logging
 * - manual cycle
 *
 * TARGET mode (setTarget(angle)) still maps 0..360° -> 0..1 as a fallback,
 * but direct silo commands bypass angle math entirely and go to fixed positions.
 */
public class Spindexer {

    // ─────────────────────────────────────────────────────────────────────
    // MODES / STATE STORAGE
    // These fields track what the spindexer currently believes each silo contains,
    // along with some historical tracking for sensor ownership and filtering.
    // ─────────────────────────────────────────────────────────────────────
    public BallColor[] siloColors = new BallColor[3];
    private int[] lastSeenSiloBySensor = {-1, -1, -1};

    // smoothing / filtering (position-specific)
    private double lastPositionFiltered = 0.0;

    /**
     * Operational modes for the spindexer.
     *
     * NEUTRAL = do not actively move to a fixed target
     * SILO_1 / SILO_2 / SILO_3 = move directly to one of the preset silo poses
     * MANUAL = cycle through fixed poses on a timer
     * TARGET = move to an arbitrary target position or angle fallback
     * SEQUENCE = run a fixed multi-step sequence
     */
    public enum Mode {
        NEUTRAL,
        SILO_1,
        SILO_2,
        SILO_3,
        MANUAL,     // cycles through fixed poses
        TARGET,     // map arbitrary angle -> position (fallback)
        SEQUENCE
    }

    /**
     * Color classification for what a silo currently contains.
     *
     * GREEN and PURPLE are the two detectable ball colors.
     * NONE means no usable ball is detected.
     */
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    // Current detected color state for each silo slot
    private final BallColor[] silos = {
            BallColor.NONE,
            BallColor.NONE,
            BallColor.NONE
    };

    // Shared silo angle variables used by older logic / initialization
    public static double silo1;
    public static double silo2;
    public static double silo3;

    // Hardcoded target angles for the three named silos
    private static final double SILO_1_TARGET_DEG = 303.5;
    private static final double SILO_2_TARGET_DEG = 183.5;
    private static final double SILO_3_TARGET_DEG = 60.5;

    // Keep existing angle-based silo definitions for compatibility
    // These are still used by color matching and initSilos().
    public static double[] SILO_ANGLES = {
            silo1,   // SILO_1
            silo2,   // SILO_2
            silo3    // SILO_3
    };

    // Fixed servo positions for the three silos
    // These are the actual positions used by the new positional-servo behavior.
    public static double[] SILO_POSITIONS = {
            0.0,   // SILO_1 -> servo position 0.0
            0.5,   // SILO_2 -> servo position 0.5
            1.0    // SILO_3 -> servo position 1.0
    };

    // Sensor offsets used to map each color sensor into the spindexer's world angle space
    private static final double[] SENSOR_OFFSETS = {
            0.0,    // middle
            120.0,  // backLeft
            240.0   // backRight
    };

    // Current mode of the spindexer
    public Mode mode = Mode.NEUTRAL;

    // ─────────────────────────────────────────────────────────────────────
    // HARDWARE
    // ─────────────────────────────────────────────────────────────────────

    // Positional servo that physically rotates the spindexer
    private Servo spindexer;

    // Analog encoder input used to estimate current spindexer angle
    private AnalogInput encoder;

    // Color sensors on the robot
    private ColorSensor middleT = null;
    private ColorSensor backLeftT = null;
    private ColorSensor backRightT = null;
    private ColorSensor middleB = null;
    private ColorSensor backLeftB = null;
    private ColorSensor backRightB = null;

    // Angle variables (retained for fallback / compatibility with older logic)
    public double targetAngleDeg = 0;

    // Direct target servo position in the range 0..1
    // Used by silo modes, manual mode, and pose-based fallback.
    private double targetPosition = 0.0;

    // Encoder voltage range used to convert raw voltage to 0..360 degrees
    private final double MAX_VOLTAGE = 3.3;

    // Controls whether color sensor updates are allowed right now
    // This is useful during shooting so sensor state doesn’t change mid-cycle.
    private boolean sensorUpdatesEnabled = true;

    // Angle/velocity helpers kept from the earlier implementation
    private double lastAngle = 0.0;            // last measured angle (deg)
    private long lastTimeMs = System.currentTimeMillis();

    // Position rate limiting to keep servo movement smooth
    private double lastPosition = 0.0;
    private final double MAX_POSITION_DELTA = 0.12; // maximum allowed position change per loop

    // Sequence state for multi-step automatic behavior
    private int siloStep = 0;
    private boolean sequenceActive = false;

    // Manual cycling state: automatically rotates through fixed poses over time
    private boolean manualCycleActive = false;
    private int manualCycleIndex = 0;
    private long lastManualCycleMs = 0;
    private static final long MANUAL_CYCLE_MS = 700; // ms between pose switches in manual cycle

    // Angle tolerance used by older angle-based matching logic
    private final double ANGLE_TOLERANCE = 0.5;
    private final double ANGLE_TOLERANCE_COLOR = 15;

    // Timer used by some control paths
    private final ElapsedTime timer = new ElapsedTime();

    // Position arrival tolerance for "am I close enough?" checks
    private final double POSITION_TOLERANCE = 0.02; // ~2% of travel

    // Arrival latch: once the spindexer reaches the target, this can stay true
    private boolean atTargetLatched = false;

    // ─────────────────────────────────────────────────────────────────────
    // TELEMETRY STRUCT
    // This object is used to expose the current spindexer state to other code
    // without requiring the caller to read hardware directly.
    // ─────────────────────────────────────────────────────────────────────
    public static class TelemetryPacket {
        public Mode mode;
        public double voltage;
        public double currentAngle;
        public double targetAngle;
        public double angleError;
        public double appliedPower;   // repurposed as applied position (0..1)
        public double manualPower;
        public double pidP;
        public double pidI;
        public double pidD;
        public BallColor[] siloColors = new BallColor[3];
    }

    // Current telemetry snapshot
    private TelemetryPacket t = new TelemetryPacket();

    // CSV logging support
    private FileWriter csv;
    private DecimalFormat df = new DecimalFormat("0.00");

    // Constructor: binds hardware and initializes the spindexer to a known position
    public Spindexer(HardwareMap hardwaremap) {
        spindexer = hardwaremap.get(Servo.class, "spin");
        encoder = hardwaremap.get(AnalogInput.class, "spinInput");

        // Top color sensors
        middleT = hardwaremap.get(ColorSensor.class, "middlecolorsensorT");
        backLeftT = hardwaremap.get(ColorSensor.class, "BLcolorsensorT");
        backRightT = hardwaremap.get(ColorSensor.class, "BRcolorsensorT");

        // Bottom color sensors
        middleB = hardwaremap.get(ColorSensor.class, "middlecolorsensorB");
        backLeftB = hardwaremap.get(ColorSensor.class, "BLcolorsensorB");
        backRightB = hardwaremap.get(ColorSensor.class, "BRcolorsensorB");

        // Initialize angle estimate from encoder so the first motion estimate is not extreme
        double v = encoder.getVoltage();
        lastAngle = normalize((v / MAX_VOLTAGE) * 360.0);
        lastPositionFiltered = angleToPosition(lastAngle);     // initialize position filter
        lastTimeMs = System.currentTimeMillis();

        // Initialize servo at the current filtered position
        spindexer.setPosition(lastPositionFiltered);
        lastPosition = lastPositionFiltered;
        targetPosition = lastPositionFiltered;
    }

    // ─────────────────────────────────────────────────────────────────────
    // MODE COMMANDS
    // These methods are the primary control API for the rest of the robot code.
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Put the spindexer into a passive resting state.
     */
    public void setNeutral() {
        mode = Mode.NEUTRAL;
        sequenceActive = false;
        manualCycleActive = false;
    }

    // direct silo commands: set the fixed position and mode
    /**
     * Move directly to silo 1 using a fixed servo position.
     */
    public void goToSilo1() {
        targetPosition = SILO_POSITIONS[0];
        targetAngleDeg = SILO_1_TARGET_DEG;
        mode = Mode.SILO_1;
    }

    /**
     * Move directly to silo 2 using a fixed servo position.
     */
    public void goToSilo2() {
        targetPosition = SILO_POSITIONS[1];
        targetAngleDeg = SILO_2_TARGET_DEG;
        mode = Mode.SILO_2;
    }

    /**
     * Move directly to silo 3 using a fixed servo position.
     */
    public void goToSilo3() {
        targetPosition = SILO_POSITIONS[2];
        targetAngleDeg = SILO_3_TARGET_DEG;
        mode = Mode.SILO_3;
    }

    /**
     * Start a non-blocking manual cycle through poses:
     * Silo1 -> Silo2 -> Silo3 -> repeat.
     *
     * This method does not block program execution; it only updates the target
     * and lets update() continue the motion over time.
     */
    public void startManualCycle() {
        double t = timer.seconds();
        if (t < 1.5) {
            goToSilo1();
        } else if (t < 2.5) {
            goToSilo3();
        } else {
            goToSilo2();
            if (t > 4) {
                timer.reset();
            }
        }

        mode = Mode.MANUAL;
    }

    /**
     * Stops the manual pose cycle and returns to neutral mode.
     */
    public void stopManualCycle() {
        manualCycleActive = false;
        mode = Mode.NEUTRAL;
    }

    /**
     * Set an arbitrary angle target.
     *
     * This is retained for backward compatibility or fallback use cases where
     * the system wants angle-based targeting instead of a direct silo pose.
     */
    public void setTarget(double angleDeg) {
        targetAngleDeg = normalize(angleDeg);
        mode = Mode.TARGET;
    }

    /**
     * Directly command a servo pose in the range 0..1.
     *
     * This is useful for manual placement or preset positions that do not
     * require angle math.
     */
    public void goToPose(double val) {
        targetPosition = Range.clip(val, 0.0, 1.0);
        mode = Mode.TARGET; // use TARGET so update() writes the mapped pos (we check targetPosition first)
    }

    // ─────────────────────────────────────────────────────────────────────
    // AUTOMATIC SILO SEQUENCE
    // This is a fixed multi-step pattern of silo movements.
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Starts the automated three-step silo sequence.
     */
    public void startSequence() {
        siloStep = 0;
        sequenceActive = true;
        mode = Mode.SEQUENCE;
    }

    /**
     * Executes one step of the automatic sequence.
     *
     * The sequence goes:
     *   step 0 -> Silo 2
     *   step 1 -> Silo 3
     *   step 2 -> Silo 1
     *   step 3 -> finish
     */
    private void runSequence() {
        if (!sequenceActive) return;

        switch (siloStep) {
            case 0:
                goToSilo2();           // now uses fixed position 0.5
                if (atTarget()) siloStep++;
                break;

            case 1:
                goToSilo3();           // now uses fixed position 1.0
                if (atTarget()) siloStep++;
                break;

            case 2:
                goToSilo1();           // now uses fixed position 0.0
                if (atTarget()) siloStep++;
                break;

            case 3:
                sequenceActive = false;
                setNeutral();
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // UPDATE LOOP — CALL EVERY LOOP
    //
    // This is the core runtime method.
    // It:
    // 1. reads the encoder
    // 2. updates silo color readings
    // 3. runs any active sequence / manual cycle
    // 4. computes the output servo position
    // 5. updates telemetry and logging
    // ─────────────────────────────────────────────────────────────────────
    public void update() {

        // --- read encoder / current angle ---
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // ALWAYS update sensors (telemetry must never starve)
        if (sensorUpdatesEnabled) {
            mapSensorsToSilos(current);
        }

        // Run sequence logic if sequence mode is active
        if (mode == Mode.SEQUENCE) runSequence();

        // Manual cycling handling (non-blocking): advance to next silo every MANUAL_CYCLE_MS
        if (manualCycleActive) {
            long now = System.currentTimeMillis();
            if (now - lastManualCycleMs >= MANUAL_CYCLE_MS) {
                manualCycleIndex = (manualCycleIndex + 1) % 3;
                targetPosition = SILO_POSITIONS[manualCycleIndex];
                lastManualCycleMs = now;
            }
        }

        double appliedPosition;

        // Mode -> position mapping (silo modes directly map to fixed positions)
        if (mode == Mode.NEUTRAL) {
            appliedPosition = angleToPosition(current); // hold current mapped position
            atTargetLatched = false;
        } else if (mode == Mode.SILO_1) {
            appliedPosition = SILO_POSITIONS[0];
        } else if (mode == Mode.SILO_2) {
            appliedPosition = SILO_POSITIONS[1];
        } else if (mode == Mode.SILO_3) {
            appliedPosition = SILO_POSITIONS[2];
        } else if (mode == Mode.MANUAL) {
            appliedPosition = targetPosition;
        } else if (mode == Mode.SEQUENCE) {
            // sequence uses goToSiloX() so targetPosition already set appropriately
            appliedPosition = targetPosition;
        } else { // Mode.TARGET -> fallback mapping from angle to position
            // If a direct targetPosition was set (via goToPose or similar), prefer it
            appliedPosition = targetPosition;
            // If targetPosition left at default or invalid, fallback to angle mapping
            // (we initially set targetPosition to lastPositionFiltered, so this branch uses the explicit pos)
            if (Double.isNaN(appliedPosition) || appliedPosition < 0.0 || appliedPosition > 1.0) {
                appliedPosition = angleToPosition(targetAngleDeg);
            }
        }

        // Smooth / rate-limit position changes, then apply them to the servo
        double outputPosition = positionRateLimit(appliedPosition);
        spindexer.setPosition(outputPosition);

        // arrival latch (position-based)
        double posErr = Math.abs(outputPosition - lastPositionFiltered);
        if (posErr < POSITION_TOLERANCE) {
            atTargetLatched = true;
        } else {
            atTargetLatched = false;
        }

        // --- telemetry packet ---
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = smallestAngleError(targetAngleDeg, current);
        t.appliedPower = outputPosition; // repurposed as "position"
        t.manualPower = 0.0;

        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }

        writeCSV(current, targetAngleDeg, t.angleError);

        if (mode == Mode.NEUTRAL) {
            Arrays.fill(lastSeenSiloBySensor, -1);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // POSITION / ANGLE CONVERSION
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Convert an angle in degrees (0..360) to a servo position (0..1).
     *
     * This is used as a fallback when the system is in TARGET mode and does
     * not have a named silo target.
     */
    private double angleToPosition(double angleDeg) {
        double normalized = normalize(angleDeg); // 0..360
        return normalized / 360.0;
    }

    /**
     * Rate-limit servo position changes so motion is smoother and less abrupt.
     *
     * This prevents large sudden jumps from happening in a single loop.
     */
    private double positionRateLimit(double requested) {
        double diff = requested - lastPosition;
        double allowed = MAX_POSITION_DELTA;
        if (Math.signum(requested) != Math.signum(lastPosition)) {
            // allow quicker reversal proportionally (not strictly needed but safe)
            allowed *= 4.0;
        }
        if (Math.abs(diff) > allowed) {
            requested = lastPosition + Math.signum(diff) * allowed;
        }
        lastPosition = requested;
        // filter for smoother output
        lastPositionFiltered = 0.55 * lastPositionFiltered + (1.0 - 0.55) * requested;
        return Range.clip(lastPositionFiltered, 0.0, 1.0);
    }

    // ─────────────────────────────────────────────────────────────────────
    // SENSOR MAPPING
    // These methods assign detected ball colors into the correct silo slots.
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Maps the current sensor readings into silo color slots.
     *
     * Each sensor is associated with a world-angle offset. The current spindexer
     * angle is used to determine which silo each sensor is "looking at."
     */
    private void mapSensorsToSilos(double currentAngle) {
        ColorSensor[] sensors = { middleB, backLeftB, backRightB };

        for (int sensorIdx = 0; sensorIdx < sensors.length; sensorIdx++) {
            ColorSensor s = sensors[sensorIdx];
            // compute the world angle that this sensor is "looking at"
            double sensorAngle = normalize(currentAngle + SENSOR_OFFSETS[sensorIdx]);

            // find nearest silo index for that angle (with hysteresis fallback)
            int siloIndex = angleToSilo(sensorAngle);
            if (siloIndex == -1) {
                int prev = lastSeenSiloBySensor[sensorIdx];
                if (prev != -1) {
                    double prevErr = Math.abs(smallestAngleError(SILO_ANGLES[prev], sensorAngle));
                    if (prevErr < ANGLE_TOLERANCE * 3.0) siloIndex = prev;
                }
            }

            if (siloIndex != -1) {
                // write the sensor reading into the silo slot
                silos[siloIndex] = detectBall(s, sensorIdx);
                lastSeenSiloBySensor[sensorIdx] = siloIndex;
            } else {
                lastSeenSiloBySensor[sensorIdx] = -1;
            }
        }
    }

    /**
     * Reads the sensors immediately and writes the detected colors into the silo array.
     *
     * This should only be called when the spindexer is aligned at a known reading position.
     */
    public void sampleSensorsNow() {
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // map sensors -> silos
        mapSensorsToSilos(current);

        // copy into telemetry packet so callers reading getTelemetry() see the update immediately
        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // HELPER METHODS
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Normalizes any angle into the range 0..360.
     */
    private static double normalize(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    /**
     * Returns whether the spindexer has reached its intended target.
     */
    private boolean atTarget() {
        return atTargetLatched;
    }

    /**
     * Computes the shortest signed difference between two angles.
     * Useful for comparing angles near wrap-around points like 0/360.
     */
    private double smallestAngleError(double target, double current) {
        double diff = target - current;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    /**
     * Clamps a value between a minimum and maximum.
     */
    private double clamp(double p, double min, double max) {
        return Math.max(min, Math.min(max, p));
    }

    // ─────────────────────────────────────────────────────────────────────
    // CSV LOGGER
    // Writes basic telemetry to a CSV file if logging is enabled.
    // ─────────────────────────────────────────────────────────────────────
    private void writeCSV(double angle, double target, double error) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(t.pidP) + ",0,0,");
            csv.flush();
        } catch (IOException ignored) {}
    }

    /**
     * Returns the most recent telemetry snapshot.
     */
    public TelemetryPacket getTelemetry() { return t; }

    /**
     * Stops motion and returns the system to neutral.
     */
    public void stop() {
        // stop motion: set neutral and hold current position
        spindexer.setPosition(lastPosition); // hold last position
        mode = Mode.NEUTRAL;
        manualCycleActive = false;
        sequenceActive = false;
    }

    /**
     * Detects whether a sensor sees a GREEN, PURPLE, or NONE ball.
     *
     * Sensor thresholds differ slightly by sensor location because the
     * sensors may have different lighting / viewing angles.
     */
    private BallColor detectBall(ColorSensor s, int sensorIndex) {
        int g = s.green();
        int b = s.blue();

        if (sensorIndex == 0) { // middle sensor
            if (b > 350 && b > g) return BallColor.PURPLE;
            else if (g > 300) return BallColor.GREEN;
            return BallColor.NONE;
        } else if (sensorIndex == 1) { // backLeft
            if (b > 225  && b > g) return BallColor.PURPLE;
            else if (g > 350) return BallColor.GREEN;
            return BallColor.NONE;
        } else { // sensorIndex == 2 -> backRight
            if (b > 175  && b > g) return BallColor.PURPLE;
            if (g > 250) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }

    /**
     * Clears a silo’s stored color state.
     */
    public void clearSilo(int index) {
        if (index >= 0 && index < silos.length) {
            silos[index] = BallColor.NONE;
        }
    }

    /**
     * Finds which silo angle the given angle most closely matches.
     */
    private int angleToSilo(double angle) {
        for (int i = 0; i < 3; i++) {
            double error = smallestAngleError(SILO_ANGLES[i], angle);
            if (Math.abs(error) < ANGLE_TOLERANCE_COLOR) return i;
        }
        return -1;
    }

    /**
     * Enables normal sensor updates during update().
     */
    public void enableSensorUpdates() {
        sensorUpdatesEnabled = true;
    }

    /**
     * Disables sensor updates during a cycle when readings should stay stable.
     */
    public void disableSensorUpdates() {
        sensorUpdatesEnabled = false;
    }

    /**
     * Convenience method to send the system to one of the three silo presets.
     */
    public void goToSilo(int index) {
        switch (index) {
            case 0: goToSilo1(); break;
            case 1: goToSilo2(); break;
            case 2: goToSilo3(); break;
        }
    }

    /**
     * Recomputes silo angles relative to the robot’s current spin location.
     *
     * This preserves the older angle-based logic so color matching and
     * compatibility features keep working.
     */
    public void initSilos() {
        silo1 = normalize(Parameters.spinLocation - 120);
        silo2 = normalize(Parameters.spinLocation);
        silo3 = normalize(Parameters.spinLocation + 120);

        // keep your existing angle array updated (not used for direct silo commands)
        SILO_ANGLES = new double[]{
                silo1,   // SILO_1
                silo2,   // SILO_2
                silo3    // SILO_3
        };
    }

}