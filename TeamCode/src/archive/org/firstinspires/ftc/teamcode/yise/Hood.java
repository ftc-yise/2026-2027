package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;

/**
 * Hood
 *
 * Angle-controlled hood using an Axon (CRServo) + analog encoder.
 * Style and telemetry modeled after Spindexer.
 *
 * IMPORTANT:
 * - Calibrate by calling setCalibrationFromTwoPoints(v1, angleDeg1, v2, angleDeg2)
 *   so the class can map encoder voltage -> hood angle (degrees).
 */
public class Hood {

    public enum Mode {
        NEUTRAL,
        MANUAL,
        TARGET
    }

    // Hardware
    private final CRServo hood;
    private final AnalogInput encoder;

    // State / mode
    public Mode mode = Mode.NEUTRAL;
    private double manualPower = 0.0;
    private boolean manualMode = false;

    // Angle state (degrees)
    private double targetAngleDeg = 0.0;

    // Calibration (linear mapping: angle = volts * slope + offset)
    // MUST BE SET using setCalibrationFromTwoPoints(...) for meaningful angles.
    private double voltsToDegSlope = 0.0;
    private double voltsToDegOffset = 0.0;
    private boolean calibrated = false;

    // Mechanical limits (degrees) - set these to your measured min/max hood angles
    private double MIN_ANGLE_DEG = 0.0;
    private double MAX_ANGLE_DEG = 60.0;

    // P / optional D control (start with P-only)
    private double kP = 0.02;   // tune me
    private double kI = 0.0;
    private double kD = 0.0;

    // Small D state
    private double lastError = 0.0;
    private long lastTimeMs = System.currentTimeMillis();

    // Rate limiting
    private double lastPower = 0.0;
    private double MAX_DELTA = 0.4;

    // Telemetry struct (mirrors Spindexer style)
    public static class TelemetryPacket {
        public Mode mode;
        public double voltage;
        public double currentAngle;
        public double targetAngle;
        public double angleError;
        public double appliedPower;
        public double manualPower;
        public double pidP;
        public double pidI;
        public double pidD;
        public boolean calibrated;
    }
    private TelemetryPacket t = new TelemetryPacket();

    // CSV logging support (optional)
    private FileWriter csv = null;
    private DecimalFormat df = new DecimalFormat("0.00");

    public Hood(HardwareMap hardwaremap) {
        hood = hardwaremap.get(CRServo.class, "hood");
        encoder = hardwaremap.get(AnalogInput.class, "hoodInput");
        hood.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ---------------- Calibration helpers ----------------

    /**
     * Provide two known points (voltage->angle) so the class can build a linear mapping.
     * Example: measure voltage at hood down (angle 10deg) and hood up (angle 40deg).
     */
    public void setCalibrationFromTwoPoints(double v1, double angleDeg1, double v2, double angleDeg2) {
        if (Math.abs(v2 - v1) < 1e-6) return;
        voltsToDegSlope = (angleDeg2 - angleDeg1) / (v2 - v1);
        voltsToDegOffset = angleDeg1 - voltsToDegSlope * v1;
        calibrated = true;
    }

    /** Optional: set mechanical limits (degrees) */
    public void setAngleLimits(double minDeg, double maxDeg) {
        MIN_ANGLE_DEG = minDeg;
        MAX_ANGLE_DEG = maxDeg;
    }

    /** Convert measured analog voltage -> hood angle degrees (uses linear mapping) */
    public double voltageToAngleDeg(double v) {
        if (!calibrated) return 0.0;
        return voltsToDegSlope * v + voltsToDegOffset;
    }

    // ---------------- Mode commands ----------------

    public void setNeutral() {
        mode = Mode.NEUTRAL;
        manualMode = false;
    }

    public void setManual(double power) {
        manualPower = clamp(power, -1.0, 1.0);
        manualMode = true;
        mode = Mode.MANUAL;
    }

    public void setTarget(double angleDeg) {
        // clamp to mechanical limits immediately
        targetAngleDeg = clamp(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
        mode = Mode.TARGET;
        manualMode = false;
    }

    public void goToMin() { hood.setPower(-1); }
    public void goToMax() { hood.setPower(1); }

    // ---------------- Update loop — call every loop ----------------
    public void update() {
        double voltage = encoder.getVoltage();
        double currentAngle = voltageToAngleDeg(voltage);

        double error = targetAngleDeg - currentAngle;

        double power;
        if (manualMode) {
            power = manualPower;
        } else if (mode == Mode.NEUTRAL) {
            power = 0.0;
        } else { // TARGET mode
            power = pControl(error);
        }

        power = rateLimit(power);
        hood.setPower(power);

        // Telemetry population
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = currentAngle;
        t.targetAngle = targetAngleDeg;
        t.angleError = error;
        t.appliedPower = power;
        t.manualPower = manualPower;

        t.pidP = kP * error;
        t.pidI = 0.001;
        t.pidD = 0;
        t.calibrated = calibrated;

        // Optional CSV (if csv != null)
        writeCSV(currentAngle, targetAngleDeg, error, power);
    }

    // ---------------- P-only control (with D placeholder) ----------------
    private double pControl(double error) {
        long now = System.currentTimeMillis();
        double dt = (now - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 1e-3;

        double pTerm = kP * error;
        double dTerm = 0.0;
        if (kD != 0.0) {
            dTerm = kD * ((error - lastError) / dt);
        }

        lastError = error;
        lastTimeMs = now;

        double out = pTerm + dTerm;
        return clamp(out, -1.0, 1.0);
    }

    // ---------------- Rate limiter ----------------
    private double rateLimit(double p) {
        double diff = p - lastPower;
        if (Math.abs(diff) > MAX_DELTA) {
            p = lastPower + Math.signum(diff) * MAX_DELTA;
        }
        lastPower = p;
        return clamp(p, -1.0, 1.0);
    }

    // ---------------- CSV logger ----------------
    public void enableCSV(String path) {
        try {
            csv = new FileWriter(path, true);
            csv.write("time_ms,angle,target,error,power\n");
            csv.flush();
        } catch (IOException ignored) {}
    }

    private void writeCSV(double angle, double target, double error, double power) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(power) + "\n");
            csv.flush();
        } catch (IOException ignored) {}
    }

    // ---------------- Telemetry getter ----------------
    public TelemetryPacket getTelemetry() { return t; }

    // ---------------- Utilities ----------------
    private double clamp(double v, double a, double b) {
        return Math.max(a, Math.min(b, v));
    }

    // ---------------- Configuration setters (tuning) ----------------
    public void setPID(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void setRateLimit(double maxDelta) {
        // maxDelta should be small (e.g., 0.02..0.08)
        // This prevents jerky motion on the hood motor
        // but may limit responsiveness if too small.
        // Keep > 0.
        // Example: setRateLimit(0.04);
        if (maxDelta > 0) this.MAX_DELTA = maxDelta;
    }

    // Expose some useful getters
    public double getTargetAngleDeg() { return targetAngleDeg; }
    public double getCurrentAngleDeg() { return voltageToAngleDeg(encoder.getVoltage()); }

    /**
     * Stop and neutralize the hood motor.
     */
    public void stop() {
        hood.setPower(0.0);
        mode = Mode.NEUTRAL;
        manualMode = false;
    }
}
