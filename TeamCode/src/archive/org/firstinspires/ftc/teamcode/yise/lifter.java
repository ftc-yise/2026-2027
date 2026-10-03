package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class lifter {

    public enum Mode { NEUTRAL, TARGET }

    private final CRServo motor;
    private final AnalogInput encoder;

    private Mode mode = Mode.NEUTRAL;
    private double targetPos = 0.0;

    // calibration
    private double slope = 0.0;
    private double offset = 0.0;
    private boolean calibrated = false;

    // presets
    private double DOWN_POS = 0.0;
    private double UP_POS = 1.0;

    // ---- TUNING (SAFE VALUES) ----
    private double kP = 0.85;
    private double kD = 0.002;      // VERY small
    private double holdPower = 0.04;

    private static final double DEADZONE = 0.0007;
    private static final double MAX_POWER = 0.6;

    // state
    private double lastPos = 0.0;
    private double filtVelocity = 0.0;
    private long lastTimeMs = System.currentTimeMillis();

    // telemetry
    public static class TelemetryPacket {
        public Mode mode;
        public double voltage;
        public double position;
        public double target;
        public double error;
        public double power;
        public boolean calibrated;
    }
    private final TelemetryPacket t = new TelemetryPacket();

    public lifter(HardwareMap map) {
        this(map, "lift", "analogLift");
    }

    public lifter(HardwareMap map, String liftName, String analogName) {
        motor = map.get(CRServo.class, liftName);
        encoder = map.get(AnalogInput.class, analogName);
        motor.setPower(0.0);
    }

    // ---------------- Calibration ----------------
    public boolean setCalibration(double v1, double pos1, double v2, double pos2) {
        if (Math.abs(v2 - v1) < 1e-6) return false;
        slope = (pos2 - pos1) / (v2 - v1);
        offset = pos1 - slope * v1;
        calibrated = true;

        lastPos = getPosition();
        filtVelocity = 0.0;
        lastTimeMs = System.currentTimeMillis();
        return true;
    }

    public void setPresetPositions(double down, double up) {
        DOWN_POS = clamp01(down);
        UP_POS = clamp01(up);
    }

    // ---------------- Commands ----------------
    public void setUp()   { setTarget(UP_POS); }
    public void setDown() { setTarget(DOWN_POS); }

    public void setTarget(double pos) {
        if (!calibrated) return;
        targetPos = clamp01(pos);
        mode = Mode.TARGET;

        lastPos = getPosition();
        filtVelocity = 0.0;
        lastTimeMs = System.currentTimeMillis();
    }

    public void setNeutral() {
        mode = Mode.NEUTRAL;
        motor.setPower(0.0);
    }

    // ---------------- Main Loop ----------------
    public void update() {
        double pos = getPosition();
        double error = targetPos - pos;

        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTimeMs) / 1000.0, 1e-3);

        double velocity = (pos - lastPos) / dt;
        filtVelocity = 0.7 * filtVelocity + 0.3 * velocity; // low-pass

        double power = 0.0;

        if (mode == Mode.TARGET && calibrated) {

            if (Math.abs(error) <= DEADZONE) {
                // ARRIVED — hold gently
                power = Math.signum(error) * holdPower;
            } else {
                double p = kP * error;
                double d = -kD * filtVelocity;
                power = p + d;
            }

            power = clamp(power, -MAX_POWER, MAX_POWER);
        }

        motor.setPower(power);

        lastPos = pos;
        lastTimeMs = now;

        // telemetry
        t.mode = mode;
        t.voltage = encoder.getVoltage();
        t.position = pos;
        t.target = targetPos;
        t.error = error;
        t.power = power;
        t.calibrated = calibrated;
    }

    // ---------------- Helpers ----------------
    public double getPosition() {
        if (!calibrated) return 0.0;
        return clamp01(slope * encoder.getVoltage() + offset);
    }

    public boolean isUp()   { return Math.abs(getPosition() - UP_POS) <= DEADZONE; }
    public boolean isDown() { return Math.abs(getPosition() - DOWN_POS) <= DEADZONE; }
    public boolean isBusy() { return Math.abs(targetPos - getPosition()) > DEADZONE; }

    public TelemetryPacket getTelemetry() { return t; }

    private double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }
    private double clamp(double v, double a, double b) { return Math.max(a, Math.min(b, v)); }
}
