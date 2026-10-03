package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ShooterClass
 *
 * REV Ultraplanetary direct-drive shooter (1:1)
 * Closed-loop RPM control using REV firmware velocity PIDF
 * Includes optional battery voltage compensation
 */
public class ShooterClass {

    // ======================================================
    // CONSTANTS (DIRECT DRIVE MODEL)
    // ======================================================
    private static final double TICKS_PER_REV = 24;
    private static final double MAX_RPM = 5450.0;

    private static final double MAX_TICKS_PER_SEC =
            (MAX_RPM * TICKS_PER_REV) / 60.0;

    // Feedforward base (REV standard)
    private static final double BASE_F = 32767.0 / MAX_TICKS_PER_SEC + .005;

    // ======================================================
    // PIDF (REV VELOCITY CONTROLLER)
    // ======================================================
    private double kP = 0.0082;
    private double kI = 0.00;
    private double kD = 0.000;
    private double kF = BASE_F;

    // ======================================================
    // BATTERY VOLTAGE COMPENSATION
    // ======================================================
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double VOLTAGE_UPDATE_THRESHOLD = 0.15;

    private VoltageSensor battery;
    private double lastVoltage = NOMINAL_VOLTAGE;

    // ======================================================
    // READY-TO-FIRE THRESHOLDS (RPM ERROR)
    // ======================================================
    private static final double READY_LOOSE   = 150;
    private static final double READY_NORMAL  = 75;
    private static final double READY_TIGHT   = 40;
    private static final double READY_SNIPER  = 20;

    // ======================================================
    // MODES
    // ======================================================
    public enum ShooterMode {
        STOP,
        IDLE,
        LOW,
        FULL,
        AUTO_DISTANCE
    }

    private ShooterMode mode = ShooterMode.STOP;

    // ======================================================
    // HARDWARE
    // ======================================================
    private DcMotorEx shooter;

    // ======================================================
    // STATE
    // ======================================================
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double rpmError = 0.0;

    private boolean readyLoose = false;
    private boolean readyNormal = false;
    private boolean readyTight = false;
    private boolean readySniper = false;

    // Spin-up profiling
    public ElapsedTime spinupTimer = new ElapsedTime();
    private double spinupTimeSec = 0.0;
    private boolean spinningUp = false;

    /*
     * NOTE:
     * These were previously used for auto PID tuning.
     * Auto-tuning was removed because REV firmware PID
     * should not be live-modified during runtime.
     *
     * They are intentionally left here (commented)
     * to preserve structure and intent.
     */
    // private double lastError = 0.0;
    // private double stableTimer = 0.0;

    // ======================================================
    // TELEMETRY PACKET
    // ======================================================
    public static class ShooterTelemetry {
        public ShooterMode mode;
        public double targetRPM;
        public double currentRPM;
        public double errorRPM;

        public boolean readyLoose;
        public boolean readyNormal;
        public boolean readyTight;
        public boolean readySniper;

        public double spinupTimeSec;

        public double kP, kI, kD, kF;
        public double motorPower;
        public double batteryVoltage;

        public double pose;


    }

    private final ShooterTelemetry t = new ShooterTelemetry();

    // ======================================================
    // CONSTRUCTOR
    // ======================================================
    public ShooterClass(HardwareMap hw) {

        shooter = hw.get(DcMotorEx.class, "shoot");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        battery = hw.voltageSensor.iterator().next();

        applyPIDF(); // IMPORTANT: actually applies kF to motor
    }

    // ======================================================
    // MAIN UPDATE
    // ======================================================
    public void update(boolean stop, boolean low, boolean full) {

        // Mode selection
        if (stop) mode = ShooterMode.STOP;
        else if (full) mode = ShooterMode.FULL;
        else if (low) mode = ShooterMode.LOW;
        else mode = ShooterMode.IDLE;

        // Target RPM logic
        switch (mode) {
            case STOP:
                targetRPM = 0;
                break;

            case IDLE:
                targetRPM = 3650;
                break;

            case LOW:
                targetRPM = 3000;
                break;

            case FULL:
                targetRPM = 3650;
                break;
        }

        applyVelocityControl();
        updateTelemetry();
    }

    // ======================================================
    // RPM CONTROL (REV VELOCITY MODE)
    // ======================================================
    private void applyVelocityControl() {

        // Update PIDF if battery voltage has changed meaningfully
        double voltage = battery.getVoltage();
        if (Math.abs(voltage - lastVoltage) > VOLTAGE_UPDATE_THRESHOLD) {
            applyPIDF();
            lastVoltage = voltage;
        }

        // Read velocity
        double velocityTicks = shooter.getVelocity();
        currentRPM = (((velocityTicks * 60.0) / TICKS_PER_REV) * (1.105) * 0.923076) ;
        rpmError = targetRPM - currentRPM;

        // Spin-up profiling
        if (targetRPM > 0 && !spinningUp) {
            spinningUp = true;
        }

        if (spinningUp && Math.abs(rpmError) < READY_NORMAL) {
            spinupTimeSec = spinupTimer.seconds();
            spinupTimer.reset();
            spinningUp = false;
        }

        // Ready-to-fire thresholds
        readyLoose  = Math.abs(rpmError) < READY_LOOSE;
        readyNormal = Math.abs(rpmError) < READY_NORMAL;
        readyTight  = Math.abs(rpmError) < READY_TIGHT;
        readySniper = Math.abs(rpmError) < READY_SNIPER;

        // Convert RPM → ticks/sec
        double targetTicksPerSec =
                (targetRPM * TICKS_PER_REV) / 60.0;

        shooter.setVelocity(targetTicksPerSec * 1.04651162790697674418604651162791);
    }

    // ======================================================
    // APPLY PIDF (WITH VOLTAGE COMPENSATION)
    // ======================================================
    private void applyPIDF() {

        double voltage = battery.getVoltage();
        if (voltage <= 0) voltage = NOMINAL_VOLTAGE;

        double compensatedF =
                BASE_F * (NOMINAL_VOLTAGE / voltage);

        // Safety clamp
        compensatedF = clamp(
                compensatedF,
                BASE_F * 0.85,
                BASE_F * 1.15
        );

        kF = compensatedF;

        shooter.setVelocityPIDFCoefficients(
                kP, kI, kD, kF
        );
    }

    // ======================================================
    // DISTANCE → RPM MAP (OPTIONAL)
    // ======================================================
    private double rpmFromDistance(double meters) {
        return clamp(3000 + (meters * 600), 3200, 5450);
    }

    // ======================================================
    // TELEMETRY
    // ======================================================
    public void updateTelemetry() {
        t.mode = mode;
        t.targetRPM = targetRPM;
        t.currentRPM = currentRPM;
        t.errorRPM = rpmError;

        t.readyLoose = readyLoose;
        t.readyNormal = readyNormal;
        t.readyTight = readyTight;
        t.readySniper = readySniper;

        t.spinupTimeSec = spinupTimeSec;

        t.kP = kP;
        t.kI = kI;
        t.kD = kD;
        t.kF = kF;

        t.motorPower = shooter.getPower();
        t.batteryVoltage = battery.getVoltage();

        t.pose = shooter.getCurrentPosition();
    }

    public ShooterTelemetry getTelemetry() {
        return t;
    }

    // ======================================================
    // UTIL
    // ======================================================
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void setPower(double v) {
        shooter.setPower(v);
    }

    public double getVel() {
        return shooter.getVelocity();
    }

    public double getSmth() {
        return shooter.getCurrentPosition();
    }

    public void stop() {
        shooter.setPower(0);
        mode = ShooterMode.STOP;
    }
}
