package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveClass {
    // Telemetry output variables
    private double t_rawX, t_rawY, t_rawTurn;
    private double t_txField, t_tyField, t_rotationCmd;
    private double t_robotX, t_robotY;
    private double t_lf, t_rf, t_lb, t_rb;
    private double t_headingDeg;
    private SparkFunOTOS.Pose2D t_pose;



    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double fullSpeed = 1.0;
    private double slowSpeed = 0.45;
    private double currentSpeed = fullSpeed;

    private double transDeadband = 0.06;
    private double rotDeadband = 0.08;
    private double stickExpo = 3.0;

    private double accelRate = 0.06;
    private double absStepCap = 0.40;
    private double lastLF = 0.0, lastRF = 0.0, lastLB = 0.0, lastRB = 0.0;

    // turn override threshold
    private double turnOverrideThreshold = 0.25;

    // debounce helper for Y toggle
    private boolean yPreviouslyPressed = false;
    private boolean xPreviouslyPressed = false;
    private boolean runmodeFieldorientation = false;

    private SparkFunOTOS myOtos = null;

    public DriveClass(HardwareMap hardwareMap) {
        // hardware map
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        setBrakeMode(true);

        myOtos.resetTracking();
        SparkFunOTOS.Pose2D pose = myOtos.getPosition();
    }

    public void handleSpeedToggle(Gamepad gamepad) {
        if (gamepad.y && !yPreviouslyPressed) {
            currentSpeed = (currentSpeed == fullSpeed) ? slowSpeed : fullSpeed;
            yPreviouslyPressed = true;
        } else if (!gamepad.y) {
            yPreviouslyPressed = false;
        }
    }

    public void stopAllMotors() {
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }

    private double applyDeadband(double val, double threshold) {
        return Math.abs(val) > threshold ? val : 0.0;
    }

    private double shape(double v, double expo) {
        if (expo == 1.0) return v;
        double s = Math.signum(v);
        return s * Math.pow(Math.abs(v), expo);
    }

    private double applyRampUpAndCap(double last, double target, double accel, double absCap) {
        if (Math.abs(target) < Math.abs(last)) {
            // instant down (aggressive stop)
            return target;
        }
        double delta = target - last;
        double step = Math.signum(delta) * Math.min(Math.abs(delta), accel);
        if (Math.abs(step) > absCap) step = Math.signum(step) * absCap;
        return last + step;
    }

    private void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior mode = enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontDrive.setZeroPowerBehavior(mode);
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftBackDrive.setZeroPowerBehavior(mode);
        rightBackDrive.setZeroPowerBehavior(mode);
    }

    public void updateMotors(Gamepad gamepad, boolean reverse) {
        double directional = 1;

        if (reverse == true){
            directional = -1;
        } else {
            directional = 1;
        }

        //read the angle of the robot
        SparkFunOTOS.Pose2D pose = myOtos.getPosition();
        double headingDeg = 0; //pose.h; //OTOS provides degrees (assumed)
        double headingRad = Math.toRadians(headingDeg);

        // read raw inputs
        double rawX = gamepad.left_stick_x * directional;    // strafe
        double rawY = -gamepad.left_stick_y * directional;  // forward positive
        double rawTurn = -gamepad.right_stick_x * directional; // rotation (driver)

        // deadbands to remove stick drift
        double dbX = applyDeadband(rawX, transDeadband);
        double dbY = applyDeadband(rawY, transDeadband);
        double dbTurn = applyDeadband(rawTurn, rotDeadband);

        // shaping
        double shapedX = shape(dbX, stickExpo);
        double shapedY = shape(dbY, stickExpo);
        double shapedTurn = shape(dbTurn, stickExpo);

        // D-Pad override (perfect unit vectors) - ignore sticks entirely when any dpad pressed
        boolean dpadActive = false;
        double dpadX = 0.0;
        double dpadY = 0.0;
        if (gamepad.dpad_up)    { dpadY = 1.0; dpadActive = true; }
        if (gamepad.dpad_down)  { dpadY = -1.0; dpadActive = true; }
        if (gamepad.dpad_right) { dpadX = 1.0; dpadActive = true; }
        if (gamepad.dpad_left)  { dpadX = -1.0; dpadActive = true; }

        double tx_field = dpadActive ? dpadX : shapedX; // field-frame translation x (strafe)
        double ty_field = dpadActive ? dpadY : shapedY; // field-frame translation y (forward)

        // rotation command is driver only (no assist here)
        double rotationCmd = shapedTurn;
        if (dpadActive) rotationCmd = 0.0; // disable turning while D-pad linear mode is active

        // field->robot transform (robot-relative commands)
        // rotate the field vector by -heading to produce robot-relative motion
        double cosH = Math.cos(-headingRad);
        double sinH = Math.sin(-headingRad);
        double robotX = tx_field * cosH - ty_field * sinH;
        double robotY = tx_field * sinH + ty_field * cosH;

        // TURN OVERRIDE: if driver asks for a strong turn, spin in place and skip the mixing math
        double lf, rf, lb, rb;
        if (Math.abs(rawTurn) > turnOverrideThreshold) {
            double turnPower = rawTurn; // allow raw (no shaping) or use shapedTurn if you prefer
            lf = -turnPower;
            rf = turnPower;
            lb = -turnPower;
            rb = turnPower;
        } else {
            // standard mecanum mix (robot-frame)
            lf = robotY + robotX + rotationCmd;
            rf = robotY - robotX - rotationCmd;
            lb = robotY - robotX + rotationCmd;
            rb = robotY + robotX - rotationCmd;

            // normalize if necessary
            double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                    Math.max(Math.abs(lb), Math.abs(rb)))));
            lf /= max; rf /= max; lb /= max; rb /= max;

            // scale per currentSpeed
            lf *= currentSpeed; rf *= currentSpeed; lb *= currentSpeed; rb *= currentSpeed;

            // ramp-up-only smoothing (instant down)
            lf = applyRampUpAndCap(lastLF, lf, accelRate, absStepCap);
            rf = applyRampUpAndCap(lastRF, rf, accelRate, absStepCap);
            lb = applyRampUpAndCap(lastLB, lb, accelRate, absStepCap);
            rb = applyRampUpAndCap(lastRB, rb, accelRate, absStepCap);
        }

        // small output deadband
        double outputDeadband = 0.02;
        if (Math.abs(lf) < outputDeadband) lf = 0.0;
        if (Math.abs(rf) < outputDeadband) rf = 0.0;
        if (Math.abs(lb) < outputDeadband) lb = 0.0;
        if (Math.abs(rb) < outputDeadband) rb = 0.0;

        // set motor powers
        if (gamepad.x && !xPreviouslyPressed) {
            runmodeFieldorientation = !runmodeFieldorientation;
            xPreviouslyPressed = true;
        } else if (!gamepad.x) {
            xPreviouslyPressed = false;
        }

        if (runmodeFieldorientation) {
            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);
        } else {
            double leftFrontPower  = rawY + rawX - rawTurn;
            double rightFrontPower = rawY - rawX + rawTurn;
            double leftBackPower   = rawY - rawX - rawTurn;
            double rightBackPower  = rawY + rawX + rawTurn;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);
        }
        // update last applied powers
        lastLF = lf; lastRF = rf; lastLB = lb; lastRB = rb;

        // set brake mode when no input requested
        boolean anyInput = (Math.abs(dbX) > 0.0001) || (Math.abs(dbY) > 0.0001) || (Math.abs(dbTurn) > 0.0001) || dpadActive;
        setBrakeMode(!anyInput);

        /// telemetry output
        t_rawX = rawX;
        t_rawY = rawY;
        t_rawTurn = rawTurn;

        t_txField = tx_field;
        t_tyField = ty_field;
        t_rotationCmd = rotationCmd;

        t_robotX = robotX;
        t_robotY = robotY;

        t_lf = lf;
        t_rf = rf;
        t_lb = lb;
        t_rb = rb;

        t_headingDeg = headingDeg;
        t_pose = pose;

    }

    public void setAutoPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    //telemetry
    public static class DriveTelemetry {
        public double rawX, rawY, rawTurn;
        public double tx_field, ty_field, rotationCmd;
        public double robotX, robotY;
        public double lf, rf, lb, rb;
        public double headingDeg;
        public SparkFunOTOS.Pose2D pose;

        public double currentSpeed;
    }

    public DriveTelemetry getDriveTelemetry() {
        DriveTelemetry d = new DriveTelemetry();
        d.rawX = t_rawX;
        d.rawY = t_rawY;
        d.rawTurn = t_rawTurn;

        d.tx_field = t_txField;
        d.ty_field = t_tyField;
        d.rotationCmd = t_rotationCmd;

        d.robotX = t_robotX;
        d.robotY = t_robotY;

        d.lf = t_lf;
        d.rf = t_rf;
        d.lb = t_lb;
        d.rb = t_rb;

        d.headingDeg = t_headingDeg;
        d.pose = t_pose;

        d.currentSpeed = currentSpeed;

        return d;
    }
}



