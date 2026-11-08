package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveClass {
    // Telemetry output variables
    private double t_rawXInput, t_rawYInput, t_rawTurnInput;
    private double t_shapedXInput, t_shapedYInput, t_shapedTurnInput;
    private double t_finalXDirection, t_finalYDirection;
    private double t_lf, t_rf, t_lb, t_rb;
    private Pose2D t_pose;


    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    private static final double fullSpeed = 1.0;
    private static final double slowSpeed = 0.45;
    private double currentSpeed = fullSpeed;

    private static final double transDeadband = 0.06;
    private static final double rotDeadband = 0.08;
    private static final double stickExpo = 3.0;

    private static final double accelRate = 0.06;
    private static final double absStepCap = 0.40;
    private double lastLF = 0.0, lastRF = 0.0, lastLB = 0.0, lastRB = 0.0;

    // turn override threshold
    private static final double turnOverrideThreshold = 0.24; // changed from 0.25

    // debounce helper for Y toggle
    private boolean yPreviouslyPressed = false;
    private final GoBildaPinpointDriver odo;

    public DriveClass(HardwareMap hardwareMap) {
        // hardware map
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        odo.setOffsets(184.0, -164.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // We need to add setposition(); to track robot coordinates according to FTC guidelines, with 0,0 being the center of the FTC field. Appropriate offsets and heading are included in the Pinpoint User Guide PDF.

        setBrakeMode(true);
        odo.resetPosAndIMU();
        t_pose = odo.getPosition();
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

    private double shape(double v) {
        if (stickExpo == 1.0) return v;
        double s = Math.signum(v);
        return s * Math.pow(Math.abs(v), stickExpo);
    }

    private double applyRampUpAndCap(double last, double target) {
        if (Math.abs(target) < Math.abs(last)) {
            // instant down (aggressive stop)
            return target;
        }
        double delta = target - last;
        double step = Math.signum(delta) * Math.min(Math.abs(delta), accelRate);
        if (Math.abs(step) > absStepCap) step = Math.signum(step) * absStepCap;
        return last + step;
    }

    private void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior mode = enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontDrive.setZeroPowerBehavior(mode);
        rightFrontDrive.setZeroPowerBehavior(mode);
        leftBackDrive.setZeroPowerBehavior(mode);
        rightBackDrive.setZeroPowerBehavior(mode);
    }

    public void frontRightForward() {
        rightFrontDrive.setPower(-1);
    }

    public void backRightForward() {
        rightBackDrive.setPower(-1);
    }

    public void backLeftForward() {
        leftBackDrive.setPower(-1);
    }

    public void frontLeftForward() {
        leftFrontDrive.setPower(-1);
    }

    public void frontRightBackward() {
        rightFrontDrive.setPower(1);
    }

    public void backRightBackward() {
        rightBackDrive.setPower(1);
    }

    public void backLeftBackward() {
        leftBackDrive.setPower(1);
    }

    public void frontLeftBackward() {
        leftFrontDrive.setPower(1);
    }

    public void resetDebug() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void updateMotors(Gamepad gamepad, boolean reverse) {
        double directional = 1;
        if (reverse) {
            directional = -1;
        }

        if (gamepad.b) {
            odo.resetPosAndIMU();
        }

        // read the angle of the robot
        odo.update();
        Pose2D pose = odo.getPosition();
        double headingRad = pose.getHeading(AngleUnit.RADIANS); //uses orientation to pull the yaw

        // read raw inputs
        double rawXInput = gamepad.left_stick_x * directional;    // strafe
        double rawYInput = -gamepad.left_stick_y * directional;  // forward positive
        double rawTurnInput = 0.45 * (-gamepad.right_stick_x * directional); // rotation (driver)

        // deadbands to remove stick drift
        double dbX = applyDeadband(rawXInput, transDeadband);
        double dbY = applyDeadband(rawYInput, transDeadband);
        double dbTurn = applyDeadband(rawTurnInput, rotDeadband);

        // shaping
        double shapedXInput = shape(dbX);
        double shapedYInput = shape(dbY);
        double shapedTurnInput = shape(dbTurn);

        // field->robot transform (robot-relative commands)
        // rotate the field vector by -heading to produce robot-relative motion
        double cosH = Math.cos(-headingRad);
        double sinH = Math.sin(-headingRad);
        double finalXDirection = shapedXInput * cosH - shapedYInput * sinH;
        double finalYDirection = shapedXInput * sinH + shapedYInput * cosH;

        // TURN OVERRIDE: if driver asks for a strong turn, spin in place and skip the mixing math
        double lf, rf, lb, rb;
        if (Math.abs(rawTurnInput) > turnOverrideThreshold) {
            // double turnPower = rawTurnInput; // allow raw (no shaping) or use shapedTurnInput if you prefer
            lf = -rawTurnInput;
            rf = rawTurnInput;
            lb = -rawTurnInput;
            rb = rawTurnInput;
        } else {
            // standard mecanum mix (robot-frame)
            lf = finalYDirection + finalXDirection + shapedTurnInput;
            rf = finalYDirection - finalXDirection - shapedTurnInput;
            lb = finalYDirection - finalXDirection + shapedTurnInput;
            rb = finalYDirection + finalXDirection - shapedTurnInput;

            // normalize if necessary
            double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                    Math.max(Math.abs(lb), Math.abs(rb)))));
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;

            // scale per currentSpeed
            lf *= currentSpeed;
            rf *= currentSpeed;
            lb *= currentSpeed;
            rb *= currentSpeed;

            // ramp-up-only smoothing (instant down)
            lf = applyRampUpAndCap(lastLF, lf);
            rf = applyRampUpAndCap(lastRF, rf);
            lb = applyRampUpAndCap(lastLB, lb);
            rb = applyRampUpAndCap(lastRB, rb);
        }

        // small output deadband
        double outputDeadband = 0.02;
        if (Math.abs(lf) < outputDeadband) lf = 0.0;
        if (Math.abs(rf) < outputDeadband) rf = 0.0;
        if (Math.abs(lb) < outputDeadband) lb = 0.0;
        if (Math.abs(rb) < outputDeadband) rb = 0.0;

        // setting motor powers
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);

        // update last applied powers
        lastLF = lf;
        lastRF = rf;
        lastLB = lb;
        lastRB = rb;

        // set brake mode when no input requested
        boolean anyInput = (Math.abs(dbX) > 0.0001) || (Math.abs(dbY) > 0.0001) || (Math.abs(dbTurn) > 0.0001);
        setBrakeMode(!anyInput);

        /// telemetry output
        t_rawXInput = rawXInput;
        t_rawYInput = rawYInput;
        t_rawTurnInput = rawTurnInput;

        t_shapedXInput += shapedXInput;
        t_shapedYInput += shapedYInput;
        t_shapedTurnInput = shapedTurnInput;

        t_finalXDirection = finalXDirection;
        t_finalYDirection = finalYDirection;

        t_lf = lf;
        t_rf = rf;
        t_lb = lb;
        t_rb = rb;

        t_pose = pose;
    }

    public void setAutoPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
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
        public Pose2D pose;
        public String mode;
        public double currentSpeed;
    }

    public DriveTelemetry getDriveTelemetry() {
        DriveTelemetry d = new DriveTelemetry();
        d.rawX = t_rawXInput;
        d.rawY = t_rawYInput;
        d.rawTurn = t_rawTurnInput;

        d.tx_field = t_shapedXInput;
        d.ty_field = t_shapedYInput;
        d.rotationCmd = t_shapedTurnInput;

        d.robotX = t_finalXDirection;
        d.robotY = t_finalYDirection;

        d.lf = t_lf;
        d.rf = t_rf;
        d.lb = t_lb;
        d.rb = t_rb;

        d.pose = t_pose;

        d.currentSpeed = currentSpeed;

        return d;
    }
}
