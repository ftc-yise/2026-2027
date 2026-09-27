package org.firstinspires.ftc.teamcode.archived.yiseArchivedClass;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Romeo strafe drive", group="Linear OpMode")
public class RomeoStrafeDrive extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    // Manipulator
    private CRServo one = null;
    private CRServo two = null;
    private CRServo three = null;
    private CRServo four = null;


    // Timing / telemetry
    private ElapsedTime runtime = new ElapsedTime();

    // Tunable constants
    private static final double SLOW_SPEED = 0.45;
    private static final double FULL_SPEED = 1.0;
    private static final double DEADBAND = 0.05;

    // Shoulder / Elbow tuning
    // Hold bias tries to counter gravity when no input is present. TUNE ON ROBOT.
    private static final double SHOULDER_HOLD_BIAS = 0.02; // small holding power (positive or negative depending on mechanism)
    private static final double ELBOW_HOLD_BIAS = 0.015;

    private static final double SHOULDER_PRESET_UP = 1;    // A button quick power up (dangerous - tune)
    private static final double SHOULDER_PRESET_DOWN = 0; // B button quick power down
    private static final double ELBOW_PRESET_UP = 0.15;
    private static final double ELBOW_PRESET_DOWN = -0.15;

    // Claw positions (tweak)
    private static final double CLAW_OPEN = 0.8;
    private static final double CLAW_CLOSED = 0.2;

    // State variables
    private double currentSpeed = FULL_SPEED;
    private boolean prevLeftBumper = false;   // used for speed toggle (edge detect)
    private boolean prevRightBumper = false;  // used for claw toggle (edge detect)
    private boolean clawClosed = false;       // current claw state

    @Override
    public void runOpMode() {

        // hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RightBackDrive");

        two = hardwareMap.get(CRServo.class, "shoulder");
        three = hardwareMap.get(CRServo.class, "elbow");
        one = hardwareMap.get(CRServo.class, "claw");
        four = hardwareMap.get(CRServo.class, "ramp");

        // Directions - adjust if a motor/servo is reversed on your robot
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        // Safety: use BRAKE to hold position for drive when joystick released
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize claw closed or open depending on preference
        clawClosed = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ----- DRIVE INPUTS -----
            // Standard mapping: left_stick_y = forward/back (push forward -> negative), left_stick_x = strafe,
            // right_stick_x = rotation
            double rawForward = gamepad1.left_stick_y; // forward positive
            double rawStrafe  =  gamepad1.left_stick_x; // right positive
            double rawTurn    =  gamepad1.right_stick_x; // cw positive

            // apply deadband
            double forward = applyDeadband(rawForward, DEADBAND);
            double strafe  = applyDeadband(rawStrafe, DEADBAND);
            double turn    = applyDeadband(rawTurn, DEADBAND);

            // compute raw wheel powers (mecanum)
            double lf = forward - strafe + turn;
            double rf = -forward - strafe - turn;
            double lb = -forward + strafe + turn;
            double rb = forward + strafe - turn;

            // normalize so max abs is 1
            double max = Math.max(
                    Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lb), Math.abs(rb))
            );
            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }

            // speed toggle (left bumper edge -> toggle slow/full)
            if (gamepad1.left_bumper && !prevLeftBumper) {
                // toggle
                currentSpeed = (Math.abs(currentSpeed - FULL_SPEED) < 1e-6) ? SLOW_SPEED : FULL_SPEED;
            }
            prevLeftBumper = gamepad1.left_bumper;

            // apply currentSpeed
            leftFrontDrive.setPower(lf * currentSpeed);
            rightFrontDrive.setPower(rf * currentSpeed);
            leftBackDrive.setPower(lb * currentSpeed);
            rightBackDrive.setPower(rb * currentSpeed);

            // ----- CLAW TOGGLE (right bumper edge) -----
            if (gamepad1.right_trigger > 0.75){
                one.setPower(1);
                two.setPower(1);
                three.setPower(1);
            } else if (gamepad1.left_trigger > 0.75){
                one.setPower(-1);
                two.setPower(-1);
                three.setPower(-1);
            } else {
                one.setPower(0);
                two.setPower(0);
                three.setPower(0);
            }

            if (gamepad1.right_bumper){
                four.setPower(-1);

            } else if (gamepad1.left_bumper){
                four.setPower(0);

            } else {
                four.setPower(1);
            }

            // ----- TELEMETRY -----
            telemetry.addData("Mode", "TeleOp");
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.addLine("=== Drive ===");
            telemetry.addData("Speed Mode", (currentSpeed == FULL_SPEED) ? "FULL" : "SLOW");
            telemetry.addData("LF/RF/LB/RB", "%.2f, %.2f, %.2f, %.2f", lf, rf, lb, rb);
            telemetry.addLine("=== Manip ===");
            telemetry.addData("ClawClosed", clawClosed);
            telemetry.update();
        } // end while opModeIsActive
    } // end runOpMode

    // helper: applies deadband
    private double applyDeadband(double v, double deadband) {
        if (Math.abs(v) < deadband) return 0.0;
        // keep linear outside deadband
        return v;
    }
}