package org.firstinspires.ftc.teamcode.archived23_24SeaonCenterStage.demoBotDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Romeo strafe drive", group="Linear OpMode")
public class demoBot extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo Claw = null;
    private CRServo Shoulder = null;
    private CRServo Elbow = null;

    private double slowSpeed = 0.4;
    private double fullSpeed = 1;
    private double currentSpeed = 1;
    private boolean canChangeSpeeds = true;
    private boolean RightBumperPressed = false;
    private boolean LeftTriggerPressed = false;
    private boolean elbowPressed = false;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        Shoulder = hardwareMap.get(CRServo.class, "shoulder");
        Elbow = hardwareMap.get(CRServo.class, "elbow");
        Claw = hardwareMap.get(Servo.class, "claw");
        Shoulder.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_y;
            double turn = 0;
            if (Math.abs(gamepad1.right_stick_x) > 0.4) turn =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -forward - strafe + turn;
            double rightFrontPower = -forward + strafe - turn;
            double leftBackPower   = -forward + strafe + turn;
            double rightBackPower  = -forward - strafe - turn;

            /*if (!gamepad1.right_bumper) {
                RightTriggerPressed = false; // Reset the flag when the trigger is released
            }// Check the conditions for opening/closing the claw
            if (gamepad1.right_bumper && !RightTriggerPressed) {
                // If the right trigger is pressed, toggle the claw and reset the sensor flag
                Claw.setPosition(Claw.getPosition() == 0.7 ? 0.4 : 0.7);
                RightTriggerPressed = true; // Reset the flag when the trigger is released
            }*/

            //controlling shoulder when grabbing specimen of the wall position using a
            // toggle boolean and a ternary operator
            if (!gamepad1.right_bumper) {
                RightBumperPressed = false; // Reset the flag when the trigger is released
            }// Check the conditions for opening/closing the claw
            if (gamepad1.right_bumper && !RightBumperPressed) {
                // If the right trigger is pressed, toggle the claw and reset the sensor flag
                Claw.setPosition(Claw.getPosition() == 0.8 ? 0.2 : 0.8);
                RightBumperPressed = true; // Reset the flag when the trigger is released
            }

            if (gamepad1.right_trigger > 0.15) {
                Shoulder.setPower(0.35);
            } else if (gamepad1.left_trigger > 0.15) {
                Shoulder.setPower(-0.35);
            } else {
                Shoulder.setPower(0.004);
                // Check the conditions for opening/closing the claw
            }

            if (gamepad1.y) {
                Elbow.setPower(0.25);
            } else if (gamepad1.x) {
                Elbow.setPower(-0.25);
            } else {
                Elbow.setPower(0.008);
                // Check the conditions for opening/closing the claw
            }

            if (gamepad1.a) {
                Shoulder.setPower(0.51);
            } else if (gamepad1.b) {
                Shoulder.setPower(-1);
            } else {
                //Shoulder.setPower(0.0001);
                // Check the conditions for opening/closing the claw
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * currentSpeed);
            rightFrontDrive.setPower(rightFrontPower * currentSpeed);
            leftBackDrive.setPower(leftBackPower * currentSpeed);
            rightBackDrive.setPower(rightBackPower * currentSpeed);

            telemetry.addData("serve", Claw.getPosition());
            telemetry.addData("right trigger", RightBumperPressed);
            telemetry.addData("shoulder", Claw.getPosition());
            telemetry.addData("left trigger", RightBumperPressed);
            telemetry.update();
        }
    }}