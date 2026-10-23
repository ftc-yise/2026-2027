package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Drive", group="Linear OpMode")
public class Test_and_DemoOpmode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double currentSpeed = 1;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

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
            double forward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_x;
            double turn = 0;
            if (Math.abs(gamepad1.right_stick_x) > 0.4) turn =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -forward - strafe + turn;
            double rightFrontPower = -forward + strafe - turn;
            double leftBackPower   = -forward + strafe + turn;
            double rightBackPower  = -forward - strafe - turn;

            if(gamepad1.a){
                leftBackDrive.setPower(0.5);
                telemetry.addLine("left back - current");
                telemetry.addLine("left back - a");
                telemetry.addLine("right back - B");
                telemetry.addLine("left front - x");
                telemetry.addLine("Right front - Y");

            } else if (gamepad1.b) {
                telemetry.addLine("right back - current");
                telemetry.addLine("left back - a");
                telemetry.addLine("right back - B");
                telemetry.addLine("left front - x");
                telemetry.addLine("Right front - Y");
                rightBackDrive.setPower(0.5);
            }else if (gamepad1.x) {
                telemetry.addLine("left front - current");
                telemetry.addLine("left back - a");
                telemetry.addLine("right back - B");
                telemetry.addLine("left front - x");
                telemetry.addLine("Right front - Y");
                leftFrontDrive.setPower(0.5);
            }else if (gamepad1.y) {
                telemetry.addLine("right front - current");
                telemetry.addLine("left back - a");
                telemetry.addLine("right back - B");
                telemetry.addLine("left front - x");
                telemetry.addLine("Right front - Y");
                rightFrontDrive.setPower(0.5);
            } else {
                telemetry.addLine("left back - a");
                telemetry.addLine("right back - B");
                telemetry.addLine("left front - x");
                telemetry.addLine("Right front - Y");
                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower * currentSpeed);
                rightFrontDrive.setPower(rightFrontPower * currentSpeed);
                leftBackDrive.setPower(leftBackPower * currentSpeed);
                rightBackDrive.setPower(rightBackPower * currentSpeed);
            }
            telemetry.update();
        }
    }}