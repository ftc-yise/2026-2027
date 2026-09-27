package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;

@TeleOp(name="BaseTeleop", group="Linear Opmode")
public class BaseTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor intake = null;
    //private ColorSensor floor = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode() {
        DriveClass drive = new DriveClass(hardwareMap);

        //intake = hardwareMap.get(DcMotor.class, "intake");
        //floor = hardwareMap.get(ColorSensor.class, "floor");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // --- DRIVE & SPEED TOGGLE ---
            //drive.handleSpeedToggle(gamepad1);
            //drive.updateMotors(gamepad1, true);


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward   = -gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double strafe =  gamepad1.left_stick_y;
            double turn     =  -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forward + strafe + turn;
            double rightFrontPower = forward - strafe + turn;
            double leftBackPower   = forward - strafe - turn;
            double rightBackPower  = forward + strafe - turn;

            // put your teleop code here
           /* if (gamepad1.right_trigger > 0.75) {
                intake.setPower(.6);

            } else if (gamepad1.left_trigger > .75) {
                intake.setPower(-.6);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-.6);
            } else {
                intake.setPower(0);
            }*/

            if (gamepad1.a){
                leftFrontDrive.setPower(1);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                telemetry.addLine("=== left front drive ===");
            } else if (gamepad1.y){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(1);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                telemetry.addLine("=== left back drive ===");
            } else if (gamepad1.x){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(1);
                rightFrontDrive.setPower(0);
                telemetry.addLine("=== right back drive ===");
            } else if (gamepad1.b){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                rightFrontDrive.setPower(1);
                telemetry.addLine("=== right front drive ===");
            } else{
                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
                rightFrontDrive.setPower(rightFrontPower);
                telemetry.addLine("=== nuthin ===");
            }
            /*telemetry.addData("red", floor.red());
            telemetry.addData("blue", floor.blue());
            telemetry.addData("green", floor.green());
            */
            telemetry.update();


        }
    }
}
