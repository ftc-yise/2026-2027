package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="chasis tester", group="Ball Bot")
public class MoterTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        // --- Hardware map ---

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.75 || gamepad1.left_trigger > 0.75) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.a) {
                leftFrontDrive.setPower(1);
            } else if (gamepad1.b) {
                leftBackDrive.setPower(1);
            } else if (gamepad1.x) {
                rightFrontDrive.setPower(1);
            } else if (gamepad1.y){
                rightBackDrive.setPower(1);
            } else {
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left Front (%.2f), left Back (%.2f), right Front (%.2f), right Back (%.2f)", leftFrontDrive.getPower(), leftBackDrive.getPower(), rightFrontDrive.getPower(), rightBackDrive.getPower());
            telemetry.addLine("A - Left Front B - Left Back X - Right Front Y - Right Back");
            telemetry.update();
        }
    }
}
