package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "DualIntakeTest", group = "Linear OpMode")
public class DualIntakeTest extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor frontIntake = null;
    private DcMotor rearIntake = null;

    // --- LOGGER VARIABLES ---
    // runtime measures the total time since PLAY was pressed. It is recorded in
    // each CSV row so measurements can be compared against when they happened.
    private final ElapsedTime runtime = new ElapsedTime();

    // logTimer measures the time since the previous CSV row was written.
    // It is reset after each row so the logger can sample at a regular interval.
    private final ElapsedTime logTimer = new ElapsedTime();

    // 0.05 seconds per row targets 1 / 0.05 = 20 rows per second (20 Hz).
    // Change this value to change the target sample rate:
    //   0.10 = 10 rows/s, 0.05 = 20 rows/s, 0.02 = 50 rows/s.
    // The actual rate can be lower if the robot loop or file writing runs slowly.
    private final double logInterval = 0.05;

    private double currentSpeed = 1.0;

    // PrintWriter is the object used to write text lines to the CSV file.
    // It stays null if the file could not be opened; the OpMode can still run.
    private PrintWriter logWriter = null;

    // Save the exact path so it can be displayed on the Driver Station at startup
    // and again when the file is closed.
    private String logFilePath = null;

    @Override
    public void runOpMode() {
        // --- HARDWARE ---
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        frontIntake = hardwareMap.get(DcMotor.class, "FrontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "RearIntake");

        // --- MOTOR DIRECTIONS & BRAKING ---
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- CSV LOGGER: CREATE THE FILE AND WRITE ITS HEADER ---
        try {
            // /sdcard/FIRST is a common storage folder on the FTC Robot Controller.
            // mkdirs() creates the folder if it is missing; if it already exists,
            // no new folder is needed.
            File logFolder = new File("/sdcard/FIRST");
            if (!logFolder.exists()) logFolder.mkdirs();

            // Add the current date/time to the filename so each OpMode run gets
            // its own file instead of overwriting a previous test.
            // Pattern guide: yyyy = year, MM = month, dd = day, HH = 24-hour hour,
            // mm = minute, ss = second. Example: 20261022_174500.
            // Locale.US keeps formatting predictable regardless of device locale.
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            logFilePath = "/sdcard/FIRST/dual_intake_" + timestamp + ".csv";

            // FileWriter opens the file; PrintWriter provides convenient methods
            // such as println(), printf(), flush(), and close().
            logWriter = new PrintWriter(new FileWriter(logFilePath));

            // The FIRST line is the CSV header: one column name for every value
            // that will be written in each data row below. Keep these names in the
            // same order as the values in the printf() call in the logging loop.
            // To log another measurement, add its column here AND add its value
            // in the matching position in that printf() call.
            logWriter.println("time_seconds,forward_command,strafe_command,turn_command,left_front_power,right_front_power,left_back_power,right_back_power,front_intake_power,rear_intake_power,left_front_encoder,right_front_encoder,left_back_encoder,right_back_encoder,button_a,button_b,button_x,button_y");

            // flush() pushes buffered text toward the file immediately. Flushing
            // the header makes the column names available even before the first
            // data row is recorded.
            logWriter.flush();

            telemetry.addData("LOGGER", "Ready");
            telemetry.addData("File", logFilePath);
        } catch (IOException error) {
            // File creation can fail because of storage/path problems. Leave the
            // writer null so the loop safely skips CSV writes while the robot
            // itself can continue to operate. The error is shown on telemetry.
            logWriter = null;
            telemetry.addData("LOGGER ERROR", error.getMessage());
        }

        telemetry.addLine("Robot initialized.");
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();

        // Start both clocks at the beginning of the actual test, not during
        // hardware initialization or while waiting for the driver to press PLAY.
        runtime.reset();
        logTimer.reset();

        while (opModeIsActive()) {
            // --- DRIVE INPUTS ---
            double forward = 0.0;
            double strafe = 0.0;
            double turn = 0.0;

            if (gamepad1.dpad_up) {
                forward = 0.5;
            } else if (gamepad1.dpad_down) {
                forward = -0.5;
            } else if (gamepad1.dpad_left) {
                strafe = -0.5;
            } else if (gamepad1.dpad_right) {
                strafe = 0.5;
            } else {
                forward = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                turn = -gamepad1.right_stick_x * 0.5;
            }

            // --- DRIVE MOTOR POWERS ---
            double leftFrontPower = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower = forward - strafe + turn;
            double rightBackPower = forward + strafe - turn;

            double largestPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
            if (largestPower > 1.0) {
                leftFrontPower /= largestPower;
                rightFrontPower /= largestPower;
                leftBackPower /= largestPower;
                rightBackPower /= largestPower;
            }

            leftFrontPower *= currentSpeed;
            rightFrontPower *= currentSpeed;
            leftBackPower *= currentSpeed;
            rightBackPower *= currentSpeed;

            // --- DUAL INTAKE ---
            double frontIntakePower = 0.0;
            double rearIntakePower = 0.0;

            if (gamepad1.right_trigger > 0.75) {
                frontIntakePower = 0.75;
                rearIntakePower = 0.75;
            } else if (gamepad1.left_trigger > 0.75) {
                frontIntakePower = -0.75;
                rearIntakePower = -0.75;
            }

            frontIntake.setPower(frontIntakePower);
            rearIntake.setPower(rearIntakePower);

            // --- INDIVIDUAL DRIVE MOTOR TESTS ---
            if (gamepad1.a) {
                leftFrontDrive.setPower(0.0);
                rightFrontDrive.setPower(0.0);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(0.0);
                telemetry.addLine("Testing LEFT BACK motor");
            } else if (gamepad1.b) {
                leftFrontDrive.setPower(0.0);
                rightFrontDrive.setPower(0.0);
                leftBackDrive.setPower(0.0);
                rightBackDrive.setPower(0.5);
                telemetry.addLine("Testing RIGHT BACK motor");
            } else if (gamepad1.x) {
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(0.0);
                leftBackDrive.setPower(0.0);
                rightBackDrive.setPower(0.0);
                telemetry.addLine("Testing LEFT FRONT motor");
            } else if (gamepad1.y) {
                leftFrontDrive.setPower(0.0);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.0);
                rightBackDrive.setPower(0.0);
                telemetry.addLine("Testing RIGHT FRONT motor");
            } else {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            // --- CSV LOGGING (TARGET: 20 ROWS PER SECOND) ---
            // Only write when the file was opened successfully AND the target
            // interval has elapsed. Checking logWriter != null prevents a file
            // creation failure from causing a null-pointer error in the loop.
            if (logWriter != null && logTimer.seconds() >= logInterval) {

                // printf() writes one comma-separated row. The format string and
                // the arguments below MUST stay in the same order as the header.
                // Format specifier quick guide:
                //   %.3f = floating-point number with 3 digits after the decimal
                //   %d   = whole number (used here for encoder tick counts)
                //   %b   = boolean value (true/false)
                //   %n   = platform-correct newline ending the CSV row
                // There are 18 columns in this format string, so it must receive
                // exactly 18 matching values after the format string.
                // Locale.US makes decimal values use a period, not a locale-specific
                // comma; this matters because commas separate CSV columns.
                logWriter.printf(Locale.US, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%b,%b,%b,%b%n",
                        runtime.seconds(), forward, strafe, turn,
                        leftFrontDrive.getPower(), rightFrontDrive.getPower(), leftBackDrive.getPower(), rightBackDrive.getPower(),
                        frontIntake.getPower(), rearIntake.getPower(),
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                        gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y);

                // flush() writes buffered data regularly instead of waiting until
                // close(). This helps preserve recent rows if the Controller stops
                // unexpectedly, although frequent flushing can add some I/O work.
                logWriter.flush();

                // Begin timing the next interval. This gives the logger a target
                // sample interval, although loop delays can reduce the real rate.
                logTimer.reset();
            }

            // --- TELEMETRY ---
            telemetry.addData("Runtime", "%.2f seconds", runtime.seconds());
            telemetry.addData("Movement", "Forward %.2f | Strafe %.2f | Turn %.2f", forward, strafe, turn);
            telemetry.addData("LF Power", "%.2f", leftFrontDrive.getPower());
            telemetry.addData("RF Power", "%.2f", rightFrontDrive.getPower());
            telemetry.addData("LB Power", "%.2f", leftBackDrive.getPower());
            telemetry.addData("RB Power", "%.2f", rightBackDrive.getPower());
            telemetry.addData("Front Intake", "%.2f", frontIntake.getPower());
            telemetry.addData("Rear Intake", "%.2f", rearIntake.getPower());
            telemetry.addData("LF Encoder", "%d", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RF Encoder", "%d", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LB Encoder", "%d", leftBackDrive.getCurrentPosition());
            telemetry.addData("RB Encoder", "%d", rightBackDrive.getCurrentPosition());
            // Show whether file creation succeeded. This checks whether a writer
            // exists; it is not a guarantee that every future disk write succeeds.
            telemetry.addData("LOGGER", logWriter != null ? "Recording" : "Not recording");
            telemetry.addLine("Motor test buttons: A = left back | B = right back | X = left front | Y = right front");
            telemetry.update();

            idle();
        }

        // --- STOP MOTORS ---
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        frontIntake.setPower(0.0);
        rearIntake.setPower(0.0);

        // --- SAVE AND CLOSE CSV FILE ---
        // Close the file only after the OpMode loop has ended. flush() asks the
        // writer to send any buffered row data to the file; close() then releases
        // the file handle. Always check for null because creating the logger may
        // have failed during initialization.
        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            logWriter = null;

            // Report the path so the test file can be located and copied from the
            // Robot Controller. The timestamp in the filename identifies this run.
            telemetry.addData("LOGGER", "File saved");
            telemetry.addData("File", logFilePath);
            telemetry.update();
        }
    }
}
