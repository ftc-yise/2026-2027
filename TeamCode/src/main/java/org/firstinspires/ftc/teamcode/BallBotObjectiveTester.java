package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Ball Bot Straightness Test", group = "Diagnostics")
public class BallBotObjectiveTester extends LinearOpMode {

    // =========================================================
    // CONFIGURATION
    // =========================================================

    // Hardware Configuration name for the SparkFun OTOS.
    // Change this string if your Robot Configuration uses a different name.
    private static final String OTOS_HARDWARE_NAME = "otos";

    // Square dimensions.
    private static final double SIDE_LENGTH = 24.0; // inches
    private static final int SIDE_COUNT = 4;
    private static final double TURN_ANGLE = 90.0; // degrees, counterclockwise

    // OTOS mounting offset relative to the center of the robot.
    // X: positive is right of robot center; negative is left.
    // Y: positive is forward of robot center; negative is behind.
    // H: sensor rotation relative to robot; clockwise is negative.
    // Leave all at zero if the OTOS is centered and aligned with the robot.
    private static final double OTOS_OFFSET_X = 0.0;
    private static final double OTOS_OFFSET_Y = 0.0;
    private static final double OTOS_OFFSET_H = 0.0;

    // Motor velocity targets are fractions of each motor's configured
    // maximum ticks per second, not raw motor power values.
    private static final double MAX_DRIVE = 0.32;
    private static final double MIN_DRIVE = 0.16;
    private static final double MAX_TURN = 0.20;
    private static final double MIN_TURN = 0.055;

    // Begin reducing the straight-line command this far from the endpoint.
    private static final double BRAKE_DISTANCE = 8.0; // inches
    private static final double DISTANCE_TOLERANCE = 0.45; // inches

    // Set both gains to zero for a raw, open-loop straightness baseline.
    // Leave enabled to test how well the chassis drives with OTOS correction.
    private static final double HEADING_KP = 0.012;
    private static final double MAX_HEADING_CORRECTION = 0.15;
    private static final double HEADING_DEADBAND = 0.5; // degrees

    private static final double CROSS_TRACK_KP = 0.025;
    private static final double MAX_STRAFE_CORRECTION = 0.12;

    private static final double TURN_KP = 0.009;
    private static final double TURN_TOLERANCE = 2.0; // degrees

    // Change in normalized velocity fraction per second.
    // Each motor is ramped independently.
    private static final double RAMP_UP_RATE = 0.85;
    private static final double RAMP_DOWN_RATE = 1.40;

    private static final double DRIVE_TIMEOUT = 10.0;
    private static final double TURN_TIMEOUT = 5.0;
    private static final double BRAKE_TIMEOUT = 2.5;

    // =========================================================
    // HARDWARE
    // =========================================================

    private DcMotorEx LF;
    private DcMotorEx RF;
    private DcMotorEx LB;
    private DcMotorEx RB;

    private SparkFunOTOS otos;

    // Each wheel is normalized against its own motor's max ticks/second.
    private double LF_MAX_TPS;
    private double RF_MAX_TPS;
    private double LB_MAX_TPS;
    private double RB_MAX_TPS;

    private boolean otosCalibrationSucceeded;

    // =========================================================
    // AUTOMATION STATE
    // =========================================================

    private enum AutoState {
        WAITING,
        DRIVE_SIDE,
        BRAKE_BEFORE_TURN,
        TURN_CORNER,
        BRAKE_AFTER_TURN,
        FINISHED,
        ABORTED
    }

    private AutoState state = AutoState.WAITING;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private int sideIndex = 0;

    // Start pose of the current straight segment.
    private double driveStartX;
    private double driveStartY;
    private double driveHeading;
    private double turnTargetHeading;

    // Start pose of the entire square, used to measure closure error.
    private double squareStartX;
    private double squareStartY;
    private double squareStartHeading;

    // Diagnostic measurements.
    private double lastProgress;
    private double lastCrossTrack;
    private double lastHeadingError;
    private double maximumAbsoluteCrossTrack;
    private double maximumAbsoluteHeadingError;
    private double finalPositionError;
    private double finalHeadingError;

    private String abortReason = "";

    // =========================================================
    // CSV DATA LOGGING
    // =========================================================

    // Logs at approximately 20 Hz. Each square run gets its own CSV file.
    private static final double CSV_LOG_INTERVAL_SEC = 0.05;
    private static final int CSV_FLUSH_EVERY_ROWS = 10;

    private BufferedWriter csvWriter = null;
    private File csvFile = null;
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime csvLogTimer = new ElapsedTime();
    private int csvRowsSinceFlush = 0;
    private String csvStatus = "No run logged yet";

    // Current ramped commands for each wheel, normalized from -1 to +1.
    private double LF_command = 0.0;
    private double RF_command = 0.0;
    private double LB_command = 0.0;
    private double RB_command = 0.0;

    // =========================================================
    // INITIALIZATION
    // =========================================================

    @Override
    public void runOpMode() {

        // Get the four mecanum drive motors.
        LF = hardwareMap.get(DcMotorEx.class, "LeftFrontDrive");
        RF = hardwareMap.get(DcMotorEx.class, "RightFrontDrive");
        LB = hardwareMap.get(DcMotorEx.class, "LeftBackDrive");
        RB = hardwareMap.get(DcMotorEx.class, "RightBackDrive");

        // Match the motor directions from your existing DriveClass.
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use encoder-based velocity control on all four motors.
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Read the configured maximum ticks per second for each motor.
        LF_MAX_TPS = getMaxTicksPerSecond(LF);
        RF_MAX_TPS = getMaxTicksPerSecond(RF);
        LB_MAX_TPS = getMaxTicksPerSecond(LB);
        RB_MAX_TPS = getMaxTicksPerSecond(RB);

        // Get and configure the OTOS.
        otos = hardwareMap.get(SparkFunOTOS.class, OTOS_HARDWARE_NAME);
        configureOTOS();

        stopImmediately();

        telemetry.addLine("BALL BOT MECANUM SQUARE TEST");
        telemetry.addLine("----------------------------");
        telemetry.addLine("Gamepad 1 A = Start / Run Again");
        telemetry.addLine("Gamepad 1 B = Cancel Current Run");
        telemetry.addData("Square Side", "%.1f inches", SIDE_LENGTH);
        telemetry.addData("OTOS", OTOS_HARDWARE_NAME);
        telemetry.addData("OTOS Calibration", otosCalibrationSucceeded ? "Successful" : "FAILED");
        telemetry.addLine("CSV files: /sdcard/FIRST/BallBotStraightness/");
        telemetry.addLine("Keep the robot stationary during calibration.");
        telemetry.update();

        waitForStart();

        // Do not drive if the OTOS could not complete calibration.
        if (isStopRequested() || !otosCalibrationSucceeded) {
            stopImmediately();
            return;
        }

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.reset();

        boolean aPreviouslyPressed = false;

        // =====================================================
        // MAIN LOOP
        // =====================================================

        while (opModeIsActive()) {

            // Delta time keeps ramp rates consistent if loop timing varies.
            double dt = Math.max(0.001, Math.min(0.05, loopTimer.seconds()));
            loopTimer.reset();

            // OTOS units were set to inches and degrees during configuration.
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            double x = pose.x;
            double y = pose.y;
            double heading = pose.h;

            // Motor targets default to zero. A state must request motion.
            double targetLF = 0.0;
            double targetRF = 0.0;
            double targetLB = 0.0;
            double targetRB = 0.0;

            // A rising edge starts or restarts the square.
            boolean aNow = gamepad1.a;
            if ((state == AutoState.WAITING || state == AutoState.FINISHED || state == AutoState.ABORTED)
                    && aNow && !aPreviouslyPressed) {
                startSquare(pose);
            }

            // B cancels any active portion of the path.
            if (gamepad1.b && isRunningState()) {
                abortRun("Cancelled by Gamepad B");
            }

            if (state != AutoState.ABORTED) {
                switch (state) {

                    case WAITING:
                        break;

                    // -------------------------------------------------
                    // Drive one side while holding heading and path.
                    // -------------------------------------------------
                    case DRIVE_SIDE: {

                        double dx = x - driveStartX;
                        double dy = y - driveStartY;
                        double h = Math.toRadians(driveHeading);

                        // OTOS convention: at 0 degrees, forward is +Y.
                        // Forward unit vector at heading h is (-sin(h), cos(h)).
                        double progress = -dx * Math.sin(h) + dy * Math.cos(h);

                        // Positive cross-track means the robot drifted to its right.
                        // A negative strafe command corrects that drift.
                        double crossTrack = dx * Math.cos(h) + dy * Math.sin(h);

                        double remaining = SIDE_LENGTH - progress;

                        lastProgress = progress;
                        lastCrossTrack = crossTrack;

                        double headingError = angleError(driveHeading, heading);
                        lastHeadingError = headingError;

                        maximumAbsoluteCrossTrack = Math.max(
                                maximumAbsoluteCrossTrack, Math.abs(crossTrack));
                        maximumAbsoluteHeadingError = Math.max(
                                maximumAbsoluteHeadingError, Math.abs(headingError));

                        if (stateTimer.seconds() > DRIVE_TIMEOUT) {
                            abortRun("Drive timeout on side " + (sideIndex + 1));
                            break;
                        }

                        if (remaining <= DISTANCE_TOLERANCE) {
                            state = AutoState.BRAKE_BEFORE_TURN;
                            stateTimer.reset();
                            break;
                        }

                        // Reduce the commanded speed as the endpoint approaches.
                        double forwardCommand = MAX_DRIVE;
                        if (remaining < BRAKE_DISTANCE) {
                            double ratio = remaining / BRAKE_DISTANCE;
                            ratio = clamp(ratio, 0.0, 1.0);
                            forwardCommand = MIN_DRIVE + (MAX_DRIVE - MIN_DRIVE) * ratio;
                        }

                        // Correct any rotation away from the intended heading.
                        double turnCorrection = 0.0;
                        if (Math.abs(headingError) > HEADING_DEADBAND) {
                            turnCorrection = clamp(
                                    HEADING_KP * headingError,
                                    -MAX_HEADING_CORRECTION,
                                    MAX_HEADING_CORRECTION);
                        }

                        // Correct sideways drift relative to this segment's path.
                        double strafeCorrection = clamp(
                                -CROSS_TRACK_KP * crossTrack,
                                -MAX_STRAFE_CORRECTION,
                                MAX_STRAFE_CORRECTION);

                        double[] wheelTargets = mecanumMix(
                                forwardCommand,
                                strafeCorrection,
                                turnCorrection);

                        targetLF = wheelTargets[0];
                        targetRF = wheelTargets[1];
                        targetLB = wheelTargets[2];
                        targetRB = wheelTargets[3];
                        break;
                    }

                    // -------------------------------------------------
                    // Ramp all four wheels down before a corner.
                    // -------------------------------------------------
                    case BRAKE_BEFORE_TURN: {
                        if (stateTimer.seconds() > BRAKE_TIMEOUT) {
                            abortRun("Could not stop before turning");
                            break;
                        }

                        if (wheelsSettled()) {
                            turnTargetHeading = wrapDegrees(driveHeading + TURN_ANGLE);
                            state = AutoState.TURN_CORNER;
                            stateTimer.reset();
                        }
                        break;
                    }

                    // -------------------------------------------------
                    // Rotate counterclockwise by 90 degrees.
                    // -------------------------------------------------
                    case TURN_CORNER: {
                        double error = angleError(turnTargetHeading, heading);
                        lastHeadingError = error;
                        maximumAbsoluteHeadingError = Math.max(
                                maximumAbsoluteHeadingError, Math.abs(error));

                        if (stateTimer.seconds() > TURN_TIMEOUT) {
                            abortRun("Turn timeout at corner " + (sideIndex + 1));
                            break;
                        }

                        if (Math.abs(error) <= TURN_TOLERANCE) {
                            state = AutoState.BRAKE_AFTER_TURN;
                            stateTimer.reset();
                            break;
                        }

                        double turnCommand = clamp(TURN_KP * error, -MAX_TURN, MAX_TURN);
                        if (Math.abs(turnCommand) < MIN_TURN) {
                            turnCommand = Math.copySign(MIN_TURN, error);
                        }

                        // Positive turn is counterclockwise, matching the drive
                        // motor direction convention used in the team's DriveClass.
                        targetLF = turnCommand;
                        targetRF = -turnCommand;
                        targetLB = turnCommand;
                        targetRB = -turnCommand;
                        break;
                    }

                    // -------------------------------------------------
                    // Settle after the turn, then start the next side.
                    // -------------------------------------------------
                    case BRAKE_AFTER_TURN: {
                        if (stateTimer.seconds() > BRAKE_TIMEOUT) {
                            abortRun("Could not settle after turning");
                            break;
                        }

                        if (wheelsSettled()) {
                            if (sideIndex >= SIDE_COUNT - 1) {
                                // The final 90-degree turn closes the square's heading.
                                state = AutoState.FINISHED;
                                finalPositionError = Math.hypot(x - squareStartX, y - squareStartY);
                                finalHeadingError = angleError(squareStartHeading, heading);
                            } else {
                                sideIndex++;
                                beginDriveSide(pose, turnTargetHeading);
                                state = AutoState.DRIVE_SIDE;
                            }
                        }
                        break;
                    }

                    case FINISHED:
                        break;

                    case ABORTED:
                        break;
                }
            }

            // Send smoothed velocity targets to the four drive motors.
            if (state == AutoState.ABORTED) {
                stopImmediately();
            } else {
                applyRampedVelocity(targetLF, targetRF, targetLB, targetRB, dt);
            }

            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addLine("===== OTOS MECANUM SQUARE =====");
            telemetry.addData("State", state);
            telemetry.addData("Side", "%d / %d", Math.min(sideIndex + 1, SIDE_COUNT), SIDE_COUNT);
            telemetry.addLine("----------------------------");
            telemetry.addData("OTOS X", "%.2f in", x);
            telemetry.addData("OTOS Y", "%.2f in", y);
            telemetry.addData("OTOS Heading", "%.2f deg", heading);
            telemetry.addLine("----------------------------");
            telemetry.addData("Side Progress", "%.2f / %.2f in", lastProgress, SIDE_LENGTH);
            telemetry.addData("Sideways Drift", "%.3f in", lastCrossTrack);
            telemetry.addData("Heading Error", "%.2f deg", lastHeadingError);
            telemetry.addData("Max |Drift|", "%.3f in", maximumAbsoluteCrossTrack);
            telemetry.addData("Max |Heading Error|", "%.2f deg", maximumAbsoluteHeadingError);
            telemetry.addLine("----------------------------");
            telemetry.addData("Ramp Targets", "LF %.2f RF %.2f LB %.2f RB %.2f",
                    LF_command, RF_command, LB_command, RB_command);
            telemetry.addData("Actual Vel / Max", "LF %.2f RF %.2f LB %.2f RB %.2f",
                    LF.getVelocity() / LF_MAX_TPS,
                    RF.getVelocity() / RF_MAX_TPS,
                    LB.getVelocity() / LB_MAX_TPS,
                    RB.getVelocity() / RB_MAX_TPS);
            telemetry.addData("CSV Log", csvStatus);

            if (state == AutoState.FINISHED) {
                telemetry.addLine("===== SQUARE COMPLETE =====");
                telemetry.addData("Final Position Error", "%.2f in", finalPositionError);
                telemetry.addData("Final Heading Error", "%.2f deg", finalHeadingError);
                telemetry.addLine("Press A again to repeat.");
            } else if (state == AutoState.ABORTED) {
                telemetry.addData("ABORT REASON", abortReason);
                telemetry.addLine("Release B, then press A to retry.");
            } else if (state == AutoState.WAITING) {
                telemetry.addLine("Press A to start.");
            }

            telemetry.update();

            // Save time-series data while the run is active. A final row is
            // forced when the square finishes or is aborted.
            if (state == AutoState.FINISHED) {
                writeCsvSample(pose, "FINAL_FINISHED", true);
                closeCsvLogger();
            } else if (state == AutoState.ABORTED) {
                writeCsvSample(pose, "FINAL_ABORTED", true);
                closeCsvLogger();
            } else {
                writeCsvSample(pose, "SAMPLE", false);
            }

            aPreviouslyPressed = aNow;
        }

        stopImmediately();

        // If the OpMode is stopped manually, preserve one last sample too.
        if (csvWriter != null) {
            SparkFunOTOS.Pose2D finalPose = otos.getPosition();
            finalPositionError = Math.hypot(
                    finalPose.x - squareStartX,
                    finalPose.y - squareStartY);
            finalHeadingError = angleError(squareStartHeading, finalPose.h);
            writeCsvSample(finalPose, "OPMODE_STOPPED", true);
            closeCsvLogger();
        }
    }

    // =========================================================
    // CSV LOGGER
    // =========================================================

    /**
     * Creates a separate CSV file for each button-initiated square run.
     * Files are saved to /sdcard/FIRST/BallBotStraightness/.
     */
    private void openCsvLogger() {
        closeCsvLogger();

        try {
            File logDirectory = new File(
                    Environment.getExternalStorageDirectory(),
                    "FIRST/BallBotStraightness");

            if (!logDirectory.exists() && !logDirectory.mkdirs()) {
                throw new IOException("Could not create log directory: " + logDirectory);
            }

            String timestamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss_SSS", Locale.US).format(new Date());

            csvFile = new File(logDirectory,
                    "BallBotStraightness_" + timestamp + ".csv");

            csvWriter = new BufferedWriter(new FileWriter(csvFile, false));
            csvWriter.write(
                    "elapsed_s,event,state,side_index,side_length_in,side_progress_in,"
                            + "remaining_in,cross_track_in,heading_deg,target_heading_deg,heading_error_deg,"
                            + "otos_x_in,otos_y_in,side_start_x_in,side_start_y_in,"
                            + "square_start_x_in,square_start_y_in,square_start_heading_deg,"
                            + "lf_command_frac,rf_command_frac,lb_command_frac,rb_command_frac,"
                            + "lf_actual_tps,rf_actual_tps,lb_actual_tps,rb_actual_tps,"
                            + "lf_actual_frac,rf_actual_frac,lb_actual_frac,rb_actual_frac,"
                            + "max_abs_cross_track_in,max_abs_heading_error_deg,"
                            + "final_position_error_in,final_heading_error_deg,"
                            + "heading_kp,cross_track_kp,max_drive_frac,ramp_up_rate,ramp_down_rate,abort_reason");
            csvWriter.newLine();
            csvWriter.flush();

            csvStatus = csvFile.getAbsolutePath();
        } catch (Exception e) {
            csvWriter = null;
            csvFile = null;
            csvStatus = "CSV ERROR: " + e.getMessage();
        }
    }

    /** Writes one sensor/motor sample. The final row bypasses the sample interval. */
    private void writeCsvSample(
            SparkFunOTOS.Pose2D pose,
            String event,
            boolean force) {

        if (csvWriter == null) {
            return;
        }

        if (!force && csvLogTimer.seconds() < CSV_LOG_INTERVAL_SEC) {
            return;
        }

        csvLogTimer.reset();

        try {
            double heading = pose.h;
            double targetHeading = (state == AutoState.TURN_CORNER)
                    ? turnTargetHeading : driveHeading;

            StringBuilder row = new StringBuilder(512);
            appendCsvField(row, runTimer.seconds());
            appendCsvField(row, event);
            appendCsvField(row, state.toString());
            appendCsvField(row, sideIndex + 1);
            appendCsvField(row, SIDE_LENGTH);
            appendCsvField(row, lastProgress);
            appendCsvField(row, SIDE_LENGTH - lastProgress);
            appendCsvField(row, lastCrossTrack);
            appendCsvField(row, heading);
            appendCsvField(row, targetHeading);
            appendCsvField(row, lastHeadingError);
            appendCsvField(row, pose.x);
            appendCsvField(row, pose.y);
            appendCsvField(row, driveStartX);
            appendCsvField(row, driveStartY);
            appendCsvField(row, squareStartX);
            appendCsvField(row, squareStartY);
            appendCsvField(row, squareStartHeading);

            appendCsvField(row, LF_command);
            appendCsvField(row, RF_command);
            appendCsvField(row, LB_command);
            appendCsvField(row, RB_command);

            double lfTps = LF.getVelocity();
            double rfTps = RF.getVelocity();
            double lbTps = LB.getVelocity();
            double rbTps = RB.getVelocity();

            appendCsvField(row, lfTps);
            appendCsvField(row, rfTps);
            appendCsvField(row, lbTps);
            appendCsvField(row, rbTps);

            appendCsvField(row, lfTps / LF_MAX_TPS);
            appendCsvField(row, rfTps / RF_MAX_TPS);
            appendCsvField(row, lbTps / LB_MAX_TPS);
            appendCsvField(row, rbTps / RB_MAX_TPS);

            appendCsvField(row, maximumAbsoluteCrossTrack);
            appendCsvField(row, maximumAbsoluteHeadingError);
            appendCsvField(row, finalPositionError);
            appendCsvField(row, finalHeadingError);

            appendCsvField(row, HEADING_KP);
            appendCsvField(row, CROSS_TRACK_KP);
            appendCsvField(row, MAX_DRIVE);
            appendCsvField(row, RAMP_UP_RATE);
            appendCsvField(row, RAMP_DOWN_RATE);
            appendCsvField(row, abortReason);

            csvWriter.write(row.toString());
            csvWriter.newLine();

            csvRowsSinceFlush++;
            if (force || csvRowsSinceFlush >= CSV_FLUSH_EVERY_ROWS) {
                csvWriter.flush();
                csvRowsSinceFlush = 0;
            }
        } catch (IOException e) {
            csvStatus = "CSV WRITE ERROR: " + e.getMessage();
            closeCsvLogger();
        }
    }

    private void appendCsvField(StringBuilder row, String value) {
        if (row.length() > 0) row.append(',');
        if (value == null) {
            row.append("\"\"");
        } else {
            row.append('"');
            row.append(value.replace("\"", "\"\""));
            row.append('"');
        }
    }

    private void appendCsvField(StringBuilder row, double value) {
        if (row.length() > 0) row.append(',');
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            row.append("NaN");
        } else {
            row.append(String.format(Locale.US, "%.6f", value));
        }
    }

    private void appendCsvField(StringBuilder row, int value) {
        if (row.length() > 0) row.append(',');
        row.append(value);
    }

    private void closeCsvLogger() {
        if (csvWriter != null) {
            try {
                csvWriter.flush();
                csvWriter.close();
            } catch (IOException e) {
                csvStatus = "CSV CLOSE ERROR: " + e.getMessage();
            } finally {
                csvWriter = null;
            }
        }
    }

    // =========================================================
    // OTOS CONFIGURATION
    // =========================================================

    private void configureOTOS() {
        telemetry.addLine("Configuring SparkFun OTOS...");
        telemetry.addLine("Keep robot completely still and flat.");
        telemetry.update();

        // Pose measurements will be returned in inches and degrees.
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Tell OTOS where it is mounted relative to the robot center.
        otos.setOffset(new SparkFunOTOS.Pose2D(
                OTOS_OFFSET_X,
                OTOS_OFFSET_Y,
                OTOS_OFFSET_H));

        // Start with factory-neutral scaling. Calibrate these separately later.
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // The robot must not move during IMU calibration.
        otosCalibrationSucceeded = otos.calibrateImu();

        // Reset tracking so the square begins at (0, 0, 0).
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    // =========================================================
    // START / SEGMENT SETUP
    // =========================================================

    private void startSquare(SparkFunOTOS.Pose2D pose) {
        sideIndex = 0;
        abortReason = "";

        squareStartX = pose.x;
        squareStartY = pose.y;
        squareStartHeading = pose.h;

        maximumAbsoluteCrossTrack = 0.0;
        maximumAbsoluteHeadingError = 0.0;
        finalPositionError = 0.0;
        finalHeadingError = 0.0;

        beginDriveSide(pose, pose.h);
        state = AutoState.DRIVE_SIDE;

        // Start a new log for this particular square attempt.
        runTimer.reset();
        csvLogTimer.reset();
        csvRowsSinceFlush = 0;
        openCsvLogger();
        writeCsvSample(pose, "RUN_START", true);
    }

    private void beginDriveSide(SparkFunOTOS.Pose2D pose, double desiredHeading) {
        driveStartX = pose.x;
        driveStartY = pose.y;
        driveHeading = wrapDegrees(desiredHeading);

        lastProgress = 0.0;
        lastCrossTrack = 0.0;
        lastHeadingError = 0.0;

        stateTimer.reset();
    }

    // =========================================================
    // MECANUM MIXING
    // =========================================================

    private double[] mecanumMix(
            double forward,
            double strafeRight,
            double turnCounterclockwise) {

        double lf = forward + strafeRight + turnCounterclockwise;
        double rf = forward - strafeRight - turnCounterclockwise;
        double lb = forward - strafeRight + turnCounterclockwise;
        double rb = forward + strafeRight - turnCounterclockwise;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))));

        return new double[]{lf / max, rf / max, lb / max, rb / max};
    }

    // =========================================================
    // INDEPENDENT MOTOR RAMPING
    // =========================================================

    private void applyRampedVelocity(
            double targetLF,
            double targetRF,
            double targetLB,
            double targetRB,
            double dt) {

        LF_command = ramp(LF_command, targetLF, dt);
        RF_command = ramp(RF_command, targetRF, dt);
        LB_command = ramp(LB_command, targetLB, dt);
        RB_command = ramp(RB_command, targetRB, dt);

        LF.setVelocity(LF_command * LF_MAX_TPS);
        RF.setVelocity(RF_command * RF_MAX_TPS);
        LB.setVelocity(LB_command * LB_MAX_TPS);
        RB.setVelocity(RB_command * RB_MAX_TPS);
    }

    private double ramp(double current, double target, double dt) {
        // When changing direction, ramp down to zero before ramping the other way.
        if (current * target < 0.0) {
            return approach(current, 0.0, RAMP_DOWN_RATE * dt);
        }

        // Use a gentler rate to accelerate and a faster rate to decelerate.
        double rate = Math.abs(target) < Math.abs(current)
                ? RAMP_DOWN_RATE
                : RAMP_UP_RATE;

        return approach(current, target, rate * dt);
    }

    private double approach(double current, double target, double maxStep) {
        double difference = target - current;
        double step = Math.min(Math.abs(difference), maxStep);
        return current + Math.copySign(step, difference);
    }

    // =========================================================
    // STOPPING / MOTOR HELPERS
    // =========================================================

    private boolean wheelsSettled() {
        boolean commandsStopped =
                Math.abs(LF_command) < 0.015 &&
                        Math.abs(RF_command) < 0.015 &&
                        Math.abs(LB_command) < 0.015 &&
                        Math.abs(RB_command) < 0.015;

        boolean actualStopped =
                Math.abs(LF.getVelocity()) < LF_MAX_TPS * 0.035 &&
                        Math.abs(RF.getVelocity()) < RF_MAX_TPS * 0.035 &&
                        Math.abs(LB.getVelocity()) < LB_MAX_TPS * 0.035 &&
                        Math.abs(RB.getVelocity()) < RB_MAX_TPS * 0.035;

        return commandsStopped && actualStopped;
    }

    private double getMaxTicksPerSecond(DcMotorEx motor) {
        double max = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        return Math.max(1.0, max);
    }

    private void stopImmediately() {
        LF.setVelocity(0.0);
        RF.setVelocity(0.0);
        LB.setVelocity(0.0);
        RB.setVelocity(0.0);

        LF_command = 0.0;
        RF_command = 0.0;
        LB_command = 0.0;
        RB_command = 0.0;
    }

    private void abortRun(String reason) {
        abortReason = reason;
        state = AutoState.ABORTED;

        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
        finalPositionError = Math.hypot(
                currentPose.x - squareStartX,
                currentPose.y - squareStartY);
        finalHeadingError = angleError(squareStartHeading, currentPose.h);

        stopImmediately();
    }

    private boolean isRunningState() {
        return state == AutoState.DRIVE_SIDE ||
                state == AutoState.BRAKE_BEFORE_TURN ||
                state == AutoState.TURN_CORNER ||
                state == AutoState.BRAKE_AFTER_TURN;
    }

    // =========================================================
    // ANGLE / MATH HELPERS
    // =========================================================

    private double angleError(double target, double current) {
        double error = target - current;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;
        return error;
    }

    private double wrapDegrees(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}