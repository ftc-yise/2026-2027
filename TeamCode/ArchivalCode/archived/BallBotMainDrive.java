package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Ledclass;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@TeleOp(name = "BallBot", group = "Ball Bot")
public class BallBotMainDrive extends LinearOpMode {

    // --- Turret Logic Variables ---
    private enum SnapState { INACTIVE, SNAPPING_LEFT, PUSHING_LEFT, SNAPPING_RIGHT, PUSHING_RIGHT, GOING_HOME, HOMING_ROUTINE }
    private SnapState currentSnapState = SnapState.INACTIVE;
    private ElapsedTime homingTimer = new ElapsedTime();
    private ElapsedTime snapTimer = new ElapsedTime();
    private boolean modeTogglePressed = false;

    private static final int LEFT_LIMIT = -1370;
    public boolean once = true;
    private static final int CENTER_TARGET = -685;
    private static final int TOLERANCE = 10;
    private boolean shooting = false;

    // ------------------------------

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;
    private DcMotor foot = null;

    private ColorSensor middle = null;
    private ColorSensor backLeft = null;
    private ColorSensor backRight = null;
    private ColorSensor BRC = null;
    private ColorSensor BLC = null;
    private ColorSensor FLC = null;
    private ColorSensor FRC = null;

    public Ledclass led1;
    public Ledclass led2;
    public Ledclass led3;
    // edge detectors for pattern-vs-forced behavior
    private boolean prevA = false;
    private boolean prevX = false;
    boolean a;
    boolean x;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime ledTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05;
    public double time = runtime.seconds();

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }
    private ShotPatternManager.ShotPattern activePattern = null;
    boolean firstRun = true;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveClass drive = new DriveClass(hardwareMap);
        ShotPatternManager patternMgr = new ShotPatternManager();
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        autoShoot.setPatternManager(patternMgr);
        Parameters parem = new Parameters();
        parem.autonomous = Parameters.AUTONOMOUS.NO;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }

        Turret turret = new Turret(hardwareMap, alliance, telemetry);

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        foot = hardwareMap.get(DcMotor.class, "foot");
        wallright.setDirection(CRServo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");

        middle = hardwareMap.get(ColorSensor.class, "middlecolorsensor");
        backLeft = hardwareMap.get(ColorSensor.class, "BLcolorsensor");
        backRight = hardwareMap.get(ColorSensor.class, "BRcolorsensor");
        BLC = hardwareMap.get(ColorSensor.class, "BLC");
        BRC = hardwareMap.get(ColorSensor.class, "BRC");
        FLC = hardwareMap.get(ColorSensor.class, "FLC");
        FRC = hardwareMap.get(ColorSensor.class, "FRC");

        led1 = new Ledclass(hardwareMap, "led1");
        led2 = new Ledclass(hardwareMap, "led2");
        led3 = new Ledclass(hardwareMap, "led3");

        hood.stop();

        //spin.initSilos();

        //spin.goToSilo2();
        lifter.setDown();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            time = runtime.seconds();

            if (Parameters.allianceColor == Parameters.Color.RED) {
                alliance = Turret.turretAlliance.RED;
            } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
                alliance = Turret.turretAlliance.BLUE;
            }

            // --- DRIVE & SPEED TOGGLE ---
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1, false);


            // --- PATTERN vs FORCED SHOOTING ---
// Behavior:
//  - On button EDGE (press), try pattern-aware startCycle() if the pattern can be satisfied.
//  - If pattern cannot be satisfied, start forced-cycle while button is HELD.
//  - If neither button held and we were forcing, stopForcedCycle().

            if (!a && !x && gamepad2.a && !prevX && !prevA) {
                a = true;
                shooting = true;
            } else if (!a && !x && gamepad2.x && !prevX && !prevA) {
                x = true;
                shooting = true;
            }

            if (!gamepad2.a && autoShoot.shots > 2) {
                a = false;
                //autoShoot.shots = 0;
                shooting = false;
            }
            if (!gamepad2.x && autoShoot.shots > 2) {
                x = false;
                //autoShoot.shots = 0;
                shooting = false;
            }

            if (!a && !x && !gamepad2.x && !prevX && !prevA && !gamepad2.a && autoShoot.shots >= 3){
                autoShoot.shots = 0;
            }

// --- A button (high-goal) ---
            if (a && !prevA) {
                // edge: attempt pattern-aware cycle if possible
                if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                    shooter.update(false, false, true);
                    hood.setTarget(75);
                    autoShoot.startCycle();
                } else {
                    // pattern not satisfiable on-press -> start forced cycling while held
                    shooter.update(false, false, true);
                    hood.setTarget(75);
                    if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                        autoShoot.startForcedCycle();
                    }
                }
            } else if (a) {
                // held: ensure forced mode is active
                shooter.update(false, false, true);
                hood.setTarget(75);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            }

// --- X button (lower-goal) ---
            if (x && !prevX) {
                // edge: attempt pattern-aware cycle if possible
                if (canSatisfyPattern(patternMgr, spin) && !autoShoot.isBusy()) {
                    shooter.update(false, true, false);
                    hood.setTarget(26);
                    autoShoot.startCycle();
                } else {
                    // pattern not satisfiable on-press -> start forced cycling while held
                    shooter.update(false, true, false);
                    hood.setTarget(26);
                    if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                        autoShoot.startForcedCycle();
                    }
                }
            } else if (x) {
                // held: ensure forced mode is active
                shooter.update(false, true, false);
                hood.setTarget(26);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            }

// If neither button is currently held, stop forced mode if it was active
            if (!a && !x && autoShoot.forceShooting) {
                autoShoot.stopForcedCycle();
            }

// Always update shooter state machine once per loop
            autoShoot.update();

// update previous edge flags for next loop
            prevA = a;
            prevX = x;

            /*
            // --- SHOOTING & SPINDEXOR ---
            if (gamepad2.a && !autoShoot.isBusy()) {
                shooter.update(false, false, true);
                hood.setTarget(40);

                if (activePattern != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(activePattern.sequence);
                }

                autoShoot.startCycle();
                shooting = true;
                //spin.goToSilo1();
            }
            else if (gamepad2.x && !autoShoot.isBusy()){
                shooter.update(false, true, false);
                hood.setTarget(15); // e.g. 0
                if (activePattern != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(activePattern.sequence);
                }

                autoShoot.startCycle();
                shooting = true;
                //spin.goToSilo2();
            }
            autoShoot.update();
            */
            /*
            // --- SHOOTING & SPINDEXOR (forced override when holding A or X) ---
            if (gamepad2.a) {
                // start forced-fire if not already
                shooter.update(false, false, true);    // shooter high goal
                hood.setTarget(40);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            } else if (gamepad2.x) {
                shooter.update(false, true, false);    // shooter lower goal
                hood.setTarget(15);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            } else {
                // if neither held, and we're in forced-mode, stop forced mode
                if (autoShoot.forceShooting) {
                    autoShoot.stopForcedCycle();
                }
                // normal idle behavior handled elsewhere
            }
            autoShoot.update();
            */

            //vision/pattern things

            if (gamepad1.y) {
                turret.limelight.pipelineSwitch(2);
                turret.runto(430); // keep this per your request

                int tagId = turret.getID();
                ShotPatternManager.ShotPattern p = patternFromTag(tagId);
                if (p != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(p.sequence);
                }

            }
            else {
                if (Parameters.allianceColor == Parameters.Color.RED) {
                    alliance = Turret.turretAlliance.RED;
                    turret.limelight.pipelineSwitch(4);
                } else {
                    alliance = Turret.turretAlliance.BLUE;
                    turret.limelight.pipelineSwitch(3);
                }
            }


            if ((turret.getID() == 20 || turret.getID() == 24 && !shooting)){
                led1.setBlue();
                led2.setBlue();
                led3.setBlue();
            } else if (time > 1) {
                led1.setOff();
                led2.setOff();
                led3.setOff();
                ledTimer.reset();
            } else if (shooting){
                led1.setGreen();
                led2.setGreen();
                led3.setGreen();
            }

            // --- INTAKE & WALL WHEELS ---
            if (gamepad1.right_trigger > 0.75 && !shooting) {
                intake.setPower(1);
                led1.setBlue();
                walleft.setPower(1);
                wallright.setPower(1);
                spin.startManualCycle();
                once = true;

            } else if (gamepad1.left_trigger > .75 && !shooting) {
                intake.setPower(-1);
                walleft.setPower(-1);
                wallright.setPower(-1);
                //spin.setManual(.6);
                once = true;
            } else if (gamepad1.right_bumper && !shooting) {
                intake.setPower(-.6);
                walleft.setPower(-1);
                wallright.setPower(-1);
                once = true;
            } else {
                intake.setPower(0);
                walleft.setPower(0.25);
                wallright.setPower(0.25);
                if (!autoShoot.isBusy() && gamepad1.right_trigger < 0.75 && gamepad1.left_trigger < 0.75) {
                    if (gamepad2.right_stick_button && gamepad2.left_stick_button){
                        Parameters.spinLocation = spin.getTelemetry().currentAngle;
                        spin.initSilos();
                    }

                    if (once) {
                        lifter.setDown();
                        once = false;
                        spin.setNeutral();
                        shooting = false;
                    }
                }
            }



            // --- HOOD & FOOT ---
            if (gamepad1.dpad_down) {
                foot.setPower(0.557);
                //spin.goToPose(0);
            }
            else if (gamepad1.dpad_left) {
                spin.goToPose(0);
            }
            else if (gamepad1.dpad_up) {
                foot.setPower(-0.25);
                //spin.goToPose(0.5);
            }else if (gamepad1.dpad_right) {
                foot.setPower(-0.25);
                spin.goToPose(1);
            }

            if (gamepad2.dpad_up && !autoShoot.isBusy()) {
                hood.setTarget(24); // e.g. 42.0
                lifter.setUp();
                shooting = true;
            }
            else if (gamepad2.dpad_down && !autoShoot.isBusy()) {
                hood.setTarget(0); // e.g. 42.0
                lifter.setDown();
                shooting = true;
            }

            // =========================================================
            // --- TURRET SYSTEM (INTEGRATED STATE MACHINE) ---
            // =========================================================

            // 1. Mode Toggle (y) & Homing (Share)
            if (gamepad2.y && !modeTogglePressed) {
                turret.changeMode();
                currentSnapState = SnapState.INACTIVE;
                modeTogglePressed = true;
            }
            if (!gamepad2.y) {
                modeTogglePressed = false;
            }

            if (gamepad2.share) {
                currentSnapState = SnapState.HOMING_ROUTINE;
                homingTimer.reset();
            }

            // 2. Input Detection (Manual Mode only)
            if (turret.mode == Turret.turretMode.MANUAL) {
                double turretManualTrigger = (gamepad2.left_trigger - gamepad2.right_trigger) * -.8;

                if (Math.abs(turretManualTrigger) > 0.05) {
                    currentSnapState = SnapState.INACTIVE;
                    turret.manualControl(turretManualTrigger);
                }
                else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    currentSnapState = SnapState.GOING_HOME;
                }
                else if (gamepad2.right_bumper && currentSnapState != SnapState.SNAPPING_RIGHT && currentSnapState != SnapState.PUSHING_RIGHT) {
                    currentSnapState = SnapState.SNAPPING_RIGHT;
                }
                else if (gamepad2.left_bumper && currentSnapState != SnapState.SNAPPING_LEFT && currentSnapState != SnapState.PUSHING_LEFT) {
                    currentSnapState = SnapState.SNAPPING_LEFT;
                }
            }

            // 3. State Machine Execution
            if (turret.mode == Turret.turretMode.AUTO) {
                turret.autoMode();
                turret.mode = Turret.turretMode.AUTO;
            }
            else if (currentSnapState == SnapState.HOMING_ROUTINE) {
                if (gamepad2.share) {
                    if (!turret.limit.getState()) {
                        turret.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        turret.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turret.manualControl(0.4);
                    } else {
                        double curvePower = 0.9 - (homingTimer.seconds() * 0.33);
                        turret.manualControl(Range.clip(curvePower, 0.4, 0.9));
                    }
                } else {
                    currentSnapState = SnapState.INACTIVE;
                    turret.stop();
                }
            }
            else if (currentSnapState == SnapState.GOING_HOME) {
                int currentPos = turret.turret.getCurrentPosition();
                int error = CENTER_TARGET - currentPos;
                if (Math.abs(error) > TOLERANCE) {
                    double homePower = (error > 0) ? 1.0 : -1.0;
                    if (Math.abs(error) < 200) homePower *= 0.4;
                    turret.manualControl(homePower);
                } else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            else if (currentSnapState == SnapState.SNAPPING_RIGHT) {
                turret.manualControl(1.0);
                if (!turret.limit.getState()) { snapTimer.reset(); currentSnapState = SnapState.PUSHING_RIGHT; }
            }
            else if (currentSnapState == SnapState.PUSHING_RIGHT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(0.5);
                else { turret.stop(); currentSnapState = SnapState.INACTIVE; }
            }
            else if (currentSnapState == SnapState.SNAPPING_LEFT) {
                int currentPos = turret.turret.getCurrentPosition();
                turret.manualControl(-1.0);
                if (currentPos <= LEFT_LIMIT) { snapTimer.reset(); currentSnapState = SnapState.PUSHING_LEFT; }
            }
            else if (currentSnapState == SnapState.PUSHING_LEFT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(-0.5);
                else { turret.stop(); currentSnapState = SnapState.INACTIVE; }
            }
            else if (turret.mode == Turret.turretMode.MANUAL && currentSnapState == SnapState.INACTIVE && Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) <= 0.05) {
                turret.stop();
            }


            if (logWriter == null) {
                try {
                    File dir = new File("/sdcard/FIRST");
                    if (!dir.exists()) dir.mkdirs();

                    String timestamp = new SimpleDateFormat(
                            "yyyyMMdd_HHmmss",
                            Locale.US
                    ).format(new Date());

                    logFilePath = "/sdcard/FIRST/telemetry_" + timestamp + ".csv";
                    logWriter = new PrintWriter(new FileWriter(logFilePath));

                    logWriter.println(
                            "time_s," +
                                    "input_x,input_y,input_turn," +
                                    "trans_x,trans_y,rotation_cmd," +
                                    "lf,rf,lb,rb,total_power," +
                                    "pose_x,pose_y,pose_h," +
                                    "spx_mode,spx_currentAngle,spx_targetAngle,spx_error,spx_appliedPower," +
                                    "silo1,silo2,silo3," +

                                    // button states for safety/debug
                                    "btn_a,btn_x,btn_g1_rt,btn_g1_lt,btn_g1_rb,btn_g2_y," +

                                    // per-press times (seconds)
                                    "press_shot1,press_shot2,press_shot3,press_avg," +

                                    // global averages (seconds)
                                    "avg_shot1,avg_shot2,avg_shot3,avg_overall"
                    );

                    logWriter.flush();
                    logTimer.reset();

                    telemetry.addData("LOG", "Started");
                    telemetry.addData("File", logFilePath);
                    telemetry.update();

                    sleep(300); // debounce
                } catch (IOException e) {
                    telemetry.addData("LOG ERROR", e.getMessage());
                    telemetry.update();
                }
            }


            if (gamepad1.a) {
                patternMgr.clear();
            }

            spin.sampleSensorsNow();
            spin.update();             // 2️⃣ process logic
            Spindexer.TelemetryPacket spina = spin.getTelemetry(); // 3️⃣ snapshot
            //telemetry getter
            DriveClass.DriveTelemetry d = drive.getDriveTelemetry();
            // --- Update spindexer & autoShoot ---
            hood.update();
            Hood.TelemetryPacket H = hood.getTelemetry();
            lifter.TelemetryPacket l = lifter.getTelemetry();


// --- TELEMETRY ---
// SHOOTER
            /*
            shooter.updateTelemetry();
            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Target RPM", "%.2f", s.targetRPM);
            telemetry.addData("Current RPM", "%.1f", s.currentRPM);
            telemetry.addData("Error RPM", "%.1f", s.errorRPM);
            telemetry.addData("volt", s.motorPower);
            telemetry.addData("pose", s.pose);
            telemetry.addData("spin up time", s.spinupTimeSec);

// DRIVE
            // --- SHOOTER TIMING TELEMETRY ---
            double[] lastShotsTelemetry = autoShoot.getLastCompletedPressShotTimes();
            telemetry.addLine("=== SHOOT TIMING ===");
            telemetry.addData("Press Shot1", "%.3f", lastShotsTelemetry[0]);
            telemetry.addData("Press Shot2", "%.3f", lastShotsTelemetry[1]);
            telemetry.addData("Press Shot3", "%.3f", lastShotsTelemetry[2]);
            telemetry.addData("Press Avg", "%.3f", autoShoot.getLastCompletedPressAverage());

            telemetry.addLine("--- GLOBAL AVERAGES ---");
            telemetry.addData("Avg Shot1", "%.3f", autoShoot.getGlobalAverageShot1());
            telemetry.addData("Avg Shot2", "%.3f", autoShoot.getGlobalAverageShot2());
            telemetry.addData("Avg Shot3", "%.3f", autoShoot.getGlobalAverageShot3());
            telemetry.addData("Avg Overall", "%.3f", autoShoot.getGlobalOverallAverage());

            // optional: show which buttons are pressed (safety)
            telemetry.addLine("--- BUTTONS ---");
            telemetry.addData("A", "%b", a);
            telemetry.addData("X", "%b", x);
            telemetry.addData("G1 RT>", "%b", gamepad1.right_trigger > 0.75);
            telemetry.addData("G1 LT>", "%b", gamepad1.left_trigger > 0.75);
            telemetry.addData("G1 RB", "%b", gamepad1.right_bumper);
            telemetry.addData("G2 Y", "%b", gamepad2.y);

            telemetry.addLine("=== FIELD DRIVE ===");
            telemetry.addData("Speed Mode", d.currentSpeed);
            telemetry.addData("Heading (deg)", "%.2f", d.headingDeg);
            telemetry.addData("Inputs (raw)", "x:%.2f y:%.2f t:%.2f", d.rawX, d.rawY, d.rawTurn);
            telemetry.addData("FieldCmd (tx,ty)", "%.3f, %.3f", d.tx_field, d.ty_field);
            telemetry.addData("RobotCmd (rx,ry)", "%.3f, %.3f", d.robotX, d.robotY);
            telemetry.addData("Motor LF/RF/LB/RB", "%.3f / %.3f / %.3f / %.3f", d.lf, d.rf, d.lb, d.rb);
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);

// SPINDEXER
            telemetry.addLine("=== SPINDEXER ===");
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Voltage", "%.3f", spina.voltage);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);
            telemetry.addData("A", a);
            telemetry.addData("X", x);
            telemetry.addData("shots", autoShoot.shots);

// TURRET
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("Power", turret.turretPower);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

// SILOS
            telemetry.addLine("=== SILOS ===");
            Spindexer.BallColor[] silos = spina.siloColors;
            for (int i = 0; i < silos.length; i++) {
                String label = "Silo " + (i+1);
                // Highlight the current silo
                telemetry.addData(label, silos[i]);
            }

// COLOR SENSORS
            telemetry.addLine("=== COLOR SENSORS ===");
            telemetry.addLine("Middle");
            telemetry.addData("Blue", middle.blue());
            telemetry.addData("Red", middle.red());
            telemetry.addData("Green", middle.green());

            telemetry.addLine("Back Left");
            telemetry.addData("Blue", backLeft.blue());
            telemetry.addData("Red", backLeft.red());
            telemetry.addData("Green", backLeft.green());

            telemetry.addLine("Back Right");
            telemetry.addData("Blue", backRight.blue());
            telemetry.addData("Red", backRight.red());
            telemetry.addData("Green", backRight.green());
//lift

            telemetry.addLine("=== LIFT ===");
            telemetry.addData("pose", l.position);
            telemetry.addData("volt", l.voltage);
            telemetry.addData("err", l.error);
            telemetry.addData("mode", l.mode);
            telemetry.addData("up", lifter.isUp());
            telemetry.addData("down", lifter.isDown());

//hood
             telemetry.addLine("=== HOOD ===");
            telemetry.addData("Mode", H.mode);
            telemetry.addData("Voltage", "%.3f", H.voltage);
            telemetry.addData("Angle", "%.1f°", H.currentAngle);
            telemetry.addData("Target", "%.1f°", H.targetAngle);
            telemetry.addData("Error", "%.1f°", H.angleError);
            telemetry.addData("Power", "%.2f", H.appliedPower);

           //floor sensors
            telemetry.addLine("=== FLOOR SENSORS ===");
            telemetry.addLine("BLC");
            telemetry.addData("Blue", BLC.blue());
            telemetry.addData("Red", BLC.red());
            telemetry.addData("Green", BLC.green());

            telemetry.addLine("BRC");
            telemetry.addData("Blue", BRC.blue());
            telemetry.addData("Red", BRC.red());
            telemetry.addData("Green", BRC.green());

            telemetry.addLine("FRC");
            telemetry.addData("Blue", FRC.blue());
            telemetry.addData("Red", FRC.red());
            telemetry.addData("Green", FRC.green());

            telemetry.addLine("FLC");
            telemetry.addData("Blue", FLC.blue());
            telemetry.addData("Red", FLC.red());
            telemetry.addData("Green", FLC.green());

           telemetry.update();
           */

            if (logWriter != null && logTimer.seconds() > 0.1) {

                double now = runtime.seconds();

                Spindexer.TelemetryPacket spx = spin.getTelemetry();

                double totalPower =
                        Math.abs(d.lf) + Math.abs(d.rf) +
                                Math.abs(d.lb) + Math.abs(d.rb);

                // --- gather shooter timing & averages (single synchronized calls to getters) ---
                double[] lastShots = autoShoot.getLastCompletedPressShotTimes(); // may contain NaN
                double lastPressAvg = autoShoot.getLastCompletedPressAverage();
                double globalAvg1 = autoShoot.getGlobalAverageShot1();
                double globalAvg2 = autoShoot.getGlobalAverageShot2();
                double globalAvg3 = autoShoot.getGlobalAverageShot3();
                double globalOverall = autoShoot.getGlobalOverallAverage();

                // button/safety flags
                boolean btnA = a; // edge state you already track
                boolean btnX = x;
                boolean g1Rt = gamepad1.right_trigger > 0.75;
                boolean g1Lt = gamepad1.left_trigger > 0.75;
                boolean g1Rb = gamepad1.right_bumper;
                boolean g2Y  = gamepad2.y;

                logWriter.printf(
                        "%.3f," +                                    // now
                                "%.4f,%.4f,%.4f," +                          // raw inputs
                                "%.4f,%.4f,%.4f," +                          // transformed inputs
                                "%.4f,%.4f,%.4f,%.4f,%.4f," +                // motor powers + total
                                "%.4f,%.4f,%.4f," +                          // pose x,y,h
                                "%s,%.2f,%.2f,%.2f,%.3f," +                 // spindexer (mode,current,target,error,power)
                                "%s,%s,%s," +                                // silo colors
                                "%b,%b,%b,%b,%b,%b," +                       // button states
                                "%.4f,%.4f,%.4f,%.4f," +                     // press shot1,shot2,shot3,press_avg
                                "%.4f,%.4f,%.4f,%.4f%n",                    // global avg1,avg2,avg3,overall
                        // values:
                        now,
                        d.rawX, d.rawY, d.rawTurn,
                        d.tx_field, d.ty_field, d.rotationCmd,
                        d.lf, d.rf, d.lb, d.rb, totalPower,
                        d.pose.x, d.pose.y, d.pose.h,
                        spx.mode,
                        spx.currentAngle,
                        spx.targetAngle,
                        spx.angleError,
                        spx.appliedPower,
                        spx.siloColors[0],
                        spx.siloColors[1],
                        spx.siloColors[2],
                        // button flags
                        btnA,
                        btnX,
                        g1Rt,
                        g1Lt,
                        g1Rb,
                        g2Y,
                        // per-press shot times (safe index checks)
                        (lastShots.length > 0 ? lastShots[0] : Double.NaN),
                        (lastShots.length > 1 ? lastShots[1] : Double.NaN),
                        (lastShots.length > 2 ? lastShots[2] : Double.NaN),
                        lastPressAvg,
                        // globals
                        globalAvg1,
                        globalAvg2,
                        globalAvg3,
                        globalOverall
                );

                logWriter.flush();
                logTimer.reset();
            }



        } // end while opModeIsActive

        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            telemetry.addData("LOG", "Saved");
            telemetry.addData("File", logFilePath);
            telemetry.update();
        }


    }

    /**
     * Returns true if the queued pattern (patternMgr.snapshot()) can be satisfied
     * by the current spindexer silo colors without reusing silos.
     */
    private boolean canSatisfyPattern(ShotPatternManager patternMgr, Spindexer spin) {
        if (patternMgr == null || !patternMgr.hasShots()) return false;

        Spindexer.BallColor[] queued = patternMgr.snapshot();
        // count contiguous queued entries at front
        int queuedCount = 0;
        for (int i = 0; i < queued.length; i++) {
            if (queued[i] == Spindexer.BallColor.NONE) break;
            queuedCount++;
        }
        if (queuedCount == 0) return false;

        Spindexer.TelemetryPacket spx = spin.getTelemetry();
        Spindexer.BallColor[] silos = spx.siloColors;
        boolean[] used = new boolean[silos.length];

        for (int q = 0; q < queuedCount; q++) {
            Spindexer.BallColor desired = queued[q];
            int found = -1;
            for (int s = 0; s < silos.length; s++) {
                if (!used[s] && silos[s] == desired) {
                    found = s;
                    break;
                }
            }
            if (found == -1) {
                return false; // cannot satisfy this queued color in order
            }
            used[found] = true;
        }
        return true;
    }


}