package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

/**
 * RedFarShootAuto
 *
 * This version is intentionally structured as a very explicit recipe.
 *
 * High-level idea:
 *   - Pedro owns driving.
 *   - ShooterExecutionClass owns the actual shooting cycle.
 *   - This OpMode only decides WHICH step comes next.
 *
 * The biggest difference from the older version is that there is no hidden
 * “if zone sensor says X, then maybe shoot, maybe skip, maybe re-run a path” logic.
 *
 * Instead, the autonomous routine is written as a sequence:
 *
 *   SHOOT -> PATH 0 -> SHOOT -> PATH 1 -> SHOOT -> PATH 2
 *
 * If you want the shorter version:
 *
 *   SHOOT -> PATH 0 -> SHOOT -> PATH 1 -> SHOOT
 *
 * remove the final PATH 2 step from ROUTINE below.
 *
 * This is the version I recommend for learning and for future edits,
 * because every action is visible in one list at the top of the class.
 */
@Autonomous(name = "[RED] Far Shoot Auto", group = "Auto")
public class RedFarShootAuto extends OpMode {

    // ---------------------------------------------------------
    // TUNING CONSTANTS
    // ---------------------------------------------------------
    // How long we wait after a path ends before starting a shoot step.
    // This gives the robot time to settle so the shooter is less likely to
    // fire while the chassis is still oscillating.
    private static final double PATH_SETTLE_SECONDS = 0.25;

    // Safety timeout for the shoot sequence.
    // If something goes wrong in the shooter cycle, we do not want to get stuck forever.
    private static final double SHOOT_TIMEOUT_SECONDS = 12.0;

    // Intake / wall wheel constants.
    private static final double WALL_WHEEL_POWER = 0.51;
    private static final double INTAKE_POWER = 1.0;

    // These are the paths where we want the intake/wall wheels active while moving.
    // In your current routine, path 0 and path 1 are intake travel paths.
    private static final int[] INTAKE_PATHS = {0, 1};

    // If you want a path where only the wall wheels run and intake stays off,
    // put that path index in this list.
    private static final int[] WALL_ONLY_PATHS = {2};

    // ---------------------------------------------------------
    // ROUTINE DEFINITION
    // ---------------------------------------------------------
    /**
     * This array is the entire autonomous plan.
     *
     * Read it top to bottom.
     *
     * Default order here:
     *   1. Shoot before moving.
     *   2. Run path 0.
     *   3. Shoot again.
     *   4. Run path 1.
     *   5. Shoot again.
     *   6. Run path 2 (optional parking / final move).
     *
     * If you want EXACTLY:
     *   shoot -> path -> shoot -> path -> shoot
     * then delete the last line:
     *   new Step(StepType.PATH, 2)
     */
    private static final class Step {
        final StepType type;
        final int pathIndex;

        Step(StepType type, int pathIndex) {
            this.type = type;
            this.pathIndex = pathIndex;
        }
    }

    private enum StepType {
        SHOOT,
        PATH
    }

    private static final Step[] ROUTINE = new Step[] {
            new Step(StepType.SHOOT, -1),
            new Step(StepType.PATH, 0),
            new Step(StepType.SHOOT, -1),
            new Step(StepType.PATH, 1),
            new Step(StepType.SHOOT, -1),
            new Step(StepType.PATH, 2)
    };

    // ---------------------------------------------------------
    // STATE MACHINE
    // ---------------------------------------------------------
    /**
     * START_STEP:
     *   Begin the next step in ROUTINE.
     *
     * WAIT_SHOOT:
     *   Let ShooterExecutionClass run until it finishes.
     *
     * WAIT_PATH:
     *   Let Pedro finish the path, then wait a short settle time.
     *
     * DONE:
     *   Clean shutdown.
     */
    private enum AutoState {
        START_STEP,
        WAIT_SHOOT,
        WAIT_PATH,
        DONE
    }

    private AutoState autoState = AutoState.START_STEP;

    // Index of the next step to start.
    // Example: 0 means ROUTINE[0] is next.
    private int sequenceIndex = 0;

    // Track which step is currently active, so the loop knows what it is waiting on.
    private Step activeStep = null;

    // For path steps, this stores which path is currently running.
    // For shoot steps, this will be -1.
    private int activePathIndex = -1;

    // ---------------------------------------------------------
    // PATHS
    // ---------------------------------------------------------
    /**
     * All Pedro paths are defined here in one place.
     *
     * This is the part to edit when you want to change the robot’s driving shape.
     *
     * Important idea:
     *   - The sequence/order of the autonomous is controlled by ROUTINE.
     *   - The shape of each drive leg is controlled here.
     */
    public static class Paths {
        public final PathChain[] paths;
        private static final int X_SHIFT = 4;

        public Paths(Follower follower) {
            paths = new PathChain[3];

            // Path 0
            // This is your first drive leg.
            // If the points feel too aggressive, simplify the curve before tuning anything else.
            paths[0] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.019, 8.449),
                            new Pose(89.957 - X_SHIFT, 67.040),
                            new Pose(139.506 - X_SHIFT, 10.629),
                            new Pose(143.725 - X_SHIFT, 61.415),
                            new Pose(143.895 - X_SHIFT, 36.199),
                            new Pose(136.219 - X_SHIFT, 9.494),
                            new Pose(72.539 - X_SHIFT, 35.404),
                            new Pose(129.456 - X_SHIFT, 6),
                            new Pose(90 - X_SHIFT, 10.449)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Path 1
            // This is your second drive leg.
            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(90.000 - X_SHIFT, 8.449),
                            new Pose(74.175 - X_SHIFT, 83.789),
                            new Pose(73.819 - X_SHIFT, 77.214),
                            new Pose(135.934 - X_SHIFT, 29.180),
                            new Pose(142.058 - X_SHIFT, 65.776),
                            new Pose(142.328 - X_SHIFT, 73.554),
                            new Pose(141.956 - X_SHIFT, 54.422),
                            new Pose(88.713 - X_SHIFT, 55.947),
                            new Pose(82.142 - X_SHIFT, 91.552)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Path 2
            // This is your final drive leg / parking move.
            // If you want the simpler five-step routine,
            // remove this path from ROUTINE, or leave it here as a parking move.
            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.142, 91.552),
                            new Pose(108, 70)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    // ---------------------------------------------------------
    // HARDWARE / SUBSYSTEMS
    // ---------------------------------------------------------
    private Follower follower;
    private Paths paths;

    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lift;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    private Turret turret;

    private DcMotor intake;
    private CRServo walleft;
    private CRServo wallright;

    // ---------------------------------------------------------
    // TIMERS
    // ---------------------------------------------------------
    private final Timer pathTimer = new Timer();
    private final Timer shootTimer = new Timer();
    private final Timer opmodeTimer = new Timer();

    // ---------------------------------------------------------
    // UTILITY HELPERS
    // ---------------------------------------------------------
    private static boolean isInArray(int[] arr, int v) {
        if (arr == null) return false;
        for (int x : arr) if (x == v) return true;
        return false;
    }

    /**
     * Maps AprilTag IDs to your shot pattern manager.
     *
     * This lets the robot choose the order of colors inside ShooterExecutionClass.
     * The drive order is controlled separately by ROUTINE.
     */
    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21:
                return ShotPatternManager.ShotPattern.GPP;
            case 22:
                return ShotPatternManager.ShotPattern.PGP;
            case 23:
                return ShotPatternManager.ShotPattern.PPG;
            default:
                return null;
        }
    }

    /**
     * Turns everything related to collection / feeding off.
     *
     * This is a good “safe default” any time you are not intentionally intaking.
     */
    private void setDriveFeedersOff() {
        intake.setPower(0);
        walleft.setPower(0);
        wallright.setPower(0);
    }

    /**
     * Runs intake + wall wheels together.
     *
     * Use this while driving through paths where you are collecting balls.
     */
    private void setIntakeAndWallsOn() {
        intake.setPower(INTAKE_POWER);
        walleft.setPower(WALL_WHEEL_POWER);
        wallright.setPower(WALL_WHEEL_POWER);
    }

    /**
     * Runs wall wheels but leaves intake off.
     *
     * This is useful if you want the feeder walls to keep moving balls,
     * but you do not want the intake spinning.
     */
    private void setWallsOnIntakeOff() {
        intake.setPower(0);
        walleft.setPower(WALL_WHEEL_POWER);
        wallright.setPower(WALL_WHEEL_POWER);
    }

    // ---------------------------------------------------------
    // STEP CONTROL
    // ---------------------------------------------------------
    /**
     * Starts the next step in ROUTINE.
     *
     * This is the heart of the autonomous.
     * Every time a step finishes, this method moves to the next one.
     *
     * There are only two kinds of steps:
     *   - SHOOT
     *   - PATH
     *
     * That makes the code easy to reason about and easy to edit later.
     */
    private void startNextStep() {
        if (sequenceIndex >= ROUTINE.length) {
            activeStep = null;
            autoState = AutoState.DONE;
            return;
        }

        activeStep = ROUTINE[sequenceIndex++];

        if (activeStep.type == StepType.SHOOT) {
            startShootStep();
        } else {
            startPathStep(activeStep.pathIndex);
        }
    }

    /**
     * Starts a shooting step.
     *
     * This does NOT drive the robot.
     * It only starts the shooter state machine and then waits for it to finish.
     */
    private void startShootStep() {
        // Set the hood target before the shooter cycle begins.
        // This keeps the shot step self-contained and easy to understand.
        hood.setTarget(75);

        // Make sure feeders are in a known state at the start of every shot.
        setWallsOnIntakeOff();

        // Reset the timeout timer for this shot step.
        shootTimer.resetTimer();

        // Start the shooter cycle.
        // ShooterExecutionClass owns the actual shot sequence.
        autoShoot.startCycle();

        // While the shooter is doing its work, this OpMode sits in WAIT_SHOOT.
        autoState = AutoState.WAIT_SHOOT;
    }

    /**
     * Starts a drive step.
     *
     * The path index is taken from the ROUTINE entry.
     * Pedro is then responsible for following that path.
     */
    private void startPathStep(int pathIdx) {
        activePathIndex = pathIdx;

        if (pathIdx < 0 || pathIdx >= paths.paths.length) {
            autoState = AutoState.DONE;
            activeStep = null;
            return;
        }

        // Start the path once.
        follower.followPath(paths.paths[pathIdx], 1, true);

        // Start the settle timer now, because this is when the path began.
        pathTimer.resetTimer();

        // While the path runs, we remain in WAIT_PATH.
        autoState = AutoState.WAIT_PATH;
    }

    /**
     * Ends whatever step is currently active and moves on to the next one.
     */
    private void completeActiveStep() {
        lift.setDown();
        setDriveFeedersOff();
        activePathIndex = -1;
        activeStep = null;
        autoState = AutoState.START_STEP;
    }

    /**
     * Safety fallback if the shooter gets stuck.
     */
    private void abortShootStep() {
        autoShoot.stopForcedCycle();
        lift.setDown();
        setDriveFeedersOff();
        activePathIndex = -1;
        activeStep = null;
        autoState = AutoState.START_STEP;
    }

    // ---------------------------------------------------------
    // LIFECYCLE
    // ---------------------------------------------------------
    @Override
    public void init() {
        Parameters.autonomous = Parameters.AUTONOMOUS.YES;

        opmodeTimer.resetTimer();

        // Create the Pedro follower.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(82.019, 8.449, Math.toRadians(0)));

        // Build the path list once at init.
        paths = new Paths(follower);

        // Hardware bindings.
        intake = hardwareMap.get(DcMotor.class, "intake");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);

        // Subsystems.
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lift = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lift);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);

        // If the shooter is not actively driving a shot, keep the feeder motors off.
        setDriveFeedersOff();
    }

    @Override
    public void init_loop() {
        // This runs before the match starts.
        // It is a good place for sensor observation and setup.
        turret.limelight.pipelineSwitch(2);

        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }

        // Keep these subsystem updates alive in init loop so telemetry is stable.
        hood.update();
        lift.update();
        spin.update();
        spin.sampleSensorsNow();

        telemetry.addData("sequenceIndex", sequenceIndex);
        telemetry.addData("autoState", autoState);
        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("tagId", tagId);

        Spindexer.TelemetryPacket sp = spin.getTelemetry();
        telemetry.addLine("=== SILOS ===");
        for (int i = 0; i < sp.siloColors.length; i++) {
            telemetry.addData("Silo " + (i + 1), sp.siloColors[i]);
        }

        telemetry.update();
    }

    @Override
    public void start() {
        // Start shooter in your normal ready state.
        shooter.update(false, false, true);
        opmodeTimer.resetTimer();

        // Set the Limelight pipeline for the correct alliance.
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            turret.limelight.pipelineSwitch(3);
        } else {
            turret.limelight.pipelineSwitch(4);
        }

        // Reset the routine.
        sequenceIndex = 0;
        activeStep = null;
        activePathIndex = -1;
        autoState = AutoState.START_STEP;

        // Make sure shooter state is clean.
        autoShoot.stopForcedCycle();
        lift.setDown();
        setDriveFeedersOff();

        // Kick off the first step.
        // Because ROUTINE starts with SHOOT, the robot shoots before driving.
        startNextStep();
    }

    @Override
    public void loop() {
        // -----------------------------------------------------
        // SYSTEMS THAT SHOULD ALWAYS BE ALIVE
        // -----------------------------------------------------
        // These run every loop, regardless of which step is active.
        shooter.update(false, false, true);
        shooter.updateTelemetry();
        hood.update();
        lift.update();
        spin.sampleSensorsNow();
        spin.update();

        // If the turret is supposed to track automatically, keep it in AUTO.
        turret.autoMode();
        turret.mode = Turret.turretMode.AUTO;

        // The shooter sequence must be updated continuously while it is active.
        autoShoot.update();

        // The Pedro follower must also be updated continuously while a path is active.
        follower.update();

        if (autoState == AutoState.DONE) {
            follower.breakFollowing();
            setDriveFeedersOff();
            lift.setDown();
        } else {
            switch (autoState) {
                case START_STEP:
                    // Start the next item in ROUTINE.
                    // This either becomes a shoot step or a path step.
                    startNextStep();
                    break;

                case WAIT_SHOOT:
                    // Keep the feeders moving while the shooter sequence is running.
                    // This keeps the shot flow consistent.
                    setWallsOnIntakeOff();

                    // Once the shooter is done, move to the next step.
                    if (!autoShoot.isBusy()) {
                        completeActiveStep();
                    }

                    // Safety timeout.
                    if (shootTimer.getElapsedTimeSeconds() > SHOOT_TIMEOUT_SECONDS) {
                        abortShootStep();
                    }
                    break;

                case WAIT_PATH:
                    // While the robot is still following a path, we can run intake logic.
                    // This is the part that makes the robot look alive while it drives.
                    if (follower.isBusy()) {
                        if (isInArray(INTAKE_PATHS, activePathIndex)) {
                            setIntakeAndWallsOn();
                        } else if (isInArray(WALL_ONLY_PATHS, activePathIndex)) {
                            intake.setPower(0);
                            walleft.setPower(WALL_WHEEL_POWER);
                            wallright.setPower(WALL_WHEEL_POWER);
                        } else {
                            setDriveFeedersOff();
                        }
                        break;
                    }

                    // If the path is finished, wait a tiny bit so the robot can settle.
                    if (pathTimer.getElapsedTimeSeconds() < PATH_SETTLE_SECONDS) {
                        break;
                    }

                    // Path is done and settled, so move to the next routine step.
                    completeActiveStep();
                    break;

                case DONE:
                    // Already handled above.
                    break;
            }
        }

        // -----------------------------------------------------
        // TELEMETRY
        // -----------------------------------------------------
        telemetry.addData("AutoState", autoState);
        telemetry.addData("sequenceIndex", sequenceIndex);
        telemetry.addData("activePathIndex", activePathIndex);
        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("autoShootBusy", autoShoot.isBusy());
        telemetry.addData("shotsFired", autoShoot.shotsFired);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Mode", s.mode);
        telemetry.addData("Target RPM", "%.2f", s.targetRPM);
        telemetry.addData("Current RPM", "%.1f", s.currentRPM);
        telemetry.addData("Error RPM", "%.1f", s.errorRPM);
        telemetry.addData("Motor Power", "%.2f", s.motorPower);

        Spindexer.TelemetryPacket sp = spin.getTelemetry();
        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("Mode", sp.mode);
        telemetry.addData("Angle", "%.1f°", sp.currentAngle);
        telemetry.addData("Target", "%.1f°", sp.targetAngle);
        telemetry.addData("Error", "%.1f°", sp.angleError);
        telemetry.addData("Power", "%.2f", sp.appliedPower);

        Hood.TelemetryPacket h = hood.getTelemetry();
        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Mode", h.mode);
        telemetry.addData("Angle", "%.1f°", h.currentAngle);
        telemetry.addData("Target", "%.1f°", h.targetAngle);
        telemetry.addData("Error", "%.1f°", h.angleError);

        telemetry.addLine("=== SILOS ===");
        for (int i = 0; i < sp.siloColors.length; i++) {
            telemetry.addData("Silo " + (i + 1), sp.siloColors[i]);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Make sure everything ends in a safe state.
        follower.breakFollowing();
        autoShoot.stopForcedCycle();
        setDriveFeedersOff();
        lift.setDown();
    }
}
