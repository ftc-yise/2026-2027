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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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

@Autonomous(name = "[BLUE] Far Shoot Auto", group = "Auto")
public class BlueFarShootAuto extends OpMode {

    // --- Paths & pathing ---
    public static class Paths {
        public PathChain[] paths;
        int X_SHIFT = 4;

        public Paths(Follower follower) {
            paths = new PathChain[3];

            paths[0] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.007, 7.288),
                            new Pose(54.043 + X_SHIFT, 67.040),
                            new Pose(4.494 + X_SHIFT, 10.629),
                            new Pose(0.275 + X_SHIFT, 61.415),
                            new Pose(1.766 + X_SHIFT, 24.812),
                            new Pose(7.781 + X_SHIFT, 7.494),
                            new Pose(54.000 + X_SHIFT, 7.532)
                    ))
                    // original was (0,0) → flipped becomes (180,180)
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(54.000 + X_SHIFT, 7.532),
                            new Pose(69.825 + X_SHIFT, 83.789),
                            new Pose(70.181 + X_SHIFT, 77.214),
                            new Pose(8.066 + X_SHIFT, 29.180),
                            new Pose(1.942 + X_SHIFT, 65.776),
                            new Pose(1.672 + X_SHIFT, 73.554),
                            new Pose(2.044 + X_SHIFT, 54.422),
                            new Pose(55.287 + X_SHIFT, 55.947),
                            new Pose(61.858 + X_SHIFT, 91.552)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.858, 91.552),
                            new Pose(20.000, 70.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    // --- Enums: Auto state machine ---
    private enum AutoState {
        FOLLOW_PATH,
        REQUEST_SHOT,
        WAIT_FOR_SHOT,
        DONE
    }

    private AutoState autoState = AutoState.FOLLOW_PATH;

    // --- Configuration ---
    private double lastZoneTriggerStart = 0.0;
    private final double SENSOR_HOLD_SECONDS = 0.18; // 180 ms

    private final int[] SHOOT_PATHS = {0, 1};      // firing-trigger paths
    private final int[] INTAKE_PATHS = {0, 1};
    private final int[] WALL_ONLY_PATHS = {2};
    private final boolean fireAtStart = true;
    private boolean startFinished = false;
    private final double PATH_SETTLE_SECONDS = 1; // short settle after path arrival (mirrored from RED)
    private final double SHOOT_TIMEOUT_SECONDS = 15; // mirrored from RED
    private final int ZONE_BLUE_THRESHOLD = 1000; // mirrored from RED

    // --- Subsystems & hardware ---
    private Follower follower;
    private Paths paths;
    private boolean followerStarted = false;
    private boolean lastFollowerBusy = true;
    private Timer pathTimer, opmodeTimer, mmmmmTIME;
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    private Turret turret;
    private Ledclass led1;
    private Parameters parem = new Parameters();

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    // floor-facing color sensors at corners
    private ColorSensor BackRightCorner = null; // "BRC"
    private ColorSensor BackLeftCorner = null;  // "BLC"
    private ColorSensor FrontLeftCorner = null; // "FLC"
    private ColorSensor FrontRightCorner = null;// "FRC"

    // Local timers
    private ElapsedTime shotRequestTimer = new ElapsedTime(); // used for shot timeout / timing
    private ElapsedTime waitTimer = new ElapsedTime(); // general purpose

    // path index
    private int pathIndex = 0;
    private boolean[] shotDoneForPath;
    private boolean startShotTriggered = false;

    // Zone-shooting flags & latches (one per sensor)
    private boolean blcLatched = false;
    private boolean brcLatched = false;
    private boolean flcLatched = false;
    private boolean frcLatched = false;

    private boolean zoneShootingActive = false;
    private double zoneShootingStartTime = 0.0;
    private final double ZONE_SHOOT_MAX_DURATION = 8; // seconds safety for zone shooting

    // --- NEW FIX FIELDS ---
    // When true we will prevent zone-shoot from firing immediately after a pre-start shot.
    // This gets set when we perform a pre-start shot and is cleared when we actually start following the first path.
    private boolean skipZoneShootingOnce = false;

    // helper
    private boolean isInArray(int[] arr, int v) {
        if (arr == null) return false;
        for (int x : arr) if (x == v) return true;
        return false;
    }

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    // Triangles for shooting zones
    private boolean pointInTriangle(double px, double py, double ax, double ay, double bx, double by, double cx, double cy) {
        double v0x = cx - ax;
        double v0y = cy - ay;
        double v1x = bx - ax;
        double v1y = by - ay;
        double v2x = px - ax;
        double v2y = py - ay;

        double dot00 = v0x * v0x + v0y * v0y;
        double dot01 = v0x * v1x + v0y * v1y;
        double dot02 = v0x * v2x + v0y * v2y;
        double dot11 = v1x * v1x + v1y * v1y;
        double dot12 = v1x * v2x + v1y * v2y;

        double denom = (dot00 * dot11 - dot01 * dot01);
        if (Math.abs(denom) < 1e-9) return false;

        double invDenom = 1.0 / denom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        return (u >= 0) && (v >= 0) && (u + v < 1);
    }

    private boolean insideShootingZone(double x, double y) {
        if (pointInTriangle(x, y, 96+8, 0, 72+8, 24-8, 48-8, 0)) return true;
        if (pointInTriangle(x, y, 0, 144, 72+8, 72+8, 144, 144)) return true;
        return false;
    }

    // ----------------- Lifecycle -----------------
    @Override
    public void init() {
        Parameters.autonomous = Parameters.AUTONOMOUS.YES;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        mmmmmTIME = new Timer();
        mmmmmTIME.resetTimer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60.007, 7.288, Math.toRadians(180)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        led1 = new Ledclass(hardwareMap, "led1");
        wallright.setDirection(CRServo.Direction.REVERSE);

        BackLeftCorner = hardwareMap.get(ColorSensor.class, "BLC");
        BackRightCorner = hardwareMap.get(ColorSensor.class, "BRC");
        FrontLeftCorner = hardwareMap.get(ColorSensor.class, "FLC");
        FrontRightCorner = hardwareMap.get(ColorSensor.class, "FRC");

        drive = new DriveClass(hardwareMap);
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lifter = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);
        parem.autonomous = Parameters.AUTONOMOUS.YES;

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);

        shotDoneForPath = new boolean[paths.paths.length];
        for (int i = 0; i < shotDoneForPath.length; i++) shotDoneForPath[i] = false;

    }

    @Override
    public void init_loop() {
        turret.limelight.pipelineSwitch(2);
        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }

        hood.update();
        Hood.TelemetryPacket H = hood.getTelemetry();
        lifter.TelemetryPacket l = lifter.getTelemetry();

        telemetry.addData("pre_pathIndex", pathIndex);
        telemetry.addData("pre_pathState", autoState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        Spindexer.TelemetryPacket spina = spin.getTelemetry();
        telemetry.addLine("=== SILOS ===");
        Spindexer.BallColor[] silos = spina.siloColors;
        for (int i = 0; i < silos.length; i++) {
            String label = "Silo " + (i+1);
            // Highlight the current silo
            telemetry.addData(label, silos[i]);
        }
        spin.sampleSensorsNow();
        spin.update();
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.update(false, false, true);
        opmodeTimer.resetTimer();

        if (pathIndex < 0) pathIndex = 0;

        followerStarted = false;
        lastFollowerBusy = true;
        autoState = AutoState.FOLLOW_PATH;

        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            turret.limelight.pipelineSwitch(3);
        } else {
            turret.limelight.pipelineSwitch(4);
        }

        if (fireAtStart) {
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            patternMgr.clear();
            ShotPatternManager.ShotPattern startPattern = patternFromTag(turret.getID());
            if (startPattern != null) patternMgr.addPattern(startPattern.sequence);
            autoState = AutoState.REQUEST_SHOT;

            // mark that this request was from the pre-start
            startShotTriggered = true;

            // IMPORTANT: set the skip flag so we don't zone-shoot before the first path begins
            skipZoneShootingOnce = false;
        }
        else {
            if (pathIndex < paths.paths.length) {
                follower.followPath(paths.paths[pathIndex]);
                followerStarted = true;
            }
        }

        shotRequestTimer.reset();
        waitTimer.reset();
    }

    @Override
    public void loop() {
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            turret.limelight.pipelineSwitch(3);
        } else {
            turret.limelight.pipelineSwitch(4);
        }

        shooter.update(false, false, true);
        autoShoot.update();     // ONLY once per loop
        hood.update();
        spin.sampleSensorsNow();
        spin.update();
        shooter.updateTelemetry();
        follower.update();
        turret.autoMode();
        turret.mode = Turret.turretMode.AUTO;

        if (mmmmmTIME.getElapsedTimeSeconds() > 1.5) {

            // check floor sensors and potentially trigger zone-shooting
            checkFloorSensorsForZoneAndMaybeStartShooting();

            // run single owner auto state machine
            autoStateUpdate();

            if (startFinished) {
                if (autoState == AutoState.FOLLOW_PATH) {
                    if (isInArray(INTAKE_PATHS, pathIndex)) {
                        intake.setPower(1);
                        walleft.setPower(1);
                        wallright.setPower(1);
                        spin.goToSilo2();
                        turret.stop();
                    } else if (isInArray(WALL_ONLY_PATHS, pathIndex)) {
                        intake.setPower(0);
                        walleft.setPower(0.51);
                        wallright.setPower(0.51);
                    } else {
                        intake.setPower(0);
                        walleft.setPower(0);
                        wallright.setPower(0);
                    }
                }

                if (zoneShootingActive) {
                    hood.update();
                    autoShoot.update(); // ensure firing continues
                    spin.update();
                    walleft.setPower(0.51);
                    wallright.setPower(0.51);

                    // If we've fired all 3 balls, stop zone-shooting and allow path advancement
                    if (autoShoot.shotsFired >= 3) {
                        // End zone shooting, cleanup
                        zoneShootingActive = false;
                        stopZoneShootingCleanup();

                        // mark the path as shot so FSM won't request it again
                        if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) {
                            shotDoneForPath[pathIndex] = true;
                        }

                        // *** NEW: Advance to the next path immediately so it is not re-run ***
                        pathIndex++;
                        followerStarted = false;
                        startFinished = true;

                        // prevent overflow
                        if (pathIndex >= paths.paths.length) {
                            autoState = AutoState.DONE;
                        }
                    } else if ((opmodeTimer.getElapsedTimeSeconds() - zoneShootingStartTime) > ZONE_SHOOT_MAX_DURATION) {
                        // safety timeout - abort zone shooting
                        zoneShootingActive = false;
                        if (skipZoneShootingOnce){

                            autoShoot.startCycle();
                        }else {
                            autoShoot.stopForcedCycle();
                        }
                        stopZoneShootingCleanup();
                        // advance so we don't repeat same path
                        if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) {
                            shotDoneForPath[pathIndex] = true;
                        }
                        pathIndex++;
                        followerStarted = false;
                        if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                    }
                }
            }

            // telemetry (condensed)
            Hood.TelemetryPacket H = hood.getTelemetry();
            lifter.TelemetryPacket l = lifter.getTelemetry();

            telemetry.addData("AutoState", autoState);
            telemetry.addData("pathIndex", pathIndex);
            telemetry.addData("followerBusy", follower.isBusy());
            telemetry.addData("zoneShooting", zoneShootingActive);
            telemetry.addData("skipZoneOnce", skipZoneShootingOnce);
            telemetry.addData("shotsFired", autoShoot.shotsFired);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addLine();

            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Target RPM", "%.2f", s.targetRPM);
            telemetry.addData("Current RPM", "%.1f", s.currentRPM);
            telemetry.addData("Error RPM", "%.1f", s.errorRPM);
            telemetry.addData("volt", s.motorPower);

            telemetry.addLine("=== SPINDEXER ===");
            Spindexer.TelemetryPacket spina = spin.getTelemetry();
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);

            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Mode", H.mode);
            telemetry.addData("Angle", "%.1f°", H.currentAngle);
            telemetry.addData("Target", "%.1f°", H.targetAngle);
            telemetry.addData("Error", "%.1f°", H.angleError);

            telemetry.addLine("=== SILOS ===");
            Spindexer.BallColor[] silos = spina.siloColors;
            for (int i = 0; i < silos.length; i++) {
                String label = "Silo " + (i+1);
                telemetry.addData(label, silos[i]);
            }

            telemetry.update();
        }
    }

    private void stopZoneShootingCleanup() {
        walleft.setPower(0);
        wallright.setPower(0);
        lifter.setDown();
        spin.enableSensorUpdates();
    }

    @Override
    public void stop() {
    }

    // ----------------- Auto state machine -----------------
    private void autoStateUpdate() {

        // If zone-shooting active, prevent normal path advancement.
        if (zoneShootingActive) {
            return;
        }

        switch (autoState) {

            case FOLLOW_PATH:
                startFinished = true;
                if (!followerStarted) {
                    if (pathIndex < paths.paths.length) {
                        follower.followPath(paths.paths[pathIndex], 1, true);
                        followerStarted = true;

                        // We only wanted to suppress zone-shooting until the first path actually started.
                        // Clear the skip flag once the path is actually started so future zone shooting works.
                        skipZoneShootingOnce = false;

                        lastFollowerBusy = true;
                    } else {
                        autoState = AutoState.DONE;
                        return;
                    }
                }

                // Detect arrival edge: follower busy -> not busy
                if (lastFollowerBusy && !follower.isBusy()) {
                    pathTimer.resetTimer();
                    lastFollowerBusy = false;
                }

                if (follower.isBusy()) {
                    lastFollowerBusy = true;
                    return;
                }

                if (pathTimer.getElapsedTimeSeconds() < PATH_SETTLE_SECONDS) {
                    return;
                }

                // If this path is a shooting path, request shooting (pattern-based)
                if (isInArray(SHOOT_PATHS, pathIndex) &&
                        pathIndex >= 0 && pathIndex < shotDoneForPath.length &&
                        !shotDoneForPath[pathIndex]) {
                    autoState = AutoState.REQUEST_SHOT;
                    return;
                }

                // Not a shooting path -> advance to next path
                pathIndex++;
                followerStarted = false;

                if (pathIndex >= paths.paths.length) {
                    autoState = AutoState.DONE;
                }
                break;

            case REQUEST_SHOT:
                ShotPatternManager.ShotPattern p = patternFromTag(turret.getID());
                if (p != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(p.sequence);
                }

                autoShoot.startForcedCycle();

                // If not zone-shooting, break follower to keep robot still while pattern-based shooting occurs
                if (!zoneShootingActive) {
                    follower.breakFollowing();
                    followerStarted = false;
                }

                shotRequestTimer.reset();
                autoState = AutoState.WAIT_FOR_SHOT;
                break;

            case WAIT_FOR_SHOT:
                hood.setTarget(75);

                walleft.setPower(0.51);
                wallright.setPower(0.51);

                if (autoShoot.shotsFired >= 3) {
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    if (startShotTriggered) {
                        // This was the pre-start shot. DO NOT mark shotDoneForPath[pathIndex],
                        // DO NOT advance pathIndex here. Instead return to FOLLOW_PATH so the
                        // follower will execute path 0 and we can shoot it at its end.
                        startShotTriggered = false;
                        autoState = AutoState.FOLLOW_PATH;
                        followerStarted = false;
                        pathTimer.resetTimer();
                        waitTimer.reset();
                        startFinished = true;

                        // NOTE: skipZoneShootingOnce was already set in start(); keep it true
                        // until the FOLLOW_PATH branch actually starts the path (we clear it there).
                        return;
                    } else {
                        if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) {
                            shotDoneForPath[pathIndex] = true;
                        }

                        pathIndex++;
                        followerStarted = false;
                        pathTimer.resetTimer();
                        waitTimer.reset();

                        if (pathIndex >= paths.paths.length) {
                            autoState = AutoState.DONE;
                        } else {
                            autoState = AutoState.FOLLOW_PATH;
                        }
                        startFinished = true;
                        return;
                    }
                }

                if (shotRequestTimer.seconds() > SHOOT_TIMEOUT_SECONDS && lifter.isDown()) {
                    autoShoot.stopForcedCycle();
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    pathIndex++;
                    followerStarted = false;
                    if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                    else autoState = AutoState.FOLLOW_PATH;
                    startFinished = true;
                }
                break;

            case DONE:
                follower.breakFollowing();
                walleft.setPower(0);
                wallright.setPower(0);
                lifter.setDown();
                break;
        }

    }

    // ----------------- Zone detection and zone-shooting -----------------
    private void checkFloorSensorsForZoneAndMaybeStartShooting() {
        int blc = BackLeftCorner.blue();
        int brc = BackRightCorner.blue();
        int flc = FrontLeftCorner.blue();
        int frc = FrontRightCorner.blue();

        double rx = follower.getPose().getX();
        double ry = follower.getPose().getY();
        boolean coarseInside = insideShootingZone(rx, ry);

        if (!coarseInside) {
            lastZoneTriggerStart = 0;
            blcLatched = brcLatched = flcLatched = frcLatched = false;
            return;
        }

        // If we were asked to skip zone-shooting once (pre-start shot), and we haven't started
        // the first path yet, don't trigger zone shooting here.
        if (skipZoneShootingOnce && pathIndex == 0 && !followerStarted) {
            return;
        }

        int countAbove = 0;
        if (blc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (brc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (flc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (frc > ZONE_BLUE_THRESHOLD) countAbove++;

        // immediate accept if two or more sensors are high (quick entry)
        if (countAbove >= 1) {
            startZoneShootingIfNeeded();
            return;
        }

        boolean anyHigh = (blc > ZONE_BLUE_THRESHOLD) || (brc > ZONE_BLUE_THRESHOLD)
                || (flc > ZONE_BLUE_THRESHOLD) || (frc > ZONE_BLUE_THRESHOLD);

        double now = opmodeTimer.getElapsedTimeSeconds();
        if (anyHigh) {
            if (lastZoneTriggerStart == 0) lastZoneTriggerStart = now;
            else if ((now - lastZoneTriggerStart) >= SENSOR_HOLD_SECONDS) {
                startZoneShootingIfNeeded();
                lastZoneTriggerStart = 0;
            }
        } else {
            lastZoneTriggerStart = 0;
        }

        blcLatched = blc > ZONE_BLUE_THRESHOLD;
        brcLatched = brc > ZONE_BLUE_THRESHOLD;
        flcLatched = flc > ZONE_BLUE_THRESHOLD;
        frcLatched = frc > ZONE_BLUE_THRESHOLD;
    }

    private void startZoneShootingIfNeeded() {
        if (zoneShootingActive) return;

        if (autoState != AutoState.FOLLOW_PATH) return;

        // double-check skip flag: don't zone-shoot if we were asked to skip once.
        if (skipZoneShootingOnce && pathIndex == 0 && !followerStarted) return;

        zoneShootingActive = true;
        zoneShootingStartTime = opmodeTimer.getElapsedTimeSeconds();
        spin.disableSensorUpdates();

        // Use forced cycling — it will continue cycling until we call stopForcedCycle()
        autoShoot.startForcedCycle();
    }

}