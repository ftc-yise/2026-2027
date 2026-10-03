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
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name = "[RED] Close Shoot Auto", group = "Auto")
public class RedCloseShootAutoPedroExample extends OpMode {
    static int X_SHIFT = 4;

    // --- Paths ---
    public static class Paths {
        public PathChain[] paths;
        public Paths(Follower follower) {
            paths = new PathChain[4];

            paths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(111.000, 136.000),
                            new Pose(88.000 - X_SHIFT, 92.000)
                    )).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(50))
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88.000 - X_SHIFT, 92.000),
                            new Pose(135.90691927512358 - X_SHIFT, 69.50411861614495),
                            new Pose(135.991 - X_SHIFT, 84.687),
                            new Pose(135.670 - X_SHIFT, 85.552),
                            new Pose(128.078 - X_SHIFT, 64.540),
                            new Pose(118.199 - X_SHIFT, 100.191),
                            new Pose(83.031 - X_SHIFT, 68.323),
                            new Pose(82.000 - X_SHIFT, 84.000)
                    )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            paths[2] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(82.000 - X_SHIFT, 84.000),
                            new Pose(111.694 - X_SHIFT, 30.730),
                            new Pose(143.143 - X_SHIFT, 73.264),
                            new Pose(143.092 - X_SHIFT, 66.909),
                            new Pose(142.278 - X_SHIFT, 74.143),
                            new Pose(84.043 - X_SHIFT, 95.196)
                    )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            paths[3] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(84.043 - X_SHIFT, 95.196),
                            new Pose(124.000 - X_SHIFT, 69.000)
                    )).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
        }
    }

    // --- FSM ---
    private enum AutoState { FOLLOW_PATH, REQUEST_SHOT, WAIT_FOR_SHOT, DONE }
    private AutoState autoState = AutoState.FOLLOW_PATH;

    // --- config & thresholds ---
    private final int[] INTAKE_PATHS = {2, 3};
    private final int[] SHOOT_PATHS = {1, 2, 3}; // which paths should trigger shooting
    private final double PATH_SETTLE_SECONDS = 1.0;
    private final double SHOOT_TIMEOUT_SECONDS = 15.0;
    private final int ZONE_BLUE_THRESHOLD = 1400;
    private final double SENSOR_HOLD_SECONDS = 0.18;
    private final double ZONE_SHOOT_MAX_DURATION = 20.0;

    // --- subsystems ---
    private Follower follower;
    private Paths paths;
    private Timer pathTimer, opmodeTimer;
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    private Turret turret;

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    // floor-facing color sensors
    private ColorSensor blc, brc, flc, frc;

    // local timers/counters
    private ElapsedTime shotRequestTimer = new ElapsedTime();

    // path state bookkeeping
    private int pathIndex = 0;
    private boolean followerStarted = false;
    private boolean lastFollowerBusy = true;
    private boolean[] shotDoneForPath;

    // zone-shooting & flags
    private double lastZoneTriggerStart = 0;
    private boolean zoneShootingActive = false;
    private double zoneShootingStartTime = 0.0;
    private boolean skipZoneShootingOnce = false;

    // pre-start shot flag (if you choose to pre-fire)
    private boolean startShotTriggered = false;

    // fallback: when true use forced cycle instead of pattern cycle
    private boolean shootForceFallback = false;

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

    // ---------- lifecycle ----------
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111, 136, Math.toRadians(180)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);

        blc = hardwareMap.get(ColorSensor.class, "BLC");
        brc = hardwareMap.get(ColorSensor.class, "BRC");
        flc = hardwareMap.get(ColorSensor.class, "FLC");
        frc = hardwareMap.get(ColorSensor.class, "FRC");

        drive = new DriveClass(hardwareMap);
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lifter = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);

        shotDoneForPath = new boolean[paths.paths.length];
        for (int i = 0; i < shotDoneForPath.length; i++) shotDoneForPath[i] = false;
    }

    @Override
    public void init_loop() {
        // load AprilTag pattern into patternMgr (if seen)
        turret.limelight.pipelineSwitch(2);
        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }

        // keep spindexer sensors fresh in init
        spin.update();
        spin.sampleSensorsNow();
        hood.update();

        telemetry.addData("pre_tag", tagId);
        Spindexer.TelemetryPacket sp = spin.getTelemetry();
        telemetry.addData("silos", sp.siloColors[0] + "," + sp.siloColors[1] + "," + sp.siloColors[2]);
        telemetry.update();
    }

    @Override
    public void start() {
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            turret.limelight.pipelineSwitch(3);
        } else {
            turret.limelight.pipelineSwitch(4);
        }
        opmodeTimer.resetTimer();
        followerStarted = false;
        lastFollowerBusy = true;
        autoState = AutoState.FOLLOW_PATH;

        // (optional) pre-fire logic — if you want to pre-fire at start, enable this:
        // startShotTriggered = true; autoState = AutoState.REQUEST_SHOT; skipZoneShootingOnce = true;
        // we leave it off by default, but keep flag handling in FSM
    }

    @Override
    public void loop() {
        // subsystem updates
        follower.update();
        spin.sampleSensorsNow();
        spin.update();
        hood.update();
        autoShoot.update();
        shooter.update(false, true, false); // keep shooter ready (tune as needed)
        turret.autoMode();

        // only do heavy FSM/sensor checks at a stable cadence (but keep updates above)
        checkFloorSensorsForZoneAndMaybeStartShooting();
        autoStateUpdate();

        // Intake / wall behaviour (safe defaults)
        if (!zoneShootingActive && autoState == AutoState.FOLLOW_PATH) {
            if (isInArray(INTAKE_PATHS, pathIndex)) {
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.startManualCycle();
            } else {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
            }
        }

        // zone-shooting continuation (if active)
        if (zoneShootingActive) {
            walleft.setPower(0.51);
            wallright.setPower(0.51);
            hood.update();
            autoShoot.update();

            if (autoShoot.shotsFired >= 3) {
                zoneShootingActive = false;
                stopZoneShootingCleanup();
                if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) shotDoneForPath[pathIndex] = true;
                pathIndex++;
                followerStarted = false;
            } else if ((opmodeTimer.getElapsedTimeSeconds() - zoneShootingStartTime) > ZONE_SHOOT_MAX_DURATION) {
                // timeout
                zoneShootingActive = false;
                autoShoot.stopForcedCycle();
                stopZoneShootingCleanup();
                if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) shotDoneForPath[pathIndex] = true;
                pathIndex++;
                followerStarted = false;
            }
        }

        // telemetry
        telemetry.addData("state", autoState);
        telemetry.addData("pathIndex", pathIndex);
        telemetry.addData("followerBusy", follower.isBusy());
        telemetry.addData("zone", zoneShootingActive);
        telemetry.addData("shotsFired", autoShoot.shotsFired);
        Spindexer.TelemetryPacket sp = spin.getTelemetry();
        telemetry.addData("silos", sp.siloColors[0] + "," + sp.siloColors[1] + "," + sp.siloColors[2]);
        telemetry.update();
    }

    private void stopZoneShootingCleanup() {
        walleft.setPower(0);
        wallright.setPower(0);
        lifter.setDown();
        spin.enableSensorUpdates();
    }

    // ------------------ FSM ------------------
    private void autoStateUpdate() {
        if (zoneShootingActive) return; // if zone-shooting active, FSM is paused

        switch (autoState) {
            case FOLLOW_PATH:
                if (!followerStarted) {
                    if (pathIndex < paths.paths.length) {
                        follower.followPath(paths.paths[pathIndex], 1.0, true);
                        followerStarted = true;
                        lastFollowerBusy = true;
                        // clear skip flag as soon as first path actually starts
                        skipZoneShootingOnce = false;
                    } else {
                        autoState = AutoState.DONE;
                        return;
                    }
                }

                // detect busy -> not-busy edge
                if (lastFollowerBusy && !follower.isBusy()) {
                    pathTimer.resetTimer();
                    lastFollowerBusy = false;
                }

                if (follower.isBusy()) {
                    lastFollowerBusy = true;
                    return;
                }

                // wait settle
                if (pathTimer.getElapsedTimeSeconds() < PATH_SETTLE_SECONDS) {
                    // keep sampling sensors while settling
                    spin.sampleSensorsNow();
                    return;
                }

                // if this path should shoot and hasn't yet been shot
                if (isInArray(SHOOT_PATHS, pathIndex) &&
                        pathIndex >= 0 && pathIndex < shotDoneForPath.length &&
                        !shotDoneForPath[pathIndex]) {

                    // decide pattern vs forced fallback

                    // prefer existing queued pattern if it can be satisfied by current silos
                    if (patternMgr != null && patternMgr.hasShots() && canSatisfyPattern(patternMgr, spin)) {
                        shootForceFallback = false;
                        // patternMgr already contains the sequence we want
                    } else {
                        // try to autodetect exact 3-ball pattern from silos
                        ShotPatternManager.ShotPattern detected = detectPatternFromSilos();
                        if (detected != null) {
                            patternMgr.clear();
                            patternMgr.addPattern(detected.sequence);
                            shootForceFallback = false;
                        } else {
                            // fallback: not enough info or non-exact order - prefer shooting whatever is available
                            shootForceFallback = true;
                        }
                    }

                    autoState = AutoState.REQUEST_SHOT;
                    return;
                }

                // not a shooting path -> advance
                pathIndex++;
                followerStarted = false;
                if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                break;

            case REQUEST_SHOT:
                // start pattern-aware or forced cycle
                if (!shootForceFallback && patternMgr.hasShots()) {
                    autoShoot.startCycle();
                } else {
                    autoShoot.startForcedCycle();
                }

                // stop follower while pattern-based shooting occurs (but keep follower free during zone-shoot)
                follower.breakFollowing();
                followerStarted = false;

                shotRequestTimer.reset();
                autoState = AutoState.WAIT_FOR_SHOT;
                break;

            case WAIT_FOR_SHOT:
                // maintain shooter/hood
                hood.setTarget(40);
                walleft.setPower(0.51);
                wallright.setPower(0.51);

                // finished shooting
                if (autoShoot.shotsFired >= 3) {
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    // If this was a pre-start shot, just return to follow path without marking path as shot
                    if (startShotTriggered) {
                        startShotTriggered = false;
                        autoState = AutoState.FOLLOW_PATH;
                        followerStarted = false;
                        pathTimer.resetTimer();
                        return;
                    }

                    // mark path shot and advance
                    if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) shotDoneForPath[pathIndex] = true;
                    pathIndex++;
                    followerStarted = false;
                    pathTimer.resetTimer();

                    if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                    else autoState = AutoState.FOLLOW_PATH;
                    return;
                }

                // safety timeout: if lifter stuck down for too long, abort and advance
                if (shotRequestTimer.seconds() > SHOOT_TIMEOUT_SECONDS && lifter.isDown()) {
                    autoShoot.stopForcedCycle();
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    if (pathIndex >= 0 && pathIndex < shotDoneForPath.length) shotDoneForPath[pathIndex] = true;
                    pathIndex++;
                    followerStarted = false;
                    if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                    else autoState = AutoState.FOLLOW_PATH;
                    return;
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

    // ---------------- zone detection ----------------
    private void checkFloorSensorsForZoneAndMaybeStartShooting() {
        double rx = follower.getPose().getX();
        double ry = follower.getPose().getY();
        // coarse inside - keep the original two triangles if desired; here we simply use sensor majority
        int count = 0;
        if (blc.blue() > ZONE_BLUE_THRESHOLD) count++;
        if (brc.blue() > ZONE_BLUE_THRESHOLD) count++;
        if (flc.blue() > ZONE_BLUE_THRESHOLD) count++;
        if (frc.blue() > ZONE_BLUE_THRESHOLD) count++;

        // skip zone shooting right after pre-start shot until first path starts
        if (skipZoneShootingOnce && pathIndex == 0 && !followerStarted) return;

        if (count >= 2) {
            // quick entry
            startZoneShootingIfNeeded();
            return;
        }

        // otherwise require hold
        boolean anyHigh = (blc.blue() > ZONE_BLUE_THRESHOLD) || (brc.blue() > ZONE_BLUE_THRESHOLD)
                || (flc.blue() > ZONE_BLUE_THRESHOLD) || (frc.blue() > ZONE_BLUE_THRESHOLD);
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
    }

    private void startZoneShootingIfNeeded() {
        if (zoneShootingActive) return;
        if (autoShoot.isBusy()) return;
        // don't zone-shoot if we asked to skip once (pre-start)
        if (skipZoneShootingOnce && pathIndex == 0 && !followerStarted) return;

        zoneShootingActive = true;
        zoneShootingStartTime = opmodeTimer.getElapsedTimeSeconds();
        spin.disableSensorUpdates();
        autoShoot.startForcedCycle();
    }

    // ---------- pattern detection helpers ----------
    // Return exact 3-slot pattern (GPP, PGP, PPG) or null if not exact
    private ShotPatternManager.ShotPattern detectPatternFromSilos() {
        Spindexer.TelemetryPacket t = spin.getTelemetry();
        Spindexer.BallColor[] s = t.siloColors;
        if (s == null || s.length < 3) return null;

        if (s[0] == Spindexer.BallColor.GREEN &&
                s[1] == Spindexer.BallColor.PURPLE &&
                s[2] == Spindexer.BallColor.PURPLE) return ShotPatternManager.ShotPattern.GPP;

        if (s[0] == Spindexer.BallColor.PURPLE &&
                s[1] == Spindexer.BallColor.GREEN &&
                s[2] == Spindexer.BallColor.PURPLE) return ShotPatternManager.ShotPattern.PGP;

        if (s[0] == Spindexer.BallColor.PURPLE &&
                s[1] == Spindexer.BallColor.PURPLE &&
                s[2] == Spindexer.BallColor.GREEN) return ShotPatternManager.ShotPattern.PPG;

        return null;
    }

    // Teleop's canSatisfyPattern copied here for reuse
    private boolean canSatisfyPattern(ShotPatternManager patternMgr, Spindexer spin) {
        if (patternMgr == null || !patternMgr.hasShots()) return false;

        Spindexer.BallColor[] queued = patternMgr.snapshot();
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
            if (found == -1) return false;
            used[found] = true;
        }
        return true;
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.setPower(0);
        walleft.setPower(0);
        wallright.setPower(0);
        autoShoot.stopForcedCycle();
        lifter.setDown();
    }
}
