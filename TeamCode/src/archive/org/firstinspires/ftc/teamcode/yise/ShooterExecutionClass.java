package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * ShooterExecutionClass controls the full shooting cycle.
 *
 * It coordinates:
 * - moving the spindexer to the next usable silo
 * - spinning up the shooter
 * - raising/lowering the lifter to fire
 * - optional pattern-based shooting order
 * - forced shooting mode while a button is held
 *
 * This class is basically the "shot sequence brain" for the robot.
 */
public class ShooterExecutionClass {

    // =========================================================
    // STATE MACHINE
    // These states describe where the shooting cycle currently is.
    // =========================================================
    public enum State {
        JITTER,          // Small settling motion used when no shots are available
        IDLE,            // Not currently shooting
        MOVE_TO_SILO,    // Move spindexer to the next target silo
        SPIN_WAIT,       // Short pause before shooter spin-up
        SPIN_UP_SHOOTER, // Wait until shooter reaches usable speed
        FIRE_LIFT_UP,    // Raise lifter to push ball into shooter
        FIRE_LIFT_DOWN,  // Lower lifter back down after firing
        NEXT_SILO,       // Prepare to move to the next silo
        COMPLETE         // Clean up and return to idle
    }

    // Current state of the shooting state machine
    private State state = State.IDLE;

    // =========================================================
    // HARDWARE / SUBSYSTEM REFERENCES
    // These are the three major subsystems this class orchestrates.
    // =========================================================
    private final lifter lifter;
    private final Spindexer spindexer;
    private final ShooterClass shooter;

    // Timer used for state transitions and shot timing
    private final ElapsedTime timer = new ElapsedTime();

    // Maximum time allowed for the lifter to move before timing out
    private final double LIFTER_MOVE_TIMEOUT = 1.2; // seconds

    // Total shots fired in the current overall cycle
    public int shots = 0;

    // Tracks whether the D-pad is being used by another control path
    public boolean dpad = false;

    // Number of shots fired in the current cycle
    public int shotsFired = 0;

    // Total shots expected in the current cycle
    // In forced mode this becomes very large so the cycle continues until stopped.
    private int totalShots = 0;

    // Which silo is currently being targeted
    public int currentSiloIndex = -1;

    // Used for a small settling motion when the system has no shots to process
    public boolean jittered = false;

    // =========================================================
    // FORCED SHOOTING MODE
    // This bypasses normal "shot count" completion and keeps cycling
    // until the operator releases the button.
    // =========================================================
    public boolean forceShooting = false;

    // Basic lifter position constants
    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;

    // =========================================================
    // PATTERN SUPPORT
    // Allows the operator or vision system to request an ordered shot plan.
    // Example: shoot silos in a specific color sequence.
    // =========================================================
    private ShotPatternManager patternMgr = null;   // supplied by caller
    private boolean patternMode = false;            // true when following a pattern plan
    private final int[] firingPlan = new int[ShotPatternManager.MAX_SHOTS]; // silo indices
    private int firingCount = 0;                    // number of valid entries in firingPlan
    private int firingIndex = 0;                    // next plan entry to execute

    // =========================================================
    // TIMING + AVERAGE TRACKING
    // These values are used for telemetry and CSV logging.
    // They measure shot timing within a press cycle and across all cycles.
    // =========================================================

    // Start time of the current press/hold cycle
    private long pressStartTimeMs = 0L;

    // Shot times for the current press cycle
    private final double[] currentPressShotTimes = new double[] {
            Double.NaN, Double.NaN, Double.NaN
    };

    // How many shots have been recorded in the current press
    private int currentPressShotCount = 0;

    // True while a press cycle is being timed
    private boolean pressActive = false;

    // Last completed press timings (sticky values for telemetry/logging)
    private final double[] lastCompletedPressShotTimes = new double[] {
            0.0, 0.0, 0.0
    };
    private double lastCompletedPressAverage = 0.0;

    // Global running averages across all presses/shots
    private long totalPresses = 0L;
    private double sumShot1 = 0.0;
    private long countShot1 = 0L;
    private double sumShot2 = 0.0;
    private long countShot2 = 0L;
    private double sumShot3 = 0.0;
    private long countShot3 = 0L;
    private double sumAllShotTimes = 0.0;
    private long countAllShots = 0L;

    // Most recent phase timing values for logging
    private double lastMoveToSiloSec = 0.0;
    private double lastSpinWaitSec = 0.0;
    private double lastSpinUpSec = 0.0;
    private double lastFireLiftUpSec = 0.0;
    private double lastFireLiftDownSec = 0.0;

    // Last silo and angle that were targeted
    private int lastTargetSiloIndex = -1;
    private double lastTargetAngleDeg = 0.0;

    // -----------------------------------------------------------

    /**
     * Creates the execution controller and initializes the related subsystems.
     *
     * The constructor sets up the spindexer silos and lifter calibration so the
     * shooting sequence starts from a known baseline.
     */
    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap, lifter lift) {
        this.spindexer = spin;
        this.shooter = shooter;
        this.lifter = lift;

        // Initialize the spindexer’s silo detection / configuration
        spin.initSilos();

        // Keep default lifter configuration and calibration
        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(0.457, 0, 1.42, 1);

        // Initialize plan array to "empty"
        for (int i = 0; i < firingPlan.length; i++) firingPlan[i] = -1;
    }

    // =========================================================
    // PATTERN API
    // =========================================================

    /**
     * Supplies the pattern manager used to define ordered silo shots.
     */
    public void setPatternManager(ShotPatternManager mgr) {
        this.patternMgr = mgr;
    }

    // =========================================================
    // STARTING A NORMAL CYCLE
    // =========================================================

    /**
     * Starts a standard shooting cycle.
     *
     * Behavior:
     * 1. Begins timing for this trigger press
     * 2. Samples spindexer sensors
     * 3. Tries to build a pattern-based firing plan if one exists
     * 4. Falls back to the older "shoot the next available silos" logic
     */
    public void startCycle() {
        if (state != State.IDLE) return;

        // Start timing the cycle for shot timing statistics
        beginPressTiming();

        // Freeze sensor updates while the cycle is being planned
        spindexer.sampleSensorsNow();
        spindexer.disableSensorUpdates();

        // If a pattern exists, try to satisfy it in order
        if (patternMgr != null && patternMgr.hasShots()) {
            if (buildFiringPlanFromPattern()) {
                // Pattern-based firing plan was successfully built
                shotsFired = 0;
                totalShots = firingCount;
                firingIndex = 0;
                currentSiloIndex = firingPlan[0];

                // Move to the first planned silo
                goToSiloIndex(currentSiloIndex);
                timer.reset();
                state = State.MOVE_TO_SILO;
                timer.reset();
                return;
            } else {
                // Pattern could not be satisfied, so fall back to normal order
                patternMode = false;
            }
        }

        // ----------------------------------------------------
        // FALLBACK BEHAVIOR
        // Shoot the available silos in the simplest usable order.
        // ----------------------------------------------------
        totalShots = 0;
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        for (Spindexer.BallColor color : colors) {
            if (color != Spindexer.BallColor.NONE) totalShots++;
        }

        // If there are no balls available, do a small jitter once to settle mechanisms
        if (totalShots == 0) {
            if (!jittered) {
                jittered = true;
                timer.reset();
                state = State.JITTER;
            } else {
                spindexer.enableSensorUpdates();
                state = State.IDLE;
                jittered = true;
            }
            return;
        }

        // Start the normal cycle
        shotsFired = 0;
        timer.reset();
        state = State.MOVE_TO_SILO;
        moveToNextFullSilo();
        timer.reset();
    }

    // =========================================================
    // FORCED SHOOTING MODE
    // =========================================================

    /**
     * Starts forced shooting mode.
     *
     * Forced mode ignores the normal shot count limit and keeps cycling
     * until stopForcedCycle() is called.
     */
    public void startForcedCycle() {
        if (state != State.IDLE) {
            // Already running, just mark that forced mode is desired
            forceShooting = true;
            return;
        }

        forceShooting = true;

        // Start timing this button-hold shooting session
        beginPressTiming();

        // Disable live sensor updates while we actively cycle
        spindexer.disableSensorUpdates();

        // Forced mode keeps going until manually stopped
        shotsFired = 0;
        totalShots = Integer.MAX_VALUE;

        // Start from the first silo and move through the cycle
        currentSiloIndex = 0;
        spindexer.goToSilo1();
        timer.reset();
        state = State.MOVE_TO_SILO;
        timer.reset();
    }

    /**
     * Ends forced shooting mode.
     *
     * This re-enables normal sensor behavior and lets the state machine
     * cleanly return to idle.
     */
    public void stopForcedCycle() {
        // Save any in-progress shot timing before shutting down
        finalizePressIfActive();

        forceShooting = false;
        spindexer.enableSensorUpdates();

        // Mark the cycle complete so update() can return to idle
        state = State.COMPLETE;
    }

    // =========================================================
    // MAIN UPDATE LOOP
    // =========================================================

    /**
     * Runs one step of the state machine.
     *
     * This should be called repeatedly from the main OpMode loop.
     * It advances the shooter through the movement/spin/fire sequence.
     */
    public void update() {
        lifter.update();

        switch (state) {
            case JITTER: {
                // Do nothing except wait briefly.
                return;
            }

            case IDLE:
                // In idle, keep the shooter in a safe default configuration.
                if (!dpad) {
                    if (org.firstinspires.ftc.teamcode.yise.Parameters.autonomous ==
                            org.firstinspires.ftc.teamcode.yise.Parameters.AUTONOMOUS.YES) {
                        shooter.update(false, false, true);
                    } else {
                        shooter.update(false, false, false);
                    }
                }
                return;

            case MOVE_TO_SILO:
                // Wait until the spindexer is aligned enough to proceed
                double angleErr = Math.abs(spindexer.getTelemetry().angleError);
                if (timer.seconds() > 1.25) {
                    if (angleErr < 1.5) {
                        lastMoveToSiloSec = timer.seconds();
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    } else if (timer.seconds() > 5) {
                        // Safety watchdog: don't stay stuck forever
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        lastMoveToSiloSec = timer.seconds();
                        timer.reset();
                    }
                }
                break;

            case SPIN_WAIT:
                // Small pause before spinning the shooter
                if (timer.seconds() > .1) {
                    lastSpinWaitSec = timer.seconds();
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                // Wait until the shooter error indicates it is ready
                if (shooter.getTelemetry().errorRPM < 0) {
                    lastSpinUpSec = timer.seconds();
                    lifter.setUp();
                    timer.reset();
                    state = State.FIRE_LIFT_UP;
                }
                break;

            case FIRE_LIFT_UP:
                // Raise the lifter to push the ball into the shooter
                if (lifter.isUp() || timer.seconds() > LIFTER_MOVE_TIMEOUT) {
                    if (timer.seconds() > .25) {
                        lastFireLiftUpSec = timer.seconds();
                        lifter.setDown();
                        timer.reset();
                        state = State.FIRE_LIFT_DOWN;
                    }
                }
                break;

            case FIRE_LIFT_DOWN:
                // Wait until the lifter returns down, then record the shot and move on
                if (lifter.isDown() || timer.seconds() > (LIFTER_MOVE_TIMEOUT + 0.3)) {
                    shotsFired++;
                    shots++;

                    // Record timing for shot metrics
                    recordShotTimingForCurrentPress();

                    // Save the color that was in the fired silo before it is cleared
                    Spindexer.BallColor firedColor = Spindexer.BallColor.NONE;
                    if (currentSiloIndex >= 0 && currentSiloIndex < 3) {
                        firedColor = spindexer.getTelemetry().siloColors[currentSiloIndex];
                    }

                    // In normal mode, clear the silo that was just fired
                    if (!forceShooting && currentSiloIndex != -1) {
                        spindexer.clearSilo(currentSiloIndex);
                    }

                    // If a pattern is being used, consume the pattern head only if it matches
                    // the actual color that was fired.
                    if (patternMode && patternMgr != null && patternMgr.hasShots()) {
                        Spindexer.BallColor[] queued = patternMgr.snapshot();
                        Spindexer.BallColor head = Spindexer.BallColor.NONE;

                        for (int i = 0; i < queued.length; i++) {
                            if (queued[i] != Spindexer.BallColor.NONE) {
                                head = queued[i];
                                break;
                            }
                        }

                        if (head != Spindexer.BallColor.NONE && head == firedColor) {
                            patternMgr.getNext();
                        } else {
                            // Pattern mismatch: leave queue unchanged so the intended plan stays intact
                        }
                    }

                    if (!forceShooting) {
                        if (shotsFired >= totalShots && lifter.isDown()) {
                            // Cycle is done
                            finalizePressIfActive();
                            lastFireLiftDownSec = timer.seconds();

                            state = State.COMPLETE;
                        } else {
                            lastFireLiftDownSec = timer.seconds();
                            state = State.NEXT_SILO;
                        }
                    } else {
                        lastFireLiftDownSec = timer.seconds();

                        // Forced mode keeps cycling until externally stopped
                        state = State.NEXT_SILO;
                    }
                }
                break;

            case NEXT_SILO:
                // Move to the next target and re-enter the cycle
                moveToNextFullSilo();
                timer.reset();
                state = State.MOVE_TO_SILO;
                break;

            case COMPLETE:
                // Restore normal sensor updates and return to idle
                if (!forceShooting) spindexer.enableSensorUpdates();
                spindexer.sampleSensorsNow();

                patternMode = false;
                state = State.IDLE;

                // Safety cleanup in case the press was still active
                finalizePressIfActive();
                break;
        }
    }

    // =========================================================
    // SILO SELECTION
    // =========================================================

    /**
     * Advances the spindexer to the next target silo.
     *
     * Behavior depends on mode:
     * - forced mode: simply steps through silo indices in order
     * - pattern mode: follows the planned firing order
     * - normal mode: picks the next non-empty silo
     */
    private void moveToNextFullSilo() {
        if (forceShooting) {
            // Forced mode: cycle through the three silos in order
            currentSiloIndex = (currentSiloIndex + 1) % 3;
            goToSiloIndex(currentSiloIndex);
            return;
        }

        // Pattern mode: follow the computed firing plan
        if (patternMode && firingCount > 0) {
            firingIndex++;
            if (firingIndex < firingCount) {
                currentSiloIndex = firingPlan[firingIndex];
                goToSiloIndex(currentSiloIndex);
            } else {
                // Pattern finished
                currentSiloIndex = -1;
                state = State.COMPLETE;
            }
            return;
        }

        // Normal behavior: choose the next non-empty silo
        spindexer.sampleSensorsNow();
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        int nextIndex = (currentSiloIndex + 1) % 3;

        for (int i = 0; i < 3; i++) {
            int idx = (nextIndex + i) % 3;
            if (colors[idx] != Spindexer.BallColor.NONE) {
                currentSiloIndex = idx;
                goToSiloIndex(idx);
                return;
            }
        }

        // No usable silo found
        currentSiloIndex = -1;
        state = State.COMPLETE;
    }

    /**
     * Sends the spindexer to one of the three preset silos.
     */
    private void goToSiloIndex(int idx) {
        lastTargetSiloIndex = idx;

        switch (idx) {
            case 0: spindexer.goToSilo1(); break;
            case 1: spindexer.goToSilo2(); break;
            case 2: spindexer.goToSilo3(); break;
            default: break;
        }

        // Cache the target angle for logging
        lastTargetAngleDeg = spindexer.getTelemetry().targetAngle;
    }

    // =========================================================
    // PATTERN BUILDING
    // =========================================================

    /**
     * Builds a non-destructive firing plan from the queued pattern.
     *
     * The pattern is matched against the currently detected silo colors.
     * A silo can only be used once in the plan.
     *
     * Returns true if a usable plan was built.
     */
    private boolean buildFiringPlanFromPattern() {
        patternMode = false;
        firingCount = 0;

        // Snapshot queued colors without consuming them
        Spindexer.BallColor[] queued = patternMgr.snapshot();

        // Count the number of actual queued entries at the front
        int queuedCount = 0;
        for (int i = 0; i < queued.length; i++) {
            if (queued[i] == Spindexer.BallColor.NONE) break;
            queuedCount++;
        }
        if (queuedCount == 0) return false;

        // Current silo colors
        Spindexer.BallColor[] silos = spindexer.getTelemetry().siloColors;
        boolean[] used = new boolean[silos.length];

        ArrayList<Integer> plan = new ArrayList<>();

        // For each requested color, find a matching unused silo
        for (int q = 0; q < queuedCount; q++) {
            Spindexer.BallColor desired = queued[q];
            int found = -1;

            for (int s = 0; s < silos.length; s++) {
                if (!used[s] && silos[s] == desired) {
                    found = s;
                    break;
                }
            }

            if (found != -1) {
                plan.add(found);
                used[found] = true;
            } else {
                // If none of the needed color is available, use the partial plan if there is one
                if (plan.size() == 0) {
                    return false;
                } else {
                    break;
                }
            }
        }

        if (plan.isEmpty()) return false;

        // Copy plan into fixed array form
        firingCount = plan.size();
        for (int i = 0; i < firingCount; i++) {
            firingPlan[i] = plan.get(i);
        }

        // Fill the remaining entries with -1 so unused slots are obvious
        for (int i = firingCount; i < firingPlan.length; i++) firingPlan[i] = -1;

        patternMode = true;
        return true;
    }

    // =========================================================
    // STATUS
    // =========================================================

    /**
     * Returns true when the shooter execution system is actively doing work.
     */
    public boolean isBusy() {
        return state != State.IDLE;
    }

    // =========================================================
    // TIMING HELPERS
    // These record how long each shot cycle and each phase takes.
    // =========================================================

    /**
     * Starts the timing window for the current press/hold cycle.
     */
    private void beginPressTiming() {
        pressStartTimeMs = System.currentTimeMillis();
        Arrays.fill(currentPressShotTimes, Double.NaN);
        currentPressShotCount = 0;
        pressActive = true;
    }

    /**
     * Records the elapsed time for the current shot relative to the start
     * of the press cycle.
     */
    private synchronized void recordShotTimingForCurrentPress() {
        if (!pressActive) return;

        long now = System.currentTimeMillis();
        int shotIndex = shotsFired - 1;
        double elapsedSec = (now - pressStartTimeMs) / 1000.0;

        // Only store the first three shots in the press cycle
        if (shotIndex >= 0 && shotIndex < 3) {
            currentPressShotTimes[shotIndex] = elapsedSec;
            currentPressShotCount = Math.max(currentPressShotCount, shotIndex + 1);
        }

        // Once three shots are recorded, finalize the press stats
        if (currentPressShotCount >= 3) {
            finalizePress();
        }
    }

    /**
     * Finalizes the current press cycle and updates:
     * - last completed press timings
     * - global averages
     * - running totals
     */
    private synchronized void finalizePress() {
        if (!pressActive) return;

        int n = currentPressShotCount;
        if (n == 0) {
            pressActive = false;
            return;
        }

        double pressSum = 0.0;
        for (int i = 0; i < n; i++) {
            double t = currentPressShotTimes[i];
            if (!Double.isNaN(t)) pressSum += t;
        }
        double pressAvg = pressSum / (double) n;

        // Save the completed press times for telemetry / logging
        for (int i = 0; i < 3; i++) {
            lastCompletedPressShotTimes[i] =
                    Double.isNaN(currentPressShotTimes[i]) ? 0.0 : currentPressShotTimes[i];
        }
        lastCompletedPressAverage = pressAvg;

        // Update global averages
        totalPresses++;
        if (!Double.isNaN(currentPressShotTimes[0])) {
            sumShot1 += currentPressShotTimes[0];
            countShot1++;
            sumAllShotTimes += currentPressShotTimes[0];
            countAllShots++;
        }
        if (n >= 2 && !Double.isNaN(currentPressShotTimes[1])) {
            sumShot2 += currentPressShotTimes[1];
            countShot2++;
            sumAllShotTimes += currentPressShotTimes[1];
            countAllShots++;
        }
        if (n >= 3 && !Double.isNaN(currentPressShotTimes[2])) {
            sumShot3 += currentPressShotTimes[2];
            countShot3++;
            sumAllShotTimes += currentPressShotTimes[2];
            countAllShots++;
        }

        pressActive = false;
    }

    /**
     * Finalizes the press only if timing is currently active.
     */
    private synchronized void finalizePressIfActive() {
        if (pressActive) finalizePress();
    }

    // =========================================================
    // GETTERS FOR TELEMETRY / CSV LOGGING
    // These expose the stored timing and targeting values cleanly.
    // =========================================================

    public synchronized String getStateName() {
        return state.name();
    }

    public synchronized double[] getLastCompletedPressShotTimes() {
        return Arrays.copyOf(lastCompletedPressShotTimes, 3);
    }

    public synchronized double getLastCompletedPressAverage() {
        return lastCompletedPressAverage;
    }

    public synchronized double getGlobalAverageShot1() {
        return (countShot1 > 0) ? (sumShot1 / (double) countShot1) : 0.0;
    }

    public synchronized double getGlobalAverageShot2() {
        return (countShot2 > 0) ? (sumShot2 / (double) countShot2) : 0.0;
    }

    public synchronized double getGlobalAverageShot3() {
        return (countShot3 > 0) ? (sumShot3 / (double) countShot3) : 0.0;
    }

    public synchronized double getGlobalOverallAverage() {
        return (countAllShots > 0) ? (sumAllShotTimes / (double) countAllShots) : 0.0;
    }

    public synchronized long getTotalPressesCounted() {
        return totalPresses;
    }

    public synchronized double getLastMoveToSiloSec() {
        return lastMoveToSiloSec;
    }

    public synchronized double getLastSpinWaitSec() {
        return lastSpinWaitSec;
    }

    public synchronized double getLastSpinUpSec() {
        return lastSpinUpSec;
    }

    public synchronized double getLastFireLiftUpSec() {
        return lastFireLiftUpSec;
    }

    public synchronized double getLastFireLiftDownSec() {
        return lastFireLiftDownSec;
    }

    public synchronized int getLastTargetSiloIndex() {
        return lastTargetSiloIndex;
    }

    public synchronized double getLastTargetAngleDeg() {
        return lastTargetAngleDeg;
    }
}