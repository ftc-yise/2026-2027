package org.firstinspires.ftc.teamcode.yise;

import java.util.Arrays;

public class ShotPatternManager {

    public static final int MAX_SHOTS = 9;

    public enum ShotPattern {
        GPP(new Spindexer.BallColor[]{
                Spindexer.BallColor.GREEN,
                Spindexer.BallColor.PURPLE,
                Spindexer.BallColor.PURPLE,
        }),
        PPG(new Spindexer.BallColor[]{
                Spindexer.BallColor.PURPLE,
                Spindexer.BallColor.PURPLE,
                Spindexer.BallColor.GREEN
        }),
        PGP(new Spindexer.BallColor[]{
                Spindexer.BallColor.PURPLE,
                Spindexer.BallColor.GREEN,
                Spindexer.BallColor.PURPLE
        });

        public final Spindexer.BallColor[] sequence;

        ShotPattern(Spindexer.BallColor[] seq) {
            this.sequence = seq;
        }
    }


    private final Spindexer.BallColor[] shotQueue =
            new Spindexer.BallColor[MAX_SHOTS];

    private int size = 0;

    // ─────────────────────────────────────────
    // CLEAR (Gamepad button)
    // ─────────────────────────────────────────
    public void clear() {
        Arrays.fill(shotQueue, Spindexer.BallColor.NONE);
        size = 0;
    }

    // ─────────────────────────────────────────
    // ADD PATTERN (3-ball pattern)
    // ─────────────────────────────────────────
    public void addPattern(Spindexer.BallColor[] pattern) {
        if (pattern == null) return;

        for (Spindexer.BallColor color : pattern) {
            if (size >= MAX_SHOTS) return;
            shotQueue[size++] = color;
        }
    }


    // ─────────────────────────────────────────
    // ADD SINGLE BALL (intake mistake recovery)
    // ─────────────────────────────────────────
    public void addBall(Spindexer.BallColor color) {
        if (size >= MAX_SHOTS) return;
        shotQueue[size++] = color;
    }

    // ─────────────────────────────────────────
    // GET NEXT SHOT
    // ─────────────────────────────────────────
    public Spindexer.BallColor getNext() {
        if (size == 0) return Spindexer.BallColor.NONE;

        Spindexer.BallColor next = shotQueue[0];
        System.arraycopy(shotQueue, 1, shotQueue, 0, MAX_SHOTS - 1);
        shotQueue[MAX_SHOTS - 1] = Spindexer.BallColor.NONE;
        size--;
        return next;
    }

    // ─────────────────────────────────────────
    // STATE QUERIES
    // ─────────────────────────────────────────
    public boolean hasShots() {
        return size > 0;
    }

    public boolean isFull() {
        return size >= MAX_SHOTS;
    }

    public Spindexer.BallColor[] snapshot() {
        return Arrays.copyOf(shotQueue, MAX_SHOTS);
    }
}
