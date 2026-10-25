package org.firstinspires.ftc.teamcode.yise;

public class AutoUtils {
    DriveClass FODrive;
    public static String right = "right";
    public static String left = "left";
    public static String forward = "forward";
    public static String backward = "backward";
    private final Object lock = new Object();

    public AutoUtils(DriveClass Drive) {
        FODrive = Drive;
    }

    public void moveDir(String dir, int time) {
        FODrive.stopAllMotors();
        if (dir.equals(right)) {
            FODrive.frontRightForward();
            FODrive.frontLeftForward();
            FODrive.backRightBackward();
            FODrive.backLeftBackward();
        } else if (dir.equals(left)) {
            FODrive.frontRightBackward();
            FODrive.frontLeftBackward();
            FODrive.backRightForward();
            FODrive.backLeftForward();
        } else if (dir.equals(forward)) {
            FODrive.frontLeftForward();
            FODrive.frontRightForward();
            FODrive.backLeftForward();
            FODrive.backRightForward();
        } else if (dir.equals(backward)) {
            FODrive.frontLeftBackward();
            FODrive.frontRightBackward();
            FODrive.backLeftBackward();
            FODrive.backRightBackward();
        }
        timeWait(time);
        FODrive.stopAllMotors();
    }

    public void timeWait(int millis) {
        synchronized (lock) {
            try {
                lock.wait(millis);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
