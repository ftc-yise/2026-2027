package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Turret.turretMode;

@TeleOp(name="turretTest", group="Linear Opmode")
public class turretTest extends LinearOpMode {
    private ElapsedTime homingTimer = new ElapsedTime();
    private ElapsedTime snapTimer = new ElapsedTime();
    private double distance_g;
    private boolean modeTogglePressed = false;

    private enum SnapState { INACTIVE, SNAPPING_LEFT, PUSHING_LEFT, SNAPPING_RIGHT, PUSHING_RIGHT, GOING_HOME, HOMING_ROUTINE }
    private SnapState currentSnapState = SnapState.INACTIVE;

    private static final int LEFT_LIMIT = -1370;
    private static final int CENTER_TARGET = -685;
    private static final int TOLERANCE = 10;

    private double distance = 0.0;

    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, Turret.turretAlliance.BLUE, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. SYSTEM CONTROLS ---
            if (gamepad1.options && !modeTogglePressed) {
                turret.changeMode();
                currentSnapState = SnapState.INACTIVE;
                modeTogglePressed = true;
            }
            if (!gamepad1.options) modeTogglePressed = false;

            if (gamepad1.share) {
                currentSnapState = SnapState.HOMING_ROUTINE;
                homingTimer.reset();
            }

            // --- INPUT DETECTION ---
            if (turret.mode == turretMode.MANUAL) {
                double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;

                if (Math.abs(triggerPower) > 0.05) {
                    currentSnapState = SnapState.INACTIVE;
                    turret.manualControl(triggerPower);
                } else if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    currentSnapState = SnapState.GOING_HOME;
                } else if (gamepad1.right_bumper && currentSnapState != SnapState.SNAPPING_RIGHT && currentSnapState != SnapState.PUSHING_RIGHT) {
                    currentSnapState = SnapState.SNAPPING_RIGHT;
                } else if (gamepad1.left_bumper && currentSnapState != SnapState.SNAPPING_LEFT && currentSnapState != SnapState.PUSHING_LEFT) {
                    currentSnapState = SnapState.SNAPPING_LEFT;
                }
            }

            // --- 3. STATE MACHINE ---
            if (turret.mode == turretMode.AUTO) {
                turret.autoMode();
            } else if (currentSnapState == SnapState.HOMING_ROUTINE) {
                if (gamepad1.touchpad) {
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
            } else if (currentSnapState == SnapState.GOING_HOME) {
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
            // SNAP RIGHT
            else if (currentSnapState == SnapState.SNAPPING_RIGHT) {
                turret.manualControl(1.0);
                if (!turret.limit.getState()) {
                    snapTimer.reset();
                    currentSnapState = SnapState.PUSHING_RIGHT;
                }
            } else if (currentSnapState == SnapState.PUSHING_RIGHT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(0.5);
                else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            // SNAP LEFT
            else if (currentSnapState == SnapState.SNAPPING_LEFT) {
                int currentPos = turret.turret.getCurrentPosition();
                turret.manualControl(-1.0);
                if (currentPos <= LEFT_LIMIT) {
                    snapTimer.reset();
                    currentSnapState = SnapState.PUSHING_LEFT;
                }
            } else if (currentSnapState == SnapState.PUSHING_LEFT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(-0.5);
                else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            } else if (currentSnapState == SnapState.INACTIVE && Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) <= 0.05) {
                turret.stop();
            }

            // --- 4. TELEMETRY ---
            /*telemetry.addLine("      TURRET:");
            telemetry.addLine("---------------");
            telemetry.addData("ENC", turret.turret.getCurrentPosition());
            telemetry.addData("STATE", currentSnapState);*/
            distance = turret.getDistance();
            /*telemetry.addData("DISTANCE", distance);
            telemetry.update();*/
            /*LLResult result = null;
            result = turret.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    //39.37 is a conversion from meters(what botpose gives) to inches
                    double x = botpose.getPosition().x * 39.37;
                    double y = botpose.getPosition().y * 39.37;
                    distance_g = getDistanceFromPose(x, y);

                    telemetry.addData("Distance", distance_g);

                }

                telemetry.addData("MODE", turret.mode);
                telemetry.addData("id", turret.getID());
                telemetry.update();
            */
        }

    }
     /*   double getDistanceFromPose(double x, double y){
            double a = Math.abs(55 - y);
            double b = Math.abs(-58 - x);
            double c_sqrd = Math.pow(a, 2) + Math.pow(b, 2);
            double distance = Math.sqrt(c_sqrd);
            return distance;
        }*/
}