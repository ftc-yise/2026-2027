package org.firstinspires.ftc.teamcode.yise;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Configurable
public class Turret {
    double mySlope = 0.15;
    double myOffset = 0.25;
    double tx = 0.0;
    public int homePos = 295;       // Right side home
    public int farLimit = -1075;    // Left side home
    public int centerPos = (homePos + farLimit) / 2; // Approx. calculated center


    // --- PID CONTROL CONSTANTS  ---

    // --- PIDF CONTROL CONSTANTS ---
    public static double kP = 0.016;
    public static double kI = 0.00089;
    public static double kD = 0.0065;
    public static double kF = 0.12;   // static friction feedforward

    public static double AUTO_MAX_POWER = 0.5;
    public static double TARGET_TOLERANCE_DEG = 1.3;
    public static double AUTO_MIN_POWER_FLOOR = 0.12;

    // --- ANALOG MANUAL CONTROL CONSTANTS  ---
    public static double MAX_MANUAL_POWER = 1.0;
    public static double ANALOG_POWER_CURVE_EXPONENT = 2.0;
    public static double MIN_ANALOG_POWER_FLOOR = 0.1;

    // --- PID STATE VARIABLES ---
    private double lastError = 0.0;
    private long lastTime = System.currentTimeMillis();
    private double integralSum = 0.0;

    double filtTx = 0;
    double filtVel = 0;
    public DcMotor turret;
    double lastEncoderPos;

    double lastKnownEncoderPos = lastEncoderPos;
    public double lastVelocityTime;


    // --- CLASS VARIABLES ---
    LLResult result = null;
    public double turretPower = 0.0;
    public double pose = 0.0;
    public double myTx = 0.0;
    public Limelight3A limelight;
    public DigitalChannel limit; // Digital device for limit switch instead of push sensor because I get more advanced control
    public Telemetry telemetry;

    public enum turretAlliance {RED, BLUE}
    public turretAlliance currentAlliance;

    public enum turretDirection {LEFT, RIGHT, STOP}

    public enum turretMode {AUTO, MANUAL}

    public turretMode mode;

    public Turret(HardwareMap hardwareMap, turretAlliance alliance, Telemetry telem) {
        telemetry = telem;
        currentAlliance = alliance;

        turret = hardwareMap.get(DcMotor.class, "turret");

        // RESET ENCODER but stay in WITHOUT_ENCODER mode
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //turret.setDirection(DcMotor.Direction.REVERSE);

        // Limit Switch Setup
        limit = hardwareMap.get(DigitalChannel.class, "limit");
        limit.setMode(DigitalChannel.Mode.INPUT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(40);
        limelight.start();
        if (currentAlliance == turretAlliance.RED) {
            limelight.pipelineSwitch(4);
        } else if (currentAlliance == turretAlliance.BLUE) {
            limelight.pipelineSwitch(3);
        }
        mode = turretMode.MANUAL;
        lastTime = System.currentTimeMillis();
    }

    // Limit switch safety
    private double applySafety(double power) {
        // Digital sensors are TRUE when open, FALSE when pressed
        boolean isPressed = !limit.getState();

        if (isPressed) {
            int pos = turret.getCurrentPosition();
            if (pos > 10 && power > 0) return 0;
            if (pos < -10 && power < 0) return 0;
        }
        return Range.clip(power, -1.0, 1.0);
    }

    public void changeMode() {
        if (mode == turretMode.AUTO) {
            mode = turretMode.MANUAL;
            stop();
            resetPD();
            resetVelocityTracking();
        } else if (mode == turretMode.MANUAL) {
            mode = turretMode.AUTO;
        }
    }

    public void manualMode(turretDirection direction) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mode = turretMode.MANUAL;
        double p = 0;
        switch (direction) {
            case LEFT:
                p = -0.4;
                break;
            case RIGHT:
                p = 0.4;
                break;
            case STOP:
                p = 0;
                break;
        }
        turret.setPower(applySafety(p));
    }

    public void manualControl(double power) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mode = turretMode.MANUAL;

        // --- POWER CURVE MATH STUFF ---
        double curbedPower = Math.signum(power) * Math.pow(Math.abs(power), ANALOG_POWER_CURVE_EXPONENT);
        double finalPower = curbedPower * MAX_MANUAL_POWER;

        if (Math.abs(finalPower) > 0.0 && Math.abs(finalPower) < MIN_ANALOG_POWER_FLOOR) {
            finalPower = Math.signum(finalPower) * MIN_ANALOG_POWER_FLOOR;
        }

        turretPower = applySafety(finalPower);
        turret.setPower(turretPower);
    }

    public void runto(int x){
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(x);
    }

    public void autoModeStatic() {
        mode = turretMode.AUTO;

        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            myTx = result.getTx();
            turretPower = getTurretPower(myTx, myOffset, mySlope);
            //telemetry.addData("Ty=", myTy);
        } else {
            turretPower = 0;
        }
        //telemetry.addData("Power=", turretPower);
        //telemetry.update();
        turret.setPower(turretPower);
    }

    public void autoMode() {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mode = turretMode.AUTO;
        result = limelight.getLatestResult();
        if (!result.isValid()) {
            // hold last known encoder pos with small P-only encoder loop
            double posError = lastKnownEncoderPos - turret.getCurrentPosition();
            turret.setPower(Range.clip(posError * 0.004, -0.2, 0.2));
            return;
        }
        lastKnownEncoderPos = turret.getCurrentPosition();

        if (result != null && result.isValid()) {
            if (lastTime == 0) {
                resetVelocityTracking();
            }
            double currentError = result.getTx();
            myTx = currentError;

            double pidfPower = calculatePIDF(currentError);
            turretPower = applySafety(pidfPower);

        } else {
            // Hold last known direction gently instead of snapping
            turretPower = Range.clip(lastError * 0.02, -0.15, 0.15);
        }


        turret.setPower(turretPower);
    }

    public void stop() {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setPower(0);
    }

    private double getTurretPower(double tx, double myOffset, double mySlope) {
        double myPower = 0.0;

        if (tx < 0) {
            myPower = -.2 * (tx * mySlope + myOffset);
            if (myPower > 0.7) {
                myPower = AUTO_MAX_POWER;
            } else if (myPower < 0.35) {
                myPower = AUTO_MIN_POWER_FLOOR;
            }
            return myPower;
        } else if (tx > 0) {
            myPower = -.2 * (tx * mySlope - myOffset);
            if (myPower < -0.7) {
                myPower = -1 * AUTO_MAX_POWER;
            } else if (myPower > -0.35) {
                myPower = -1 * AUTO_MIN_POWER_FLOOR;
            }
            return myPower;
        } else {
            return myPower;
        }
    }

    public void setHome() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetPD() {
        lastError = 0.0;
        lastTime = System.currentTimeMillis();
    }

    private double calculatePIDF(double error) {
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lastEncoderPos = turret.getCurrentPosition();


        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTime)/1000.0, 0.02);
        if (dt <= 0) dt = 0.04;

        // --- Deadband ---
        if (Math.abs(error) < TARGET_TOLERANCE_DEG) {
            // Hold against static friction instead of stopping
            return kF * Math.signum(error);
        }
        double rawVel = (turret.getCurrentPosition() - lastEncoderPos) / dt;
        filtVel = 0.6 * filtVel + 0.4 * rawVel;
        lastEncoderPos = turret.getCurrentPosition();


        // --- Integral ---
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -10, 10);

        // --- Velocity-based D ---
        int currentPos = turret.getCurrentPosition();
        double velocityDt = (now - lastVelocityTime) / 1000.0;
        if (velocityDt <= 0) velocityDt = 0.04;

        double angularVelocity = (currentPos - lastEncoderPos) / velocityDt;
        double derivative = -kD * angularVelocity;

        lastEncoderPos = currentPos;
        lastVelocityTime = now;

        // --- Scaled Feedforward ---
        double feedforward =
                kF * Math.signum(error) *
                        Range.clip(Math.abs(error) / 5.0, 0.0, 1.0);

        // --- PIDF sum ---
        double output =
                (kP * error) +
                        (kI * integralSum) +
                        derivative +
                        feedforward;

        // --- Slowdown near target ---
        double slowZone = 1.5;
        if (Math.abs(error) < slowZone) {
            output *= Math.abs(error) / slowZone;
        }

        output = Range.clip(output, -AUTO_MAX_POWER, AUTO_MAX_POWER);

        lastError = error;
        lastTime = now;

        // --- Minimum power floor to overcome static friction ---
        double minPower = 0.12;

        if (Math.abs(output) > 0 && Math.abs(output) < minPower) {
            output = Math.signum(output) * minPower;
        }


        return output;
    }

    public double getTx() {
        myTx = tx;
        return myTx;
    }

    public int getID() {

        // Always refresh the result
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1; // No Limelight data
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return -1; // No AprilTags detected
        }

        // Return the first detected tag's ID
        {
            return fiducials.get(0).getFiducialId();
        }
    }

    public double getDistance() {
        double distance = 0.0;
        double a = 0.0;
        double b = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            //39.37 is a conversion from meters(what botpose gives) to inches
            double x = botpose.getPosition().x * 39.37;
            double y = botpose.getPosition().y * 39.37;
            telemetry.addData("x", x);
            telemetry.addData("y", y);

            if (currentAlliance == turretAlliance.RED) {
                telemetry.addData("Alliance", "RED");

                a = Math.abs(55 - y);
                b = Math.abs(-58 - x);
            } else if (currentAlliance == turretAlliance.BLUE) {
                telemetry.addData("Alliance", "BLUE");
                a = Math.abs(55 - y);
                b = Math.abs(-58 - x);
            }
            telemetry.addData("a", a);
            telemetry.addData("b", b);

            double c_sqrd = Math.pow(a, 2) + Math.pow(b, 2);
            telemetry.addData("c_sqrd", c_sqrd);
            distance = Math.sqrt(c_sqrd);
            telemetry.addData("distance", distance);

        }
        telemetry.update();
        return distance;
    }

    private void resetPID() {
        lastError = 0.0;
        integralSum = 0.0;
        lastTime = System.currentTimeMillis();
    }

    public double getPose() {
        pose = turret.getCurrentPosition();
        return pose;
    }
    private void resetVelocityTracking() {
        lastEncoderPos = turret.getCurrentPosition();
        lastVelocityTime = System.currentTimeMillis();
    }

}
