package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Game Values (RUN THIS EVERY MATCH)", group="Necessity")
public class Parameters extends LinearOpMode {
    public enum Color {
        RED,
        BLUE
    }
    public enum AUTONOMOUS {
        YES,
        NO
    }

    public static double spinLocation = 240;


    public static Color allianceColor;
    public static AUTONOMOUS autonomous;

    public static double WAIT;
    public boolean xReleased;

    @Override
    public void runOpMode() {
        Spindexer spin = new Spindexer(hardwareMap);

        while (!gamepad1.a && !gamepad1.b) {
            telemetry.addLine("Alliance Color \n");
            telemetry.addLine("X - Blue \n O - Red");
            telemetry.update();

            if (gamepad1.a) {
                allianceColor = Color.BLUE;
            } else if (gamepad1.b) {
                allianceColor = Color.RED;
            }
        }

        while (gamepad1.a || gamepad1.b) {
                //WAIT until released
        }

        while (!gamepad1.y) {
            telemetry.addLine("Wait Seconds: " + WAIT);
            telemetry.addLine("▢ = -1 \n X = +1 \n O = 0 \n Y to continue");
            telemetry.update();

            if (gamepad1.x && xReleased) {
                WAIT--;
                xReleased = false;
            } else if (gamepad1.a && xReleased) {
                WAIT++;
                xReleased = false;
            } else if (gamepad1.b && xReleased) {
                WAIT = 0;
                xReleased = false;
            }

            if (!gamepad1.x && !gamepad1.a && !gamepad1.b && !xReleased) {
                xReleased = true;
            }
        }

        while (gamepad1.y){

        }

        while (!gamepad1.y) {
            spin.sampleSensorsNow();   // 1️⃣ read hardware
            spin.update();
            telemetry.addLine("spindexers current position \n");
            telemetry.addLine("triang or Y /n sets the it to the current angle" + "spindexers current angle == /n" + spin.getTelemetry().currentAngle);
            telemetry.update();

            if (gamepad1.y) {
                spinLocation = spin.getTelemetry().currentAngle;
                spin.initSilos();
            }
        }

        while (gamepad1.y) {
            //WAIT until released
        }

        while (!gamepad1.a) {
            telemetry.addLine("Color: " + allianceColor);
            telemetry.addLine("WAIT: " + WAIT);
            telemetry.addLine("spin: " + spinLocation);
            telemetry.addLine("\nX to end program");
            telemetry.update();
        }
        telemetry.addLine("Configuration complete. Self-destructing");
        telemetry.update();
        sleep(1500);
    }
}