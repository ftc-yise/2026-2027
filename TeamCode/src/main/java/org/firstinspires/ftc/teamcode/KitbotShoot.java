package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yise.AutoUtils;
import org.firstinspires.ftc.teamcode.yise.DriveClass;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Autonomous(name = "Kitbot Shoot", group = "Auto")
public class KitbotShoot extends OpMode {
    // Declare OpMode members.
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    AutoUtils AU;

    public final int LAUNCHER_TARGET_VELOCITY = ((2725 / 60) * 28);
    public final int LAUNCHER_MIN_VELOCITY = ((2655 / 60) * 28);
    // RPM = (Ticks/28)*60
    // Ticks = (RPM/60)*28
    // Ticks/Revolution = 28
    // Create a variable to set to the intake.
    double intakePower;
    DriveClass FODrive;

    /** This method is called once at the int of the OpMode. **/
    @Override
    public void init() {
        // hardware map
        intake = hardwareMap.get(DcMotor.class, "Intake");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        windmillServo = hardwareMap.get(CRServo.class, "WindmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "LeftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "RightIntakeServo");
        FODrive = new DriveClass(hardwareMap);
        AU = new AutoUtils(FODrive);

        // set directions
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // set encoder and PID modes
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // set initial powers to 0
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        telemetry.addData("Starting Auto Program", null);
    } // yuipo
    public void resetPowers(){
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        launcher.setVelocity(0);
    }
    public void startIntake(){
        intake.setPower(1);
        leftIntakeServo.setPower(1);
        rightIntakeServo.setPower(1);
    }
    public void init_loop(){}
    public void start(){
        FODrive.stopAllMotors();
        telemetry.addData("Starting Intake", null);
        startIntake();
        telemetry.addData("Starting Flywheel", null);
        launch();
        telemetry.addData("Waiting 10 seconds...", null);
        AU.timeWait(1000);
        telemetry.addData("Resetting Powers", null);
        resetPowers();
        telemetry.addData("Moving From Wall", null);
        AU.moveDir(AutoUtils.backward, 250);
        telemetry.addData("Finished Auto.", null);
    }
    void launch() {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        while(launcher.getVelocity()<LAUNCHER_MIN_VELOCITY){}
        windmillServo.setPower(1);
        AU.timeWait(5000);
        intake.setPower(1);
        AU.timeWait(500);
        intake.setPower(-1);
        AU.timeWait(500);
        intake.setPower(1);
        AU.timeWait(500);
        intake.setPower(-1);
        AU.timeWait(500);
        intake.setPower(1);
        AU.timeWait(500);
        intake.setPower(-1);
        AU.timeWait(500);
        intake.setPower(1);
        AU.timeWait(2000);
    } // robot=good shooting=best good=points
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
    public void loop(){}
}