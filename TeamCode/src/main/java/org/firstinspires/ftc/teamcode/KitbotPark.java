package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yise.AutoUtils;
import org.firstinspires.ftc.teamcode.yise.DriveClass;

import java.util.FormatFlagsConversionMismatchException;

@Autonomous(name = "Kitbot Park Right", group = "Auto")
public class KitbotPark extends OpMode {
    Telemetry.Item status;
    DriveClass FODrive;
    AutoUtils AU;
    /** This method is called once at the int of the OpMode. **/
    @Override
    public void init() {
        FODrive = new DriveClass(hardwareMap);
        AU = new AutoUtils(FODrive);
        telemetry.addData("Starting Auto Program", null);
    } // yuipo
    public void init_loop(){}
    public void start(){
        telemetry.addData("Moving Off The Wall", null);
        AU.moveDir(AutoUtils.backward,250);
        telemetry.addData("Moving Right", null);
        AU.moveDir(AutoUtils.right, 400);
        telemetry.addData("Finished Auto.", null);
    } // robot=good shooting=best good=points
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
    public void loop(){}
}
