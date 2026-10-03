
package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ledclass {
    private Servo ledServo;

    // Constructor
    public Ledclass(HardwareMap hardwareMap, String deviceName) {
        ledServo = hardwareMap.get(Servo.class, deviceName);
    }


    public void setColor(double position) {
        ledServo.setPosition(position);
    }


    public void setYellow() {
        setColor(0.338);
    }
    public void setOff() {
        setColor(0);
    }

    public void setGreen() {
        setColor(0.5);
    }

    public void setBlue() {
        setColor(0.611);
    }
}



