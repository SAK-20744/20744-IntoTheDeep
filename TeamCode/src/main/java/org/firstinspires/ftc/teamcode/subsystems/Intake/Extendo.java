package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo {

    private Servo leftExtension;
    private Servo rightExtension;

    public static double driving = 1;  // 0 to 1 values
    public static double transfering = 0.5;
    public static double intaking = 0;

    private double leftPosition = 0;
    private double rightPosition = 0;

    public Extendo(HardwareMap hardwareMap) {
        leftExtension = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtension = hardwareMap.get(Servo.class, "rightExtendo");
    }

    public void driving(){
        leftExtension.setPosition(driving);
        rightExtension.setPosition(1-driving);
    }

    public void transfering(){
        leftExtension.setPosition(transfering);
        rightExtension.setPosition(1-transfering);
    }

    public void intaking(){
        leftExtension.setPosition(intaking);
        rightExtension.setPosition(1-intaking);
    }

    public void setPos (double pos) {
        leftExtension.setPosition(pos);
        rightExtension.setPosition(1-pos);
    }

    public double getLeftPosition (){
        leftPosition = leftExtension.getPosition();
        return leftPosition;
    }

    public double getRightPosition (){
        rightPosition = rightExtension.getPosition();
        return rightPosition;
    }

}

