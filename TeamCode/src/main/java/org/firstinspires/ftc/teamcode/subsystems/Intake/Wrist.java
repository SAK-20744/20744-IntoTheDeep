package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    private Servo wristServo;

//    public static double driving = 1;
    public static double transfering = 1;
    public static double intaking = 0;

    private double wristPosition = 0;

    public Wrist(HardwareMap hardwareMap) {
        wristServo = hardwareMap.get(Servo.class, "wrist");
    }

//    public void driving(){
//        wristServo.setPosition(driving);
//    }
    public void transfering(){wristServo.setPosition(transfering);}
    public void intaking(){
        wristServo.setPosition(intaking);
    }

    public void setPos (double pos) {
        wristServo.setPosition(pos);
    }
    public double getPosition (){
        wristPosition = wristServo.getPosition();
        return wristPosition;
    }

}
