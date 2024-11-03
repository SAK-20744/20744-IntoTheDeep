package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Door {

    private Servo doorServo;

    public static double transfering = 0.5;
    public static double intaking = 1;

    private double doorPosition = 0;

    public Door(HardwareMap hardwareMap) {
        doorServo = hardwareMap.get(Servo.class, "door");
    }

    public void transfering(){
        doorServo.setPosition(transfering);
    }

    public void intaking(){
        doorServo.setPosition(intaking);
    }

    public void setPos (double pos) {
        doorServo.setPosition(pos);
    }
    public double getPosition (){
        doorPosition = doorServo.getPosition();
        return doorPosition;
    }

}
