package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    private Servo transferServo;

    public static double holding = 0.5;
    public static double depositing = 0;

    private double transferPosition = 0;

    public Transfer(HardwareMap hardwareMap) {
        transferServo = hardwareMap.get(Servo.class, "transfer");
    }

    public void holding(){
        transferServo.setPosition(holding);
    }
    public void depositing(){transferServo.setPosition(depositing);}

    public void setPos (double pos) {
        transferServo.setPosition(pos);
    }

    public double getPosition (){
        transferPosition = transferServo.getPosition();
        return transferPosition;
    }


}
