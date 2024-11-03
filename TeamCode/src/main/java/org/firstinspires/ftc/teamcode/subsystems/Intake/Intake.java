package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveSpeed(double speed){
        intake.setPower(speed);
    }

    public void intaking(){
        intake.setPower(1);
    }

    public void outtaking(){
        intake.setPower(-1);
    }
}
