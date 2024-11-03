package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotor liftmotor;
    private DigitalChannel liftswitch;
    private boolean liftLimit = true;
    private double liftposition = 0;
    private double liftHigh = 2855;
    private double liftLow = 1330;

    private double liftMedium = 2125;
    private double liftIntake = 50;
    private double liftOff = 1000;
    private double liftTarget = 0;
    private boolean moving = false;
    private double liftPower = 0;
    private boolean liftLimitReset = true;
    private long startTime;

    //milliseconds
    private double errorTime = 2000;


    public Lift(HardwareMap hardwareMap) {
        liftmotor = hardwareMap.get(DcMotor.class, "lift");
        liftswitch = hardwareMap.get(DigitalChannel.class, "lift switch");
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean updatelift(){
        liftposition = liftmotor.getCurrentPosition();
        liftLimit = liftswitch.getState();

        if (moving){
            if(!liftmotor.isBusy()){
                moving = false;
            }
        }

        if(liftLimit){
            if (liftLimitReset){
                liftLimitReset = false;
                liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        else{
            liftLimitReset = true;
        }
        if (liftLimit )
        {
            liftPower = Math.min(Math.max(liftPower, 0), 1);
        }
        liftmotor.setPower(liftPower);

        return liftPower != 0;
    }
    public boolean move(float liftSetPower){
        if (liftSetPower!=0){
            moving =false;
        }
        if(!moving) {
            liftPower = -liftSetPower;
            liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return liftPower != 0;
    }
    public void cancel()
    {
        moving = false;
        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPower = 0;
        liftmotor.setPower(liftPower);
    }

    public boolean isBusy(){
        if(!moving) {
            return true;
        }
        return false;
    }
    public double whereAmI()
    {
        liftposition = liftmotor.getCurrentPosition();
        return liftposition;
    }
    public boolean getLimit()
    {
        liftLimit = liftswitch.getState();
        return liftLimit;
    }

}
