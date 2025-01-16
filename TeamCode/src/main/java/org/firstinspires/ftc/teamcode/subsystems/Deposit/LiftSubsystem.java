package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class LiftSubsystem {
    private Telemetry telemetry;
    private DigitalChannel liftLimit;
    public DcMotor lift2, leftLift;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public RunAction toZero, toHighBucket, toHighChamber, toHumanPlayer, toTransfer, toPark;
    public PIDController liftPID;
    public static int target;
    public static double p = 0.015, i = 0, d = 0.0005;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        leftLift = hardwareMap.get(DcMotor.class, "lift");
        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");

        lift2.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPID = new PIDController(p, i, d);

        toZero = new RunAction(this::toZero);
        toPark = new RunAction(this::toPark);
        toHighBucket = new RunAction(this::toHighBucket);
    }

    public void updatePIDF(){
        if (!manual) {
            liftPID.setPID(p,i,d);
            
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double pid = liftPID.calculate(getPos(), target);
            double power = pid;
            if(target < getPos())
                power*=0.5;

            lift2.setPower(power);
            leftLift.setPower(power);

            if (!liftLimit.getState()){
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("lift pos", getPos());
            telemetry.addData("Lift Limit", liftLimit.getState());
            telemetry.addData("lift target", target);
        }
    }

    public double getTarget() {
        return target;
    }

    public boolean isAtTarget() {
        return Math.abs(pos - target) < 50;
    }

    public void setTarget(int b) {
        target = b;
    }

//    public int getPos() {
//        pos = lift2.getCurrentPosition() - bottom;
//        return lift2.getCurrentPosition() - bottom;
//    }

    public int getPos() {
        pos = leftLift.getCurrentPosition();
        return pos;
    }

    // OpMode
    public void init() {
        liftPID.setPID(p,i,d);
//        bottom = getPos();

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void start() {
        target = 0;
    }

    //Presets
    public void toZero() {
        manual = false;
        setTarget(liftZeroPos);
    }

    public void toPark() {
        manual = false;
        setTarget(liftToParkPos);
    }

    public void toHighBucket() {
        manual = false;
        setTarget(liftToHighBucketPos);
    }

    public void manual(double n){
        manual = true;

        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        lift2.setPower(n);
        leftLift.setPower(n);
    }

}