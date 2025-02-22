package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class LiftSubsystem {
    private Telemetry telemetry;
    private DigitalChannel liftLimit;
    private DcMotorEx lLift, rLift;
    public boolean manual = false;
    public boolean hang = false;
    public int pos, bottom;
    public RunAction toZero, toHighBucket, toHighRung, toPark;
    public PIDController liftPID;
    public static int target;
//    public static double p = 0.015, i = 0, d = 0.0005;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lLift = hardwareMap.get(DcMotorEx.class, "lLift");
        rLift = hardwareMap.get(DcMotorEx.class, "rLift");
        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");

        lLift.setDirection(DcMotorSimple.Direction.REVERSE);

        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPID = new PIDController(lp, li, ld);

        toZero = new RunAction(this::toZero);
        toPark = new RunAction(this::toPark);
        toHighBucket = new RunAction(this::toHighBucket);
        toHighRung = new RunAction(this::toHighRung);
    }

    public void updatePIDF() {

        liftPID.setPID(lp,li,ld);
        int pos = rLift.getCurrentPosition();
        double power = liftPID.calculate(pos, target);
        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lLift.setPower(power);
        rLift.setPower(power);

        telemetry.addData("lift pos", getPos());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("lift target", target);

    }

    public double getTarget() {
        return target;
    }

    public boolean isAtTarget() {
        return Math.abs(pos - target) < 25;
    }

    public void setTarget(int b) {
        target = b;
    }

    public int getPos() {
        pos = rLift.getCurrentPosition();
        return pos;
    }

    // OpMode
    public void init() {
        liftPID.setPID(lp,li,ld);

        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void start() {
        target = 0;
    }

    //Presets
    public void toZero() {
        manual = false;
        setTarget(LIFT_RETRACTED);
    }

    public void toPark() {
        manual = false;
        setTarget(LIFT_MID_BASKET);
    }

    public void toHighBucket() {
        manual = false;
        setTarget(LIFT_HIGH_BASKET);
    }

    public void toHighRung() {
        manual = false;
        setTarget(LIFT_HIGH_RUNG);
    }

}