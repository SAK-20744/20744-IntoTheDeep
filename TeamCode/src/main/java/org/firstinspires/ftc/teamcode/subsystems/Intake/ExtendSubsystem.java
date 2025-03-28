package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

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

public class ExtendSubsystem {

    private Telemetry telemetry;
    private DigitalChannel extendoLimit;
    private DcMotorEx extendo;

    public int pos;
    public PIDController extendoPID;
    public static int target;
    public RunAction extendExtendo, retractExtendo, autoExtendo;

    public ExtendSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoLimit = hardwareMap.get(DigitalChannel.class, "extendoLimit");

        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendoPID = new PIDController(ep, ei, ed);

        extendExtendo = new RunAction(this::extend);
        retractExtendo = new RunAction(this::retract);
        autoExtendo = new RunAction(this::toAuto);
    }

    public void updatePIDF() {

        extendoPID.setPID(ep,ei,ed);
        int pos = extendo.getCurrentPosition();
        double power = extendoPID.calculate(pos, target);
        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(power < 0)
                power = E_RETRACT_POWER;
        }

        extendo.setPower(power);

        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
        telemetry.addData("Extendo Power", power);
        telemetry.addData("Extendo Limit", extendoLimit.getState());
        telemetry.addData("extendo target", target);

    }

    public double getTarget() {
        return target;
    }

    public boolean isAtTarget() {
        return Math.abs(pos - target) < 25;
    }

    public void setTarget(int b) {
        if(b<EXTENDO_RETRACTED)
            target = EXTENDO_RETRACTED;
        else if(b>EXTENDO_EXTENDED)
            target = EXTENDO_EXTENDED;
        else
            target = b;
    }

    public int getPos() {
        pos = extendo.getCurrentPosition();
        return pos;
    }

    // OpMode
    public void init() {
        extendoPID.setPID(ep,ei,ed);

        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void init_loop() {

        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void start() {
        target = 0;
    }

    //Presets
    public void retract() {
        setTarget(EXTENDO_RETRACTED);
    }

    public void extend() {
        setTarget(EXTENDO_EXTENDED);
    }

    public void toAuto() {
        setTarget(EXTENDO_AUTO);
    }

}