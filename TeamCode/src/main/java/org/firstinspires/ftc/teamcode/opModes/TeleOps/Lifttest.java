package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@TeleOp(name="Lift_Test", group="z")
public class Lifttest extends OpMode {
    private PIDController liftPID;
    private DigitalChannel liftLimit;
    public static double p = -0.015, i = 0, d = 0.000005;
    public static int target;
    private DcMotor lLift, rLift;
    public int pos;
    private double power;

    @Override
    public void init() {
        liftPID = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lLift = hardwareMap.get(DcMotor.class, "lLift");
        rLift = hardwareMap.get(DcMotor.class, "rLift");
        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        lLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        liftPID.setPID(p,i,d);
        pos = rLift.getCurrentPosition();
        power = liftPID.calculate(pos, target);

        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lLift.setPower(power);
        rLift.setPower(power);

        telemetry.addData("Lift pos ", pos);
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.update();
    }
}