package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp (name = "visiontest")
public class VisionTest extends OpMode {
    VisionSubsystem vision;
    ExtendSubsystem extend;
    IntakeSubsystem intake;
    IntakeSubsystem.IntakePivotState intakePivotState;
    IntakeSubsystem.IntakeSpinState intakeSpinState;
    boolean pressed = false;


    @Override
    public void init() {
        extend = new ExtendSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry, extend);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState);
        vision.switchPipeline(VisionSubsystem.limelightState.blue);
        extend.retract();
    }

    @Override
    public void start() {
        vision.start();
        intake.pivotGround();
    }

    @Override
    public void loop() {
        vision.updateColor();

        if(gamepad1.a && pressed == false) {
            vision.extendAlign(vision.getTxError());
            vision.driveAlign(vision.getTyError());
            pressed = true;
        }

        if(gamepad1.dpad_down) {
            extend.toAuto();
            pressed = false;
        }

        if(gamepad1.b) {
            intake.spinIn();
        } else if(gamepad1.x) {
            intake.spinOut();
        } else if (gamepad1.y) {
            intake.spinStop();
        }
    }
}