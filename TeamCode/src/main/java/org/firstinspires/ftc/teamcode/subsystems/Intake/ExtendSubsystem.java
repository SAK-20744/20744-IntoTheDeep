package org.firstinspires.ftc.teamcode.subsystems.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem.ExtendoState.EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem.ExtendoState.RETRACTED;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.EXTENDO_EXTENDED;
import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.EXTENDO_RETRACTED;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class ExtendSubsystem {

    private Servo leftExtension;
    private Servo rightExtension;

    public enum ExtendoState {
        EXTENDED, RETRACTED
    }

    private ExtendoState state;
    public RunAction extendExtendo, retractExtendo;

    public ExtendSubsystem(HardwareMap hardwareMap, ExtendoState extendoState) {

        leftExtension = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtension = hardwareMap.get(Servo.class, "rightExtendo");

        this.state = extendoState;

        extendExtendo = new RunAction(this::extend);
        retractExtendo = new RunAction(this::retract);
    }

    public void setPos(double extendoPos) {
        leftExtension.setPosition(extendoPos);
        rightExtension.setPosition(1-extendoPos);
    }

    public void setState(ExtendoState extendoState) {
        if (extendoState == RETRACTED) {
            leftExtension.setPosition(EXTENDO_RETRACTED);
            rightExtension.setPosition(1-EXTENDO_RETRACTED);
            this.state = RETRACTED;
        } else if (extendoState == EXTENDED) {
            leftExtension.setPosition(EXTENDO_EXTENDED);
            rightExtension.setPosition(1-EXTENDO_EXTENDED);
            this.state = EXTENDED;
        }
    }

    public void switchState() {
        if (state == EXTENDED) {
            setState(RETRACTED);
        } else if (state == RETRACTED) {
            setState(EXTENDED);
        }
    }

    public void extend() {
        setState(EXTENDED);
    }

    public void retract() {
        setState(RETRACTED);
    }

    public void init() {
        Actions.runBlocking(retractExtendo);
    }

    public void start() {
        Actions.runBlocking(retractExtendo);
    }



}