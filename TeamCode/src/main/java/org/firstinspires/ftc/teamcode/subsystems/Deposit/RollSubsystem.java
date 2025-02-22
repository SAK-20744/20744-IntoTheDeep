package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class RollSubsystem {

    public enum RollState {
        DEPOSITING, TRANSFERING
    }

    private Servo roll;
    private RollState state;
    public RunAction transferRoll, depoRoll;

    public RollSubsystem(HardwareMap hardwareMap, RollState rollState) {
        roll = hardwareMap.get(Servo.class, "roll");
        this.state = rollState;

        depoRoll = new RunAction(this::depoRoll);
        transferRoll = new RunAction(this::transferRoll);
    }

    public void setPos(double rollPos) {
        roll.setPosition(rollPos);
    }

    public void setState(RollState rollState) {
        if (rollState == rollState.DEPOSITING) {
            roll.setPosition(ROLL_DEPO);
            this.state = rollState.DEPOSITING;
        } else if (rollState == rollState.TRANSFERING) {
            roll.setPosition(ROLL_TRANSFERING);
            this.state = rollState.TRANSFERING;
        }
    }

    public void depoRoll() {
        setState(RollState.DEPOSITING);
    }

    public void transferRoll() {
        setState(RollState.TRANSFERING);
    }

    public void init() {
        Actions.runBlocking(transferRoll);
    }

    public void start() {
        Actions.runBlocking(transferRoll);
    }



}