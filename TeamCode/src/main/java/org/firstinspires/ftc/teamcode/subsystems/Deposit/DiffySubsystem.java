package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class DiffySubsystem {

    public enum diffyState {
        TRANSFER, WALL, SCORING, CLIP, SAMPLE_AUTO, NEW_AUTO
    }

    private Servo lDiffy, rDiffy;
    private diffyState state;
    public RunAction diffyMoveTransfering, diffyMoveScoring, diffyMoveClipping, diffyMoveWall, diffyMoveAuto;

    public DiffySubsystem(HardwareMap hardwareMap, diffyState diffyState) {
        lDiffy = hardwareMap.get(Servo.class, "lDiffy");
        rDiffy = hardwareMap.get(Servo.class, "rDiffy");
        this.state = diffyState;

        diffyMoveTransfering = new RunAction(this::transferdiffy);
        diffyMoveScoring = new RunAction(this::scoringdiffy);
        diffyMoveClipping = new RunAction(this::clipdiffy);
        diffyMoveWall = new RunAction(this::walldiffy);
        diffyMoveAuto = new RunAction(this::autodiffy);
    }

    public void setPos(double lDiffyPos, double rDiffyPos) {
        lDiffy.setPosition(lDiffyPos);
        rDiffy.setPosition(rDiffyPos);
    }

    public void setState(diffyState diffyState) {
        if (diffyState == diffyState.TRANSFER) {
            lDiffy.setPosition(LDIFFY_TRANSFERING);
            rDiffy.setPosition(RDIFFY_TRANSFERING);
            this.state = diffyState.TRANSFER;
        } else if (diffyState == diffyState.SCORING) {
            lDiffy.setPosition(LDIFFY_SCORING);
            rDiffy.setPosition(RDIFFY_SCORING);
            this.state = diffyState.SCORING;
        } else if (diffyState == DiffySubsystem.diffyState.SAMPLE_AUTO)
        {
            lDiffy.setPosition(LDIFFY_START);
            rDiffy.setPosition(RDIFFY_START);
            this.state = diffyState.SAMPLE_AUTO;
        }
    }

    public void transferdiffy() {
        setState(diffyState.TRANSFER);
    }

    public void clipdiffy() {
        setState(diffyState.CLIP);
    }

    public void walldiffy() {
        setState(diffyState.WALL);
    }

    public void autodiffy() {
        setState(diffyState.SAMPLE_AUTO);
    }

    public void newautodiffy() {
        setState(diffyState.NEW_AUTO);
    }

    public void scoringdiffy() {
        setState(diffyState.SCORING);
    }

    public void init() {
        Actions.runBlocking(diffyMoveAuto);
    }

    public void start() {
        Actions.runBlocking(diffyMoveClipping);
    }



}