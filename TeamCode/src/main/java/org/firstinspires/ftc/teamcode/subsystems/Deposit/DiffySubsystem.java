package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class DiffySubsystem {

    public enum diffyState {
        TRANSFER, WALL, SCORING, CLIP
    }

    private Servo lDiffy, rDiffy;
    private diffyState state;
    public RunAction diffyMoveTransfering, diffyMoveScoring, diffyMoveClipping, diffyMoveWall;

    public DiffySubsystem(HardwareMap hardwareMap, diffyState diffyState) {
        lDiffy = hardwareMap.get(Servo.class, "lDiffy");
        rDiffy = hardwareMap.get(Servo.class, "rDiffy");
        this.state = diffyState;

        diffyMoveTransfering = new RunAction(this::transferdiffy);
        diffyMoveScoring = new RunAction(this::scoringdiffy);
        diffyMoveClipping = new RunAction(this::clipdiffy);
        diffyMoveWall = new RunAction(this::walldiffy);
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
        } else if (diffyState == diffyState.WALL) {
            lDiffy.setPosition(LDIFFY_WALL);
            rDiffy.setPosition(RDIFFY_WALL);
            this.state = diffyState.WALL;
        } else if (diffyState == diffyState.SCORING) {
            lDiffy.setPosition(LDIFFY_SCORING);
            rDiffy.setPosition(RDIFFY_SCORING);
            this.state = diffyState.SCORING;
        }
        else if (diffyState == diffyState.CLIP) {
            lDiffy.setPosition(LDIFFY_CLIPPING);
            rDiffy.setPosition(RDIFFY_CLIPPING);
            this.state = diffyState.CLIP;
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

    public void scoringdiffy() {
        setState(diffyState.SCORING);
    }

    public void init() {
        Actions.runBlocking(diffyMoveTransfering);
    }

    public void start() {
        Actions.runBlocking(diffyMoveClipping);
    }



}