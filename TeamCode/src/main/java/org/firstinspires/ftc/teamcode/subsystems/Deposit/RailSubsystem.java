package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;

public class RailSubsystem {

    public enum railState {
        TRANSFER, WALL, SCORING, CLIP
    }

    private Servo rail;
    private railState state;
    public RunAction railMoveTransfering, railMoveScoring, railMoveClipping, railMoveWall;

    public RailSubsystem(HardwareMap hardwareMap, railState railState) {
        rail = hardwareMap.get(Servo.class, "rail");
        this.state = railState;

        railMoveTransfering = new RunAction(this::transferRail);
        railMoveScoring = new RunAction(this::scoringRail);
        railMoveClipping = new RunAction(this::clipRail);
        railMoveWall = new RunAction(this::wallRail);
    }

    public void setPos(double railPos) {
        rail.setPosition(railPos);
    }

    public void setState(railState railState) {
        if (railState == railState.TRANSFER) {
            rail.setPosition(RAIL_TRANSFERING);
            this.state = railState.TRANSFER;
        } else if (railState == railState.WALL) {
            rail.setPosition(RAIL_WALL);
            this.state = railState.WALL;
        } else if (railState == railState.SCORING) {
            rail.setPosition(RAIL_SCORING);
            this.state = railState.SCORING;
        }
        else if (railState == railState.CLIP) {
            rail.setPosition(RAIL_CLIPPING);
            this.state = railState.CLIP;
        }
    }

    public void transferRail() {
        setState(railState.TRANSFER);
    }

    public void clipRail() {
        setState(railState.CLIP);
    }

    public void wallRail() {
        setState(railState.WALL);
    }

    public void scoringRail() {
        setState(railState.SCORING);
    }

    public void init() {
        Actions.runBlocking(railMoveTransfering);
    }

    public void start() {
        Actions.runBlocking(railMoveClipping);
    }



}