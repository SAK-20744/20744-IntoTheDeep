package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.SequentialAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;

//@Disabled
@Autonomous(name="BlueBucket", group="B")
public class BlueBucket extends OpMode {
    public int pathState;
    public Auto auto;

    @Override
    public void init() {
        auto = new Auto(hardwareMap, telemetry, new Follower(hardwareMap), true, true);
        Actions.runBlocking(auto.extend.retractExtendo);
        Actions.runBlocking(auto.intake.pivotTransfer);
        Actions.runBlocking(auto.claw.openClaw);
        Actions.runBlocking(auto.depo.armIn);
        Actions.runBlocking(auto.pitch.pitchIn);
    }

    @Override
    public void start() {
        auto.start();
        setPathState(0);
    }

    @Override
    public void loop() {
        auto.update();
        pathUpdate();

        telemetry.addData("state", pathState);
        telemetry.addData("x", auto.follower.getPose().getX());
        telemetry.addData("y", auto.follower.getPose().getY());
        telemetry.addData("h", auto.follower.getPose().getHeading());
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(auto.extend.retractExtendo);
                Actions.runBlocking(auto.intake.pivotTransfer);
                Actions.runBlocking(auto.claw.closeClaw);
//                Actions.runBlocking(auto.lift.toHighBucket);
                Actions.runBlocking(auto.depo.armOut);
                Actions.runBlocking(auto.pitch.pitchOut);
                auto.follower.followPath(auto.preload);
                setPathState(1);
                break;
            case 1:
                if(!auto.follower.isBusy()) {

//                    if (auto.lift.isAtTarget()) {
                        Actions.runBlocking(auto.claw.openClaw);
//                    }
                    Actions.runBlocking(auto.depo.armIn);
                    Actions.runBlocking(auto.pitch.pitchIn);
//                    Actions.runBlocking(auto.lift.toZero);
                    auto.follower.followPath(auto.element1);
                    Actions.runBlocking(auto.extend.extendExtendo);
                    Actions.runBlocking(auto.intake.pivotGround);
                    Actions.runBlocking(auto.intake.spinIn);
                    setPathState(2);
                }
                break;
            case 2:
                if(!auto.follower.isBusy()) {
                    auto.follower.followPath(auto.score1);
                    Actions.runBlocking(auto.intake.spinStop);
                    Actions.runBlocking(auto.intake.pivotTransfer);
                    Actions.runBlocking(auto.extend.retractExtendo);
                    setPathState(3);
                }
                break;
            case 3:
                if(!auto.follower.isBusy()) {
                    Actions.runBlocking(auto.claw.closeClaw);
                    Actions.runBlocking(auto.depo.armOut);
                    Actions.runBlocking(auto.pitch.pitchOut);
                    setPathState(4);
                }
                break;
            case 4:
                if(!auto.follower.isBusy()) {

//                    if (auto.lift.isAtTarget()) {
                    Actions.runBlocking(auto.claw.openClaw);
//                    }
                    Actions.runBlocking(auto.depo.armIn);
                    Actions.runBlocking(auto.pitch.pitchIn);
//                    Actions.runBlocking(auto.lift.toZero);
                    auto.follower.followPath(auto.element2);
                    Actions.runBlocking(auto.extend.extendExtendo);
                    Actions.runBlocking(auto.intake.pivotGround);
                    Actions.runBlocking(auto.intake.spinIn);
                    setPathState(5);
                }
                break;
            case 5:
                if(!auto.follower.isBusy()) {
                    auto.follower.followPath(auto.score2);
                    Actions.runBlocking(auto.intake.spinStop);
                    Actions.runBlocking(auto.intake.pivotTransfer);
                    Actions.runBlocking(auto.extend.retractExtendo);
                    setPathState(6);
                }
                break;
            case 6:
                if(!auto.follower.isBusy()) {
                    Actions.runBlocking(auto.claw.closeClaw);
                    Actions.runBlocking(auto.depo.armOut);
                    Actions.runBlocking(auto.pitch.pitchOut);
                    setPathState(7);
                }
                break;
            case 7:
                if(!auto.follower.isBusy()) {

//                    if (auto.lift.isAtTarget()) {
                    Actions.runBlocking(auto.claw.openClaw);
//                    }
                    Actions.runBlocking(auto.depo.armIn);
                    Actions.runBlocking(auto.pitch.pitchIn);
//                    Actions.runBlocking(auto.lift.toZero);
                    auto.follower.followPath(auto.element3);
                    Actions.runBlocking(auto.extend.extendExtendo);
                    Actions.runBlocking(auto.intake.pivotGround);
                    Actions.runBlocking(auto.intake.spinIn);
                    setPathState(8);
                }
                break;
            case 8:
                if(!auto.follower.isBusy()) {
                    auto.follower.followPath(auto.score3);
                    Actions.runBlocking(auto.intake.spinStop);
                    Actions.runBlocking(auto.intake.pivotTransfer);
                    Actions.runBlocking(auto.extend.retractExtendo);
                    setPathState(9);
                }
                break;
            case 9:
                if(!auto.follower.isBusy()) {
                    auto.follower.followPath(auto.park);
                    Actions.runBlocking(auto.claw.closeClaw);
                    Actions.runBlocking(auto.depo.armOut);
                    Actions.runBlocking(auto.pitch.pitchOut);
                    setPathState(10);
                }
                break;
            case 10:
                if(!auto.follower.isBusy()) {
                    auto.follower.breakFollowing();
                    setPathState(11);
                }
                break;
        }
    }

    public void setPathState(int x) {
        pathState = x;
    }
}