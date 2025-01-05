package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Auto;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.SequentialAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

//@Disabled
@Autonomous(name="BlueBucket", group="B")
public class BlueBucket extends OpMode {
    public int pathState;
    public Auto auto;
    public Timer pathTimer = new Timer();


    @Override
    public void init() {
        auto = new Auto(hardwareMap, telemetry, new Follower(hardwareMap), true, true);
        Actions.runBlocking(auto.extend.retractExtendo);
        Actions.runBlocking(auto.intake.pivotTransfer);
        Actions.runBlocking(auto.intake.openDoor);
        Actions.runBlocking(auto.depo.armIn);
        Actions.runBlocking(auto.pitch.pitchIn);
    }

    @Override
    public void init_loop() {
        if(gamepad2.right_bumper)
            Actions.runBlocking(auto.claw.openClaw);
        else
            Actions.runBlocking(auto.claw.closeClaw);
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

//        telemetry.addData("state", pathState);
//        telemetry.addData("x", auto.follower.getPose().getX());
//        telemetry.addData("y", auto.follower.getPose().getY());
//        telemetry.addData("h", auto.follower.getPose().getHeading());
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0:
                auto.startBucket();
                auto.follower.followPath(auto.preload, true);
                setPathState(1);
                break;
            case 1:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.element1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startIntake();
                    setPathState(3);
                }
                break;
            case 3:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.score1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startBucket();
                    setPathState(5);
                }
                break;
            case 5:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.element2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startIntake();
                    setPathState(7);
                }
                break;
            case 7:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.score2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startBucket();
                    setPathState(9);
                }
                break;
            case 9:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.element3);
                    setPathState(10);
                }
                break;
            case 10:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startIntake();
                    setPathState(11);
                }
                break;
            case 11:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.score3, true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startBucket();
                    setPathState(13);
                }
                break;
            case 13:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.park, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(auto.actionNotBusy()) {
                    auto.pitch.setPitchIn();
                    auto.depo.setArmOut();
                    Actions.runBlocking(auto.lift.toPark);
                    setPathState(15);
                }
                break;
            case 15:
                if(!auto.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}