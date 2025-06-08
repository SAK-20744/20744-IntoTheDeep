package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.IntakingSpecAuto;
import org.firstinspires.ftc.teamcode.config.ObservationAuto;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;


@Autonomous(name="Specimen", group="A")
public class Specimen extends OpMode {
    public int pathState;
    public IntakingSpecAuto auto;

    public Timer pathTimer = new Timer();

    @Override
    public void init() {
        auto = new IntakingSpecAuto(hardwareMap, telemetry, new Follower(hardwareMap));
        Actions.runBlocking(auto.extend.retractExtendo);
        Actions.runBlocking(auto.intake.pivotTransfer);
        Actions.runBlocking(auto.diffy.diffyMoveClipping);
        Actions.runBlocking(auto.roll.transferRoll);
        Actions.runBlocking(auto.rail.railMoveTransfering);
    }

    public void init_loop() {
        if(gamepad2.right_bumper)
            Actions.runBlocking(auto.claw.openClaw);
        else
            Actions.runBlocking(auto.claw.closeClaw);
        auto.init_loop();
    }

    @Override
    public void start() {
        auto.start();
        setPathState(0);
    }

    @Override
    public void loop() {
        telemetry.addData("State: ", pathState);
        telemetry.addData("Path Timer: ", pathTimer.getElapsedTimeSeconds());
        auto.update();
        pathUpdate();

        telemetry.update();
    }

    public void pathUpdate() {
        switch (pathState) {
            case 0: //Runs to the position of the preload and holds it's point at 0.5 power
                auto.startChamber();
                auto.follower.followPath(auto.preload, true);
                setPathState(1);
                break;
            case 1: //Runs to the position of the preload and holds it's point at 0.5 power
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRelease();
                    setPathState(2);
                }
                break;
            case 2: //Once Chamber State Machine finishes, begins Pathchain to push elements to the submersible
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startWall();
                    auto.follower.followPath(auto.pickup, true);
                    setPathState(3);
                }
                break;
            case 3: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startGrab();
                    setPathState(4);
                }
                break;
            case 4: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startChamber();
                    auto.follower.followPath(auto.score, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRelease();
                    setPathState(6);
                }
                break;
            case 6: //Once the Specimen State Machine finishes, begins the grab path
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.intake1, true);
                    setPathState(7);
                }
                break;
            case 7: //Runs to the position of the preload and holds it's point at 0.5 power
                if((!auto.follower.isBusy() && auto.actionNotBusy())) {
                    auto.startAutoIntake();
                    setPathState(8);
                }
                break;
            case 8: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.followPath(auto.drop1);
                    setPathState(10);
                }
                break;
            case 9:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startEject();
                    setPathState(10);
                }
                break;
            case 10: //Once the Specimen State Machine finishes, begins the grab path
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.intake2, true);
                    setPathState(11);
                }
                break;
            case 11: //Runs to the position of the preload and holds it's point at 0.5 power
                if((!auto.follower.isBusy() && auto.actionNotBusy())) {
                    auto.startAutoIntake();
                    setPathState(12);
                }
                break;
            case 12: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.followPath(auto.drop2, false);
                    setPathState(13);
                }
                break;
            case 13:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startEject();
                    setPathState(14);
                }
                break;
            case 14: //Once the Specimen State Machine finishes, begins the grab path
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startRetract();
                    auto.follower.followPath(auto.intake3, true);
                    setPathState(15);
                }
                break;
            case 15: //Runs to the position of the preload and holds it's point at 0.5 power
                if((!auto.follower.isBusy() && auto.actionNotBusy())) {
                    auto.startAutoIntake();
                    setPathState(16);
                }
                break;
            case 16: //Once the Pathchain finishes, begins the Specimen State Machine
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.followPath(auto.drop3);
                    setPathState(17);
                }
                break;
            case 17:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startEject();
                    setPathState(18);
                }
                break;
            case 18: //Once the Specimen State Machine finishes, begins the grab path
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startWall();
                    auto.follower.followPath(auto.specialPickup, true);
                    setPathState(19);
                }
                break;
            case 19:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startGrab();
                    setPathState(20);
                }
                break;
            case 20: //Once the Specimen State Machine finishes, begins the grab path
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startChamber();
                    auto.follower.followPath(auto.score, true);
                    setPathState(21);
                }
                break;
            case 21: //Runs to the position of the preload and holds it's point at 0.5 power
                if((!auto.follower.isBusy() && auto.actionNotBusy()) || pathTimer.getElapsedTimeSeconds() > 2.7) {
                    auto.startRelease();
                    setPathState(22);
                }
                break;
            case 22: //Once the Pathchain finishes, begins the Specimen State Machine
                if(auto.actionNotBusy()) {
                    auto.startWall();
                    auto.follower.followPath(auto.pickup, true);
                    setPathState(23);
                }
                break;
//            case 17:
//                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
//                    auto.startGrab();
//                    setPathState(18);
//                }
//                break;
//            case 18: //Once the Specimen State Machine finishes, begins the grab path
//                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
//                    auto.startChamber();
//                    auto.follower.followPath(auto.specimen4, true);
//                    setPathState(19);
//                }
//                break;
//            case 19: //Runs to the position of the preload and holds it's point at 0.5 power
//                if((!auto.follower.isBusy() && auto.actionNotBusy()) || pathTimer.getElapsedTimeSeconds() > 2.7) {
//                    auto.startRelease();
//                    setPathState(20);
//                }
//                break;
//
//            case 20: //Park and End the autonomous
//                if(auto.actionNotBusy()) {
//                    auto.follower.setMaxPower(1);
//                    auto.follower.followPath(auto.park, true);
//                    setPathState(21);
//                }
//                break;
//            case 21:
//                if(auto.actionNotBusy()) {
//                    auto.lift.toZero();
//                    auto.diffy.scoringdiffy();
//                    setPathState(-1);
//                }
//                break;
        }
    }

    public void setPathState(int x) {
        pathState = x;
        pathTimer.resetTimer();
    }
}