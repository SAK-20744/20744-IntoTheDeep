
package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.FieldConstantsCRI.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.DiffySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RailSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Deposit.RollSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

public class farBucketAuto {
//
//    private RobotStart startLocation;

    public ClawSubsystem claw;
    public ClawSubsystem.ClawState clawState;
    public LiftSubsystem lift;
    public ExtendSubsystem extend;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.IntakePivotState intakePivotState;
//    public RailSubsystem rail;
//    public RailSubsystem.railState railState;
//    public RollSubsystem roll;
//    public RollSubsystem.RollState rollState;
    public DiffySubsystem diffy;
    public DiffySubsystem.diffyState diffyState;

    public boolean actionBusy = false;
    public Follower follower;
    public Telemetry telemetry;

    public Path commonBlock, commonPreScore, oppositeControl, oppositeScore;
//            preload, element1, score1, element2, score2, element3, score3, park, parkObs;
    //    public PathChain park;
    public Pose startPose, CommonBlock, CommonPreScore, OppositeControl, OppositeScore;
//            startPose, preloadPose, element1Pose, element1ControlPose, element2Pose, element2ControlPose, element3Pose, element3ControlPose, elementScorePose, parkControlPose, parkPose, parkPoseObs, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, grab4Pose, specimen4Pose, specimenSetPose;
    //  public Pose startPose2, preloadPose2, element1Pose2, element2Pose2, element3Pose2, elementScorePose2, parkControlPose2, parkPose2, ;


    public int bucketState, intakeState, autointakeState, retractState = -1;
    public Timer intakeTimer = new Timer(), retractTimer = new Timer(), bucketTimer = new Timer();


    public farBucketAuto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
        claw = new ClawSubsystem(hardwareMap, clawState);
        lift = new LiftSubsystem(hardwareMap, telemetry, false);
        extend = new ExtendSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState);
        diffy = new DiffySubsystem(hardwareMap, diffyState);
//        rail = new RailSubsystem(hardwareMap, railState);
//        roll = new RollSubsystem(hardwareMap, rollState);

        this.follower = follower;
        this.telemetry = telemetry;

//        startLocation = isBlue ? (isBucket ? RobotStart.BLUE_BUCKET : RobotStart.BLUE_OBSERVATION) : (isBucket ? RobotStart.RED_BUCKET : RobotStart.RED_OBSERVATION);

        createPoses();
        buildPaths();

    }

    public void init() {

        claw.init();
        lift.init();
        extend.init();
        intake.init();
//        roll.init();
//        rail.init();
        diffy.init();
        telemetryUpdate();


        follower.setStartingPose(startPose);
    }

    public void init_loop() {
        lift.init_loop();
        extend.init_loop();
    }

    public void start() {
        claw.start();
        lift.start();
        extend.start();
        intake.start();
        diffy.start();
//        rail.start();
//        roll.start();
    }

    public void update() {
        follower.update();
        lift.updatePIDF();
        extend.updatePIDF();

        intake();
        bucket();
        retract();

        telemetryUpdate();
    }


    public void setBucketState(int x) {
        bucketState = x;
    }

    public void setRetractState(int x) {
        retractState = x;
    }

    public void setIntakeState(int x) {
        intakeState = x;
    }

    public void startBucket() {
        if (actionNotBusy()) {
            setBucketState(1);
        }
    }

    public void startRetract() {
        if (actionNotBusy()) {
            setRetractState(1);
        }
    }

    public void startIntake() {
        if (actionNotBusy()) {
            setIntakeState(1);
        }
    }

    public boolean actionNotBusy() {
        return !actionBusy;
    }


    public void intake() {
        switch (intakeState) {
            case 1:
                actionBusy = true;
//                roll.transferRoll();
                intake.pivotGround();
                intake.spinIn();
                claw.openClaw();
                extend.extend();
                intakeTimer.resetTimer();
                setIntakeState(2);
                break;
            case 2:
                if (intakeTimer.getElapsedTimeSeconds() > 1.5) {
                    actionBusy = false;
                    setIntakeState(-1);
                }
                break;

        }
    }

    public void bucket() {
        switch (bucketState) {
            case 1:
                actionBusy = true;
                intake.pivotTransfer();
                intake.spinStop();
                extend.retract();
                bucketTimer.resetTimer();
                setBucketState(2);
                break;
            case 2:
                if (bucketTimer.getElapsedTimeSeconds() > 0.85) {
                    claw.closeClaw();
                    bucketTimer.resetTimer();
                    setBucketState(3);
                }
                break;
            case 3:
                if (bucketTimer.getElapsedTimeSeconds() > 0.35) {
                    lift.toHighBucket();
//                    rail.clipRail();
                    diffy.autodiffy();
//                    roll.depoRoll();
                    setBucketState(4);
                }
                break;
            case 4:
                if (lift.isAtMax()) {
                    diffy.scoringdiffy();
//                    rail.scoringRail();
                    setBucketState(5);
                }
            case 5:
                if (bucketTimer.getElapsedTimeSeconds() > 2) {
                    bucketTimer.resetTimer();
                    claw.openClaw();
                    setBucketState(6);
                }
                break;
            case 6:
                if (bucketTimer.getElapsedTimeSeconds() > 0.3) {
                    actionBusy = false;
                    setBucketState(-1);
                }
                break;

        }
    }

    public void retract() {
        switch (retractState) {
            case 1:
                actionBusy = true;
                extend.retract();
//                rail.transferRail();
//                roll.transferRoll();
                diffy.transferdiffy();
                retractTimer.resetTimer();
                setRetractState(2);
                break;
            case 2:
                if (retractTimer.getElapsedTimeSeconds() > 1.2) {
                    lift.toZero();
                    claw.openClaw();
                    retractTimer.resetTimer();
                    setRetractState(3);
                }
                break;
            case 3:
                if (lift.isAtTarget()) {
                    actionBusy = false;
                    setRetractState(-1);
                }
                break;
        }
    }




    public void createPoses() {
        startPose = blueBucketStartPose;
//        startPose2 = redBucketStartPose;
//        preloadPose = blueBucketPreloadPose;
        //       preloadPose2 = redBucketPreloadPose;
//        sample1ControlPose = blueBucketLeftSampleControlPose;
//        element1Pose = blueBucketLeftSamplePose;
        //       element1Pose2 = redBucketLeftSamplePose;
//        sample2ControlPose = blueBucketMidSampleControlPose;
//        element2Pose = blueBucketMidSamplePose;
        //       element2Pose2 = redBucketMidSamplePose;
//        sample3ControlPose = blueBucketRightSampleControlPose;
//        element3Pose = blueBucketRightSamplePose;
        //       element3Pose2 = redBucketRightSamplePose;
//        elementScorePose = blueBucketScorePose;
//        elementScorePose2 = redBucketScorePose;
//        parkControlPose = blueBucketParkControlPose;
        //       parkControlPose2 = redBucketParkControlPose;
//        parkPose = blueBucketParkPose;
        //       parkPose2 = redBucketParkPose;
//        parkPoseObs = blueBucketObsPark;
        CommonBlock = commonAreaBlockPose;
        CommonPreScore = commonAreaPreScorePose;
        OppositeControl = oppositeAreaControlBucketScorePose;
        OppositeScore = oppositeAreaBucketScorePose;


        follower.setStartingPose(startPose);
        //       follower.setStartingPose(startPose2);
    }

    public void buildPaths() {

        commonBlock = new Path(new BezierCurve(new Point(startPose), new Point(commonAreaBlockPose)));
        commonBlock.setLinearHeadingInterpolation(startPose.getHeading(), commonAreaBlockPose.getHeading());

        //     preload = new Path(new BezierLine(new Point(startPose2), new Point(preloadPose2)));
        //   preload.setLinearHeadingInterpolation(startPose2.getHeading(), preloadPose2.getHeading());

//        element1 = new Path(new BezierCurve(new Point(preloadPose), new Point(element1Pose)));
//        element1.setLinearHeadingInterpolation(preloadPose.getHeading(), element1Pose.getHeading());

        //      element1 = new Path(new BezierCurve(new Point(preloadPose2), new Point(element1Pose2)));
        //       element1.setLinearHeadingInterpolation(preloadPose2.getHeading(), element1Pose2.getHeading());

//        score1 = new Path(new BezierLine(new Point(element1Pose), new Point(elementScorePose)));
//        score1.setLinearHeadingInterpolation(element1Pose.getHeading(), elementScorePose.getHeading());

        //      score1 = new Path(new BezierLine(new Point(element1Pose2), new Point(elementScorePose2)));
        //      score1.setLinearHeadingInterpolation(element1Pose2.getHeading(), elementScorePose2.getHeading());

//        element2 = new Path(new BezierCurve(new Point(element1Pose), new Point(element2Pose)));
//        element2.setLinearHeadingInterpolation(element1Pose.getHeading(), element2Pose.getHeading(), 0.5);

        //     element2 = new Path(new BezierCurve(new Point(element1Pose2), new Point(element2Pose2)));
        //     element2.setLinearHeadingInterpolation(element1Pose2.getHeading(), element2Pose2.getHeading(), 0.5);

//        score2 = new Path(new BezierLine(new Point(element2Pose), new Point(elementScorePose)));
//        score2.setLinearHeadingInterpolation(element2Pose.getHeading(), elementScorePose.getHeading());

        //     score2 = new Path(new BezierLine(new Point(element2Pose2), new Point(elementScorePose2)));
        //    score2.setLinearHeadingInterpolation(element2Pose2.getHeading(), elementScorePose2.getHeading());


//        element3 = new Path(new BezierCurve(new Point(element2Pose), new Point(element3Pose)));
//        element3.setLinearHeadingInterpolation(element2Pose.getHeading(), element3Pose.getHeading(), 0.5);

        //     element3 = new Path(new BezierCurve(new Point(element2Pose2), new Point(element3Pose2)));
        //       element3.setLinearHeadingInterpolation(element2Pose2.getHeading(), element3Pose2.getHeading(), 0.5);
//
//        score3 = new Path(new BezierLine(new Point(element3Pose), new Point(elementScorePose)));
//        score3.setLinearHeadingInterpolation(element3Pose.getHeading(), elementScorePose.getHeading());

        //      score3 = new Path(new BezierLine(new Point(element3Pose2), new Point(elementScorePose2)));
        //      score3.setLinearHeadingInterpolation(element3Pose2.getHeading(), elementScorePose2.getHeading());

//        park = new Path(new BezierCurve(new Point(elementScorePose), new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(elementScorePose.getHeading(), parkPose.getHeading(), 0.7);

        //       park = new Path(new BezierCurve(new Point(elementScorePose2), new Point(parkControlPose2), new Point(parkPose2)));
        //     park.setLinearHeadingInterpolation(elementScorePose2.getHeading(), parkPose2.getHeading(), 0.7);

        commonPreScore = new Path(new BezierCurve(new Point(commonAreaBlockPose), new Point(commonAreaPreScorePose)));
        commonPreScore.setLinearHeadingInterpolation(commonAreaBlockPose.getHeading(), commonAreaPreScorePose.getHeading());

        oppositeControl = new Path(new BezierCurve(new Point(commonAreaPreScorePose), new Point(oppositeAreaControlBucketScorePose)));
        oppositeControl.setLinearHeadingInterpolation(commonAreaPreScorePose.getHeading(), oppositeAreaControlBucketScorePose.getHeading());

        oppositeScore = new Path(new BezierCurve(new Point(oppositeAreaControlBucketScorePose), new Point(oppositeAreaBucketScorePose)));
        oppositeScore.setLinearHeadingInterpolation(oppositeAreaControlBucketScorePose.getHeading(), oppositeAreaBucketScorePose.getHeading());
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

    public void telemetryUpdate() {
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("lift: ", lift.getPos());
        telemetry.addData("liftAtTarget?: ", lift.isAtTarget());
        telemetry.addData("extendo: ", extend.getPos());
        telemetry.addData("extendoAtTarget?: ", extend.isAtTarget());
//        telemetry.addData("Bucket State: ", bucketState);
//        telemetry.addData("Retract State: ", retractState);
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Action Busy?: ", actionBusy);
        telemetry.update();
    }
}