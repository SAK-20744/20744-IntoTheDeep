package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.FieldConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.DepoArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.PitchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Action;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.ParallelAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.SequentialAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.SleepAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

public class Auto {

    private RobotStart startLocation;

    public ClawSubsystem claw;
    public ClawSubsystem.ClawState clawState;
    public LiftSubsystem lift;
    public PitchSubsystem pitch;
    public PitchSubsystem.PitchState pitchState;
    public DepoArmSubsystem depo;
    public DepoArmSubsystem.DepoState depoState;
    public ExtendSubsystem extend;
    public ExtendSubsystem.ExtendoState extendoState;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.IntakePivotState intakePivotState;
    public IntakeSubsystem.DoorState doorState;

    public boolean actionBusy = false;

    public Follower follower;
    public Telemetry telemetry;
    public boolean liftPIDF = true;
    public double liftManual = 0;

    public int bucketState, intakeState, retractState = -1;

    public Timer intakeTimer = new Timer(), retractTimer = new Timer(), bucketTimer = new Timer();

    public RunAction transfer;
    public Path preload, element1, score1, element2, score2, element3, score3, park;
    private Pose startPose, preloadPose, element1Pose, element1ControlPose, element2Pose, element2ControlPose, element3Pose, element3ControlPose, elementScorePose, parkControlPose, parkPose;

    public Auto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
        claw = new ClawSubsystem(hardwareMap, clawState);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        pitch = new PitchSubsystem(hardwareMap, pitchState);
        depo = new DepoArmSubsystem(hardwareMap, depoState);
        extend = new ExtendSubsystem(hardwareMap, extendoState);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState, doorState);

        this.follower = follower;
        this.telemetry = telemetry;

        startLocation = isBlue ? (isBucket ? RobotStart.BLUE_BUCKET : RobotStart.BLUE_OBSERVATION) : (isBucket ? RobotStart.RED_BUCKET : RobotStart.RED_OBSERVATION);

        createPoses();
        buildPaths();

    }

    public void init() {
        claw.init();
        lift.init();
        pitch.init();
        depo.init();
        extend.init();
        intake.init();
    }

    public void init_loop() {}

    public void start() {
        claw.start();
        lift.start();
        pitch.start();
        depo.start();
        extend.start();
        intake.start();
    }

    public void update() {
        follower.update();

        if(!liftPIDF)
            lift.manual(liftManual);
        else
            lift.updatePIDF();

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
                intake.pivotGround();
                intake.doorClosed();
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
//                claw.openClaw();
//                intake.doorClosed();
                extend.retract();
                bucketTimer.resetTimer();
                setBucketState(2);
                break;
            case 2:
                if (bucketTimer.getElapsedTimeSeconds() > 1) {
                    claw.closeClaw();
                    intake.doorOpen();
                    bucketTimer.resetTimer();
                    setBucketState(3);
                }
                break;
            case 3:
                if (bucketTimer.getElapsedTimeSeconds() > 0.4) {
                    lift.toHighBucket();
                    setBucketState(4);
                }
                break;
            case 4:
                if (lift.isAtTarget()) {
//                    bucketTimer.resetTimer();
                    depo.setArmOut();
                    pitch.setPitchOut();
                    setBucketState(5);
                }
            case 5:
                if (bucketTimer.getElapsedTimeSeconds() > 2) {
//                    bucketTimer.resetTimer();
                    claw.openClaw();
                    setBucketState(6);
                }
                break;
            case 6:
                if (bucketTimer.getElapsedTimeSeconds() > 2.4) {
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
                intake.doorClosed();
                extend.retract();
                depo.setArmIn();
                pitch.setPitchIn();
                retractTimer.resetTimer();
                setRetractState(2);
                break;
            case 2:
                if (retractTimer.getElapsedTimeSeconds() > 0.4) {
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
        switch (startLocation) {
            case BLUE_BUCKET:
                startPose = blueBucketStartPose;
                preloadPose = blueBucketPreloadPose;
//                element1ControlPose = blueBucketLeftSampleControlPose;
                element1Pose = blueBucketLeftSamplePose;
//                element2ControlPose = blueBucketMidSampleControlPose;
                element2Pose = blueBucketMidSamplePose;
//                element3ControlPose = blueBucketRightSampleControlPose;
                element3Pose = blueBucketRightSamplePose;
                elementScorePose = blueBucketScorePose;
                parkControlPose = blueBucketParkControlPose;
                parkPose = blueBucketParkPose;
                break;

            case BLUE_OBSERVATION:
                startPose = blueObservationStartPose;
                //parkPose = blueObservationPark;
                break;

            case RED_BUCKET:
                startPose = redBucketStartPose;
                //parkPose = redBucketPark;
                break;

            case RED_OBSERVATION:
                startPose = redObservationStartPose;
                //parkPose = redObservationPark;
                break;
        }

    }

    public void buildPaths() {
        follower.setStartingPose(startPose);

        preload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        preload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading());

        element1 = new Path(new BezierCurve(new Point(preloadPose), new Point(element1Pose)));
        element1.setLinearHeadingInterpolation(preloadPose.getHeading(), element1Pose.getHeading());

        score1 = new Path(new BezierLine(new Point(element1Pose), new Point(elementScorePose)));
        score1.setLinearHeadingInterpolation(element1Pose.getHeading(), elementScorePose.getHeading());

        element2 = new Path(new BezierCurve(new Point(element1Pose), new Point(element2Pose)));
        element2.setLinearHeadingInterpolation(element1Pose.getHeading(), element2Pose.getHeading(), 0.5);

        score2 = new Path(new BezierLine(new Point(element2Pose), new Point(elementScorePose)));
        score2.setLinearHeadingInterpolation(element2Pose.getHeading(), elementScorePose.getHeading());

        element3 = new Path(new BezierCurve(new Point(element2Pose), new Point(element3Pose)));
        element3.setLinearHeadingInterpolation(element2Pose.getHeading(), element3Pose.getHeading(), 0.5);

        score3 = new Path(new BezierLine(new Point(element3Pose), new Point(elementScorePose)));
        score3.setLinearHeadingInterpolation(element3Pose.getHeading(), elementScorePose.getHeading());

        park = new Path(new BezierCurve(new Point(elementScorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(elementScorePose.getHeading(), parkPose.getHeading(), 0.7);
    }

    public boolean notBusy() {
        return (!follower.isBusy() && actionNotBusy());
    }

    public void telemetryUpdate() {
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("lift: ", lift.getPos());
        telemetry.addData("liftAtTarget?: ", lift.isAtTarget());
        telemetry.addData("Bucket State: ", bucketState);
        telemetry.addData("Retract State: ", retractState);
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Action Busy?: ", actionBusy);
        telemetry.update();
    }
}