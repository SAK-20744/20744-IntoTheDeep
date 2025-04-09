package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationParkPose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationPreloadPose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimen1Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimen2Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimen3Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimen4Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimenPickup2Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimenPickup3Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimenPickup4Pose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimenPickupPose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationSpecimenSetPose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationStartPose;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specAvoidPickup;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specDrop1;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specDrop2;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specDrop3;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specIntake1;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specIntake2;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specIntake3;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specIntakeAvoid;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specPark;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specPickup;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specScoring;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.specStart;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.grabtime;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.li;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.DiffySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.RailSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.RollSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

public class IntakingSpecAuto {
//
//    private RobotStart startLocation;

    public ClawSubsystem claw;
    public ClawSubsystem.ClawState clawState;
    public NewIntakeSubsystem.COLOR colordetected;
    public LiftSubsystem lift;
    public ExtendSubsystem extend;
    public NewIntakeSubsystem intake;
    public NewIntakeSubsystem.IntakeSpinState intakeSpinState;
    public NewIntakeSubsystem.IntakePivotState intakePivotState;
    public RailSubsystem rail;
    public RailSubsystem.railState railState;
    public RollSubsystem roll;
    public RollSubsystem.RollState rollState;
    public DiffySubsystem diffy;
    public DiffySubsystem.diffyState diffyState;

    public boolean actionBusy = false;
    public Follower follower;
    public Telemetry telemetry;

    public PathChain preload, pickup, score, park, intake1, intake2, intake3, drop1, drop2, drop3, specialPickup;
    public Pose startPose, preloadPose, sample1Pose, sample1ControlPose, sample2Pose, sample2ControlPose, sample3Pose, sample3ControlPose, sampleScorePose, parkControlPose, parkPose, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, grab4Pose, specimen4Pose, specimenSetPose;

    public int grabState, releaseState, retractState, chamberState, wallState, autointakeState, ejectState = -1;
    public Timer grabTimer = new Timer(), ejectTimer = new Timer(), releaseTimer = new Timer(), autoIntakeTimer = new Timer();

    public IntakingSpecAuto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
        claw = new ClawSubsystem(hardwareMap, clawState);
        lift = new LiftSubsystem(hardwareMap, telemetry, true);
        extend = new ExtendSubsystem(hardwareMap, telemetry);
        intake = new NewIntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState, colordetected, telemetry);
        diffy = new DiffySubsystem(hardwareMap, diffyState);
        rail = new RailSubsystem(hardwareMap, railState);
        roll = new RollSubsystem(hardwareMap, rollState);

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
        roll.init();
        rail.init();
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
        rail.start();
        roll.start();
    }

    public void update() {
        follower.update();
        lift.updatePIDFSpecAuto();
        extend.updatePIDF();
        intake.updateCOLOR();

        grab();
        chamber();
        release();
        wall();
        autointake();
        eject();
        retract();

        telemetryUpdate();
    }

    public void setGrabState(int x) {
        grabState = x;
    }

    public void setReleaseState(int x) {
        releaseState = x;
    }

    public void setChamberState(int x) {
        chamberState = x;
    }

    public void setWallState(int x) {
        wallState = x;
    }

    public void setRetractState(int x) {
        retractState = x;
    }

    public void setAutoIntakeState(int x) {
        autointakeState = x;
    }

    public void setEjectState(int x) {
        ejectState = x;
    }

    public void startGrab() {
        if (actionNotBusy()) {
            setGrabState(1);
        }
    }

    public void startRelease() {
        if (actionNotBusy()) {
            setReleaseState(1);
        }
    }

    public void startWall() {
        if (actionNotBusy()) {
            setWallState(1);
        }
    }

    public void startEject() {
        if (actionNotBusy()) {
            setEjectState(1);
        }
    }

    public void startChamber() {
        if (actionNotBusy()) {
            setChamberState(1);
        }
    }

    public void startRetract() {
        if (actionNotBusy()) {
            setRetractState(1);
        }
    }

    public void startAutoIntake() {
        if (actionNotBusy()) {
            setAutoIntakeState(1);
        }
    }



    public void grab() {
        switch (grabState) {
            case 1:
                actionBusy = true;
                grabTimer.resetTimer();
                claw.closeClaw();
                setGrabState(2);
                break;
            case 2:
                if(grabTimer.getElapsedTimeSeconds() > grabtime)
                {
                    actionBusy = false;
                    setGrabState(-1);
                }
                break;
            }
    }

    public void release() {
        switch (releaseState) {

            case 1:
                actionBusy = true;
                claw.openClaw();
                setReleaseState(2);
                break;
            case 2:
                if(releaseTimer.getElapsedTime() > .3)
                {
                    actionBusy = false;
                    setReleaseState(-1);
                }
                break;
        }
    }

    public void chamber() {
        switch (chamberState) {
            case 1:
                actionBusy = true;
                lift.toSpecAuto();
                rail.clipRail();
                diffy.clipdiffy();
                roll.transferRoll();
                setChamberState(2);
                break;
            case 2:
                if(lift.isAtTarget()) {
                    actionBusy = false;
                    setChamberState(-1);
                }
                break;
        }
    }

    public void wall() {
        switch (wallState) {
            case 1:
                actionBusy = true;
                lift.toZero();
                extend.retract();
                rail.wallRail();
                diffy.walldiffy();
                roll.depoRoll();
                setWallState(2);
                break;
            case 2:
                if(lift.isAtTarget()) {
                    actionBusy = false;
                    setWallState(-1);
                }
                break;
        }
    }

    public void eject() {
        switch (ejectState) {
            case 1:
                actionBusy = true;
                extend.outtake();
                intake.spinOut();
                setEjectState(2);
                break;
            case 2:
                if(extend.isAtTarget() && intake.getColorDetected() == NewIntakeSubsystem.COLOR.NONE)  {
                    actionBusy = false;
                    setEjectState(-1);
                }
                break;
        }
    }

    public void autointake() {
        switch (autointakeState) {
            case 1:
                actionBusy = true;

                autoIntakeTimer.resetTimer();

                lift.toZero();
                rail.transferRail();
                roll.transferRoll();
                diffy.transferdiffy();
                claw.openClaw();

                extend.extend();
                intake.pivotGround();
                intake.spinIn();

                setAutoIntakeState(2);
            case 2:
                if (intake.getColorDetected() != NewIntakeSubsystem.COLOR.NONE && autoIntakeTimer.getElapsedTimeSeconds() > 0.7) {
                    intake.pivotTransfer();
                    intake.spinStop();
                    actionBusy = false;
                    setAutoIntakeState(-1);
                }
                else if(autoIntakeTimer.getElapsedTimeSeconds() > 0.5) {
                    extend.retract();
                    autoIntakeTimer.resetTimer();
                    setAutoIntakeState(3);
                }
            case 3:
                if(autoIntakeTimer.getElapsedTimeSeconds() > 0.27) {
                    extend.toAuto();
                    intake.pivotTransfer();
                    intake.spinStop();
                    actionBusy = false;
                    setAutoIntakeState(-1);
                }
                break;
        }

    }


    public void retract() {
        switch (retractState) {
            case 1:
                actionBusy = true;
                extend.retract();
                rail.transferRail();
                roll.transferRoll();
                diffy.transferdiffy();
                setRetractState(2);
            case 2:
                if (extend.isAtTarget()) {
                    actionBusy = false;
                    setRetractState(-1);
                }
                break;
        }
    }

    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public void createPoses() {
        startPose = specStart;
        preloadPose = specScoring;
//        specimenSetPose = blueObservationSpecimenSetPose;
//        grab1Pose = blueObservationSpecimenPickupPose;
//        grab2Pose = blueObservationSpecimenPickup2Pose;
//        grab3Pose = blueObservationSpecimenPickup3Pose;
//        grab4Pose = blueObservationSpecimenPickup4Pose;
//        specimen1Pose = blueObservationSpecimen1Pose;
//        specimen2Pose = blueObservationSpecimen2Pose;
//        specimen3Pose = blueObservationSpecimen3Pose;
//        specimen4Pose = blueObservationSpecimen4Pose;
//        parkPose = blueObservationParkPose;

        follower.setStartingPose(startPose);
    }

    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();

        pickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specScoring),  new Point(specPickup)))
                .setLinearHeadingInterpolation(specScoring.getHeading(), specPickup.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(350)
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specPickup), new Point(specScoring)))
                .setLinearHeadingInterpolation(specPickup.getHeading(), specScoring.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specScoring), new Point(specIntakeAvoid), new Point(specIntake1)))
                .setLinearHeadingInterpolation(specPickup.getHeading(), specIntakeAvoid.getHeading(), specIntake1.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        drop1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specIntake1), new Point(specDrop1)))
                .setLinearHeadingInterpolation(specIntake1.getHeading(), specDrop1.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specDrop1), new Point(specIntake2)))
                .setLinearHeadingInterpolation(specDrop1.getHeading(), specIntake2.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        drop2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specIntake2), new Point(specDrop2)))
                .setLinearHeadingInterpolation(specIntake2.getHeading(), specDrop2.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specDrop2), new Point(specIntake3)))
                .setLinearHeadingInterpolation(specDrop2.getHeading(), specIntake3.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        drop3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specIntake3), new Point(specDrop3)))
                .setLinearHeadingInterpolation(specIntake3.getHeading(), specDrop3.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        specialPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specDrop3), new Point(specAvoidPickup), new Point(specPickup)))
                .setLinearHeadingInterpolation(specDrop3.getHeading(), specAvoidPickup.getHeading(), specIntake2.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(350)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specScoring), new Point(specPark)))
                .setLinearHeadingInterpolation(specScoring.getHeading(), specPark.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .build();
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