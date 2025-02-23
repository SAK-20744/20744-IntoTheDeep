package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.config.FieldConstants.blueObservationStartPose;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.FieldConstants.RobotStart;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.FieldConstants.*;



import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.DiffySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.RailSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Deposit.RollSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

public class AmpyAuto {
//
//    private RobotStart startLocation;

    public ClawSubsystem claw;
    public ClawSubsystem.ClawState clawState;
    public LiftSubsystem lift;
    public ExtendSubsystem extend;
    public IntakeSubsystem intake;
    public IntakeSubsystem.IntakeSpinState intakeSpinState;
    public IntakeSubsystem.IntakePivotState intakePivotState;
    public RailSubsystem rail;
    public RailSubsystem.railState railState;
    public RollSubsystem roll;
    public RollSubsystem.RollState rollState;
    public DiffySubsystem diffy;
    public DiffySubsystem.diffyState diffyState;

    public boolean actionBusy = false;
    public Follower follower;
    public Telemetry telemetry;

    public PathChain pushSamples, preload,specimen1, specimen2, specimen3, specimen4, grab1, grab2, grab3, grab4, park;
    public Pose startPose, preloadPose, sample1Pose, sample1ControlPose, sample2Pose, sample2ControlPose, sample3Pose, sample3ControlPose, sampleScorePose, parkControlPose, parkPose, grab1Pose, specimen1Pose, grab2Pose, specimen2Pose, grab3Pose, specimen3Pose, grab4Pose, specimen4Pose, specimenSetPose;

    public Timer intakeTimer = new Timer(), retractTimer = new Timer(), bucketTimer = new Timer();

    public AmpyAuto(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, boolean isBlue, boolean isBucket) {
        claw = new ClawSubsystem(hardwareMap, clawState);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        extend = new ExtendSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, intakeSpinState, intakePivotState);
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
        lift.updatePIDF();
        extend.updatePIDF();

//        chamber();

        telemetryUpdate();
    }


    public boolean actionNotBusy() {
        return !actionBusy;
    }

    public void createPoses() {
        startPose = blueObservationStartPose;
        preloadPose = blueObservationPreloadPose;
        specimenSetPose = blueObservationSpecimenSetPose;
        grab1Pose = blueObservationSpecimenPickupPose;
        grab2Pose = blueObservationSpecimenPickup2Pose;
        grab3Pose = blueObservationSpecimenPickup3Pose;
        grab4Pose = blueObservationSpecimenPickup4Pose;
        specimen1Pose = blueObservationSpecimen1Pose;
        specimen2Pose = blueObservationSpecimen2Pose;
        specimen3Pose = blueObservationSpecimen3Pose;
        specimen4Pose = blueObservationSpecimen4Pose;
        parkPose = blueObservationParkPose;

        follower.setStartingPose(startPose);
    }

    public void buildPaths() {

        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        pushSamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadPose), new Point(15, 36, Point.CARTESIAN), new Point(61, 36.25, Point.CARTESIAN), new Point(59, 26.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(new Point(59.000, 26.000, Point.CARTESIAN), new Point(28, 26.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .addPath(new BezierCurve(new Point(28, 26.000, Point.CARTESIAN), new Point(52.000, 30.000, Point.CARTESIAN), new Point(58.000, 16.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .addPath(new BezierLine(new Point(58.000, 16.000, Point.CARTESIAN),new Point(28, 16.000, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .addPath(new BezierCurve(new Point(28, 16.000, Point.CARTESIAN), new Point(56.000, 16.000, Point.CARTESIAN), new Point(56.000, 10, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .addPath(new BezierLine(new Point(56.000, 10, Point.CARTESIAN), new Point(28, 10, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                //.setZeroPowerAccelerationMultiplier(0.5)
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(28,10,Point.CARTESIAN), new Point(grab1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), grab1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        specimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1Pose), new Point(specimen1Pose.getX()-10, specimen1Pose.getY(), Point.CARTESIAN), new Point(specimen1Pose)))
                .setLinearHeadingInterpolation(grab1Pose.getHeading(), specimen1Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pose), new Point(grab2Pose)))
                .setLinearHeadingInterpolation(specimen1Pose.getHeading(), grab2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        specimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab2Pose), new Point(specimen2Pose.getX() - 10, specimen2Pose.getY(), Point.CARTESIAN),new Point(specimen2Pose)))
                .setLinearHeadingInterpolation(grab2Pose.getHeading(), specimen2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pose), new Point(grab3Pose)))
                .setLinearHeadingInterpolation(specimen2Pose.getHeading(), grab3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        specimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab3Pose), new Point(specimen3Pose.getX() - 10, specimen3Pose.getY(), Point.CARTESIAN),new Point(specimen3Pose)))
                .setLinearHeadingInterpolation(grab3Pose.getHeading(), specimen3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pose), new Point(grab4Pose)))
                .setLinearHeadingInterpolation(specimen3Pose.getHeading(), grab4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        specimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab4Pose), new Point(specimen4Pose.getX() - 10, specimen3Pose.getY(), Point.CARTESIAN),new Point(specimen3Pose)))
                .setLinearHeadingInterpolation(grab4Pose.getHeading(), specimen4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen4Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(specimen4Pose.getHeading(), parkPose.getHeading())
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