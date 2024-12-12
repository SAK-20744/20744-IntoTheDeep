package org.firstinspires.ftc.teamcode.opModes.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

@Autonomous(name = "0+3")
public class SamplePlus2Auto extends OpMode{

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, topLift, intake;
    private DigitalChannel liftLimit;

    //fix wristintaking, and dooropen
    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.34, V4B_OUT = 0.55, TRANSFER_CLOSED = 0.32, TRANSFER_OPEN = 0, EXTENDO_RETRACTED = 0.09, EXTENDO_EXTENDED = 0.5, WRIST_TRANSFERING = 0.12, WRIST_UP = 0.78, WRIST_INTAKING = 0.882, DOOR_OPEN = 0.5, DOOR_CLOSED = 0.965, PITCH_DEPO = 0.3, PITCH_TRANSFERING = 0.905;
    public static int LIFT_RETRACTED = 0,LIFT_MID_BASKET = -1450 ,LIFT_HIGH_BASKET = -2850;

    private int liftTarget = LIFT_RETRACTED;
    private int liftLiftedTarget = LIFT_HIGH_BASKET;
    private double leftV4BTarget = V4B_IN;
    private double transferTarget = TRANSFER_OPEN;
    private double lExtTarget = EXTENDO_RETRACTED;
    private double wristTarget = WRIST_TRANSFERING;
    private double doorTarget = DOOR_OPEN;
    private double intakePower = INTAKE_OFF;
    private double pitchTarget = PITCH_TRANSFERING;

    private Follower follower;
    private Pose startPose = new Pose(0,0, Math.toRadians(0));
    private Pose sample1Pos = new Pose(20,19, Math.toRadians(0));
    private Pose sample2Pos = new Pose(20,10.25, Math.toRadians(0));
    private Pose sample3Pos = new Pose(20,15, Math.toRadians(25));
    private Pose basketPos = new Pose(8.3,14, Math.toRadians(-45));
    private Pose basketPos1 = new Pose(8.3,14, Math.toRadians(-45));
    private Pose basketPos2 = new Pose(8.3,14, Math.toRadians(-45));
//    private Pose basketPos2 = new Pose(10.5,21.5, Math.toRadians(0));
    private Pose basketPos3 = new Pose(8.3,17, Math.toRadians(-45));
    private Pose avoidPos = new Pose(55, 5, Math.toRadians(-90));
    private Pose parkPos = new Pose(55, -5, Math.toRadians(-90));

    private Path toBasket, toSample1, score1, toSample2, score2,toSample3, score3, toAvoid, toPark;
    private Timer pathTimer;
//    private int pathState;

    public void buildPaths() {
        toBasket = new Path(new BezierLine(new Point(startPose), new Point(basketPos)));
        toBasket.setLinearHeadingInterpolation(startPose.getHeading(), basketPos.getHeading());
        toBasket.setPathEndTimeoutConstraint(2.5);

        toSample1 = new Path(new BezierLine(new Point(basketPos), new Point(sample1Pos)));
        toSample1.setLinearHeadingInterpolation(basketPos.getHeading(), sample1Pos.getHeading(), 0.5);
        toSample1.setPathEndTimeoutConstraint(3);

        score1 = new Path(new BezierLine(new Point(sample1Pos), new Point(basketPos1)));
        score1.setLinearHeadingInterpolation(sample1Pos.getHeading(), basketPos1.getHeading());
        score1.setPathEndTimeoutConstraint(5);

        toSample2 = new Path(new BezierLine(new Point(basketPos1), new Point(sample2Pos)));
        toSample2.setLinearHeadingInterpolation(basketPos1.getHeading(), sample2Pos.getHeading(), 0.25);
        toSample2.setPathEndTimeoutConstraint(3);

        score2 = new Path(new BezierLine(new Point(sample2Pos), new Point(basketPos2)));
        score2.setLinearHeadingInterpolation(sample2Pos.getHeading(), basketPos2.getHeading());
        score2.setPathEndTimeoutConstraint(5);

        toSample3 = new Path(new BezierLine(new Point(basketPos2), new Point(sample3Pos)));
        toSample3.setLinearHeadingInterpolation(basketPos2.getHeading(), sample3Pos.getHeading(), 0.25);
        toSample3.setPathEndTimeoutConstraint(3);

        score3 = new Path(new BezierLine(new Point(sample3Pos), new Point(basketPos3)));
        score3.setLinearHeadingInterpolation(sample3Pos.getHeading(), basketPos3.getHeading());
        score3.setPathEndTimeoutConstraint(5);

        toPark = new Path(new BezierCurve(new Point(basketPos2), new Point(avoidPos), new Point(parkPos)));
        toPark.setLinearHeadingInterpolation(basketPos2.getHeading(), avoidPos.getHeading(), 0.25);
        toPark.setPathEndTimeoutConstraint(2.5);

    }

    @Override
    public void init() {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        pathTimer = new Timer();

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        leftExtendo = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtendo = hardwareMap.get(Servo.class, "rightExtendo");
        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
        topLift = hardwareMap.get(DcMotorEx.class, "lift2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");
        door = hardwareMap.get(Servo.class, "door");
        transfer = hardwareMap.get(Servo.class, "trans");
        pitch = hardwareMap.get(Servo.class, "pitch");

        door.setPosition(doorTarget);
        leftV4B.setPosition(leftV4BTarget);
        transfer.setPosition(transferTarget);
        wrist.setPosition(wristTarget);
        leftExtendo.setPosition(lExtTarget);
        rightExtendo.setPosition(1-lExtTarget);
        leftLift.setTargetPosition(liftTarget);
        topLift.setTargetPosition(liftTarget);
        pitch.setPosition(pitchTarget);

    }

    @Override
    public void init_loop(){

        if (!gamepad2.left_bumper) {
            transfer.setPosition(TRANSFER_CLOSED);
        } else {
            transfer.setPosition(TRANSFER_OPEN);
        }

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
        telemetry.addData("topLift Pos", topLift.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.update();
    }

    @Override
    public void loop() {

        autonomousPathUpdate();
        follower.update();
        leftV4B.setPosition(leftV4BTarget);
        pitch.setPosition(pitchTarget);
        door.setPosition(doorTarget);
        wrist.setPosition(wristTarget);
        leftLift.setPower(1);
        topLift.setPower(1);
        leftLift.setTargetPosition(liftTarget);
        topLift.setTargetPosition(liftTarget);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtendo.setPosition(lExtTarget);
        rightExtendo.setPosition(1-lExtTarget);
        transfer.setPosition(transferTarget);
        intake.setPower(intakePower);

        telemetry.addData("transfer Pos", transfer.getPosition());
        telemetry.addData("4Bar Pos", leftV4B.getPosition());
        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
        telemetry.addData("topLift Pos", topLift.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("Intake", intake.getPower());
        telemetry.update();

    }

    public void setPathState(int state) {
//        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void autonomousPathUpdate(){

        if (pathTimer.getElapsedTime() > 1550)
            leftV4BTarget = V4B_OUT;

        if(pathTimer.getElapsedTime() > 2400)
            transferTarget = TRANSFER_OPEN;

        if (pathTimer.getElapsedTime() > 2700)
            leftV4BTarget = V4B_IN;

        if(pathTimer.getElapsedTime() > 3200) {
            follower.followPath(toSample1);
            liftTarget = LIFT_RETRACTED;
        }

        if(pathTimer.getElapsedTime() > 3750){
            intakePower = INTAKE_IN;
            doorTarget = DOOR_CLOSED;
            lExtTarget = EXTENDO_EXTENDED;
            wristTarget = WRIST_INTAKING;
        }

        if(pathTimer.getElapsedTime() > 5250) {
            wristTarget = WRIST_TRANSFERING;
            intakePower = INTAKE_OFF;
            lExtTarget = EXTENDO_RETRACTED;
        }

        if(pathTimer.getElapsedTime() > 6000) {
            doorTarget = DOOR_OPEN;
            transferTarget = TRANSFER_CLOSED;
        }

        if(pathTimer.getElapsedTime() > 6300) {
            follower.followPath(score1);
            liftTarget = LIFT_HIGH_BASKET;
        }

        if (pathTimer.getElapsedTime() > 7500) {
//            follower.breakFollowing();
            leftV4BTarget = V4B_OUT;
        }

        if(pathTimer.getElapsedTime() > 8600)
            transferTarget = TRANSFER_OPEN;

        if (pathTimer.getElapsedTime() > 9000)
            leftV4BTarget = TRANSFER_CLOSED;

//        if(pathTimer.getElapsedTime() > 9300) {
//            follower.followPath(toSample2);
//            liftTarget = 0;
//        }
//
//        if(pathTimer.getElapsedTime() > 4250+6050){
//            intakePower = INTAKE_IN;
//            doorTarget = DOOR_CLOSED;
//            lExtTarget = EXTENDO_EXTENDED;
//            wristTarget = WRIST_INTAKING;
//        }
//
//        if(pathTimer.getElapsedTime() > 6000+6050) {
//            wristTarget = WRIST_UP;
//            intakePower = INTAKE_OFF;
//            lExtTarget = EXTENDO_RETRACTED;
//            doorTarget = DOOR_OPEN;
//        }
//
//        if(pathTimer.getElapsedTime() > 6750+6050)
//            transferTarget = 0.52;
//
//        if(pathTimer.getElapsedTime() > 7050+6050) {
//            follower.followPath(score2);
//            liftTarget = LIFT_HIGH_BASKET;
//        }
//
//        if (pathTimer.getElapsedTime() > 8600+6050) {
////            follower.breakFollowing();
//            leftV4BTarget = 0.85;
//        }
//
//        if(pathTimer.getElapsedTime() > 9600+6050)
//            transferTarget = 0.17;
//
//        if (pathTimer.getElapsedTime() > 10000+6050)
//            leftV4BTarget = 0.12;
//
////        if(pathTimer.getElapsedTime() > 16350) {
////            follower.followPath(toSample3);
////            liftTarget = 0;
////        }
//
//        if (pathTimer.getElapsedTime() > 16500) {
//            liftTarget = 0;
//            follower.breakFollowing();
////            follower.setPose(basketPos2);
////            follower.followPath(toPark);
//        }
//
////        if(pathTimer.getElapsedTime() > 17500){
////            intakePower = INTAKE_IN;
////            lExtTarget = EXTENDO_EXTENDED;
////            wristTarget = WRIST_INTAKING;
////        }
////
////        if(pathTimer.getElapsedTime() > 18700) {
////            wristTarget = WRIST_UP;
////            intakePower = INTAKE_OFF;
////            lExtTarget = EXTENDO_RETRACTED;
////            doorTarget = DOOR_OPEN;
////        }
////
////        if(pathTimer.getElapsedTime() > 19200)
////            transferTarget = 0.52;
////
////        if(pathTimer.getElapsedTime() > 19500) {
////            follower.followPath(score3);
////            liftTarget = LIFT_HIGH_BASKET;
////        }
////
////        if (pathTimer.getElapsedTime() > 21500) {
//////            follower.breakFollowing();
////            leftV4BTarget = 0.85;
////        }
////
////        if(pathTimer.getElapsedTime() > 21800)
////            transferTarget = 0.17;
////
////        if (pathTimer.getElapsedTime() > 22100)
////            leftV4BTarget = 0.12;
//
//        if (pathTimer.getElapsedTime() > 29500) {
//            liftTarget = 0;
//            follower.followPath(toPark);
//        }
    }

    @Override
    public void start() {
        super.start();
        setPathState(0);
        follower.followPath(toBasket);
        liftTarget = LIFT_HIGH_BASKET;
    }

    @Override
    public void stop() { super.stop(); }

}