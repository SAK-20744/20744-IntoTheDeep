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

@Autonomous(name = "0+4")
public class NewAuto extends OpMode{

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, topLift, intake;
    private DigitalChannel liftLimit;

    //fix wristintaking, and dooropen
    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.365, V4B_OUT = 0.6, TRANSFER_CLOSED = 0.35, TRANSFER_OPEN = 0, EXTENDO_RETRACTED = 0.08, EXTENDO_EXTENDED = 0.7, WRIST_TRANSFERING = 0.18, WRIST_UP = 0.78, WRIST_INTAKING = 0.882, DOOR_OPEN = 0.5, DOOR_CLOSED = 0.93, PITCH_DEPO = 0.5, PITCH_TRANSFERING = 0.865;
    public static int LIFT_RETRACTED = 0,LIFT_MID_BASKET = -1450 ,LIFT_HIGH_BASKET = -2700;

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
    private Pose basketPos = new Pose(8,19, Math.toRadians(-22.5));

    private Pose sample1Pos = new Pose(11,19.5, Math.toRadians(0));
    private Pose sample2Pos = new Pose(10.885,11.3, Math.toRadians(0));
    private Pose sample3Pos = new Pose(11,15, Math.toRadians(25));

    private Path toBasket,toPark;
    private Timer pathTimer;
//    private int pathState;

    public void buildPaths() {
        toBasket = new Path(new BezierLine(new Point(startPose), new Point(basketPos)));
        toBasket.setLinearHeadingInterpolation(startPose.getHeading(), basketPos.getHeading(), 0.5);
        toBasket.setPathEndTimeoutConstraint(2.5);
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

        if (pathTimer.getElapsedTime() > 3000) {
            leftV4BTarget = V4B_OUT;
            pitchTarget = PITCH_DEPO;
        }

        if(pathTimer.getElapsedTime() > 4000)
            transferTarget = TRANSFER_OPEN;

        if (pathTimer.getElapsedTime() > 7000) {
            leftV4BTarget = V4B_IN;
            pitchTarget = PITCH_TRANSFERING;
            liftTarget = LIFT_RETRACTED;
            lExtTarget = EXTENDO_EXTENDED;
            wristTarget = WRIST_UP;
        }
    }

    @Override
    public void start() {
        super.start();
        setPathState(0);
        transferTarget = TRANSFER_CLOSED;
        doorTarget = DOOR_OPEN;
        leftV4BTarget = V4B_IN;
        wristTarget = WRIST_TRANSFERING;
        follower.followPath(toBasket);
        liftTarget = LIFT_HIGH_BASKET;
        pitchTarget = PITCH_TRANSFERING;
    }

    @Override
    public void stop() { super.stop(); }

}