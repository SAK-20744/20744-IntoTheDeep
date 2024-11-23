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

@Autonomous(name = "Square")
public class MovementTesting extends OpMode{

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, intake;
    private DigitalChannel liftLimit;

    //fix wristintaking, and dooropen
    private final double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.12, V4B_OUT = 0.85, TRANSFER_CLOSED = 0.52, TRANSFER_OPEN = 0.17, EXTENDO_RETRACTED = 0.05, EXTENDO_EXTENDED = 0.7, WRIST_UP = 0.4, WRIST_INTAKING = 1, DOOR_OPEN = 0.5, DOOR_CLOSED = 1;
    private final int LIFT_RETRACTED = 0, LIFT_HIGH_BASKET = -2800;

    private int liftTarget = LIFT_RETRACTED;
    private double leftV4BTarget = V4B_IN;
    private double transferTarget = TRANSFER_CLOSED;
    private double lExtTarget = EXTENDO_RETRACTED;
    private double wristTarget = WRIST_UP;
    private double doorTarget = DOOR_OPEN;
    private double intakePower = INTAKE_OFF;

    private Follower follower;

    private Pose corner1 = new Pose(0,0, Math.toRadians(0));
    private Pose corner2 = new Pose(15,0, Math.toRadians(-90));
    private Pose corner3 = new Pose(15,15, Math.toRadians(-180));
    private Pose corner4 = new Pose(0,15, Math.toRadians(-270));


    private Path move1, move2, move3, move4;
    private Timer pathTimer;
//    private int pathState;

    public void buildPaths() {
        move1 = new Path(new BezierLine(new Point(corner1), new Point(corner2)));
        move1.setLinearHeadingInterpolation(corner1.getHeading(), corner2.getHeading());

        move2 = new Path(new BezierLine(new Point(corner2), new Point(corner3)));
        move2.setLinearHeadingInterpolation(corner2.getHeading(), corner3.getHeading(), 0.5);

        move3 = new Path(new BezierLine(new Point(corner3), new Point(corner4)));
        move3.setLinearHeadingInterpolation(corner3.getHeading(), corner4.getHeading());

        move4 = new Path(new BezierLine(new Point(corner4), new Point(corner1)));
        move4.setLinearHeadingInterpolation(corner4.getHeading(), corner1.getHeading());

    }

    @Override
    public void init() {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(corner1);
        buildPaths();
        pathTimer = new Timer();

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        leftExtendo = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtendo = hardwareMap.get(Servo.class, "rightExtendo");
        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
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
        pitch.setPosition(0.88);

    }

    @Override
    public void init_loop(){

        if (!gamepad2.left_bumper) {
            transfer.setPosition(0.52);
        } else {
            transfer.setPosition(0.17);
        }

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.update();
    }

    @Override
    public void loop() {

        autonomousPathUpdate();
        follower.update();
        leftV4B.setPosition(leftV4BTarget);
        pitch.setPosition(0.88);
        door.setPosition(doorTarget);
        wrist.setPosition(wristTarget);
        leftLift.setPower(1);
        leftLift.setTargetPosition(liftTarget);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtendo.setPosition(lExtTarget);
        rightExtendo.setPosition(1-lExtTarget);
        transfer.setPosition(transferTarget);
        intake.setPower(intakePower);

        telemetry.addData("transfer Pos", transfer.getPosition());
        telemetry.addData("4Bar Pos", leftV4B.getPosition());
        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
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

        follower.followPath(move1);

        if(pathTimer.getElapsedTime() > 25000)
            follower.followPath(move2);

//        if(pathTimer.getElapsedTime() > 26000)
//            follower.followPath(move3);

//        if(pathTimer.getElapsedTime() > 24000)
//            follower.followPath(move4);
    }

    @Override
    public void start() {
        super.start();
        setPathState(0);

    }

    @Override
    public void stop() { super.stop(); }

}