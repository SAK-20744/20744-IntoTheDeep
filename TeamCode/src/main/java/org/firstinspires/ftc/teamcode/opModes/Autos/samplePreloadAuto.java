//package org.firstinspires.ftc.teamcode.opModes.Autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;
//
////LEFT EDGE 1 TILE OVER
//@Autonomous(name = "0+1")
//public class samplePreloadAuto extends OpMode{
//
//    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
//    private DcMotorEx leftLift, intake;
//    private DigitalChannel liftLimit;
//
//    //fix wristintaking, and dooropen
//    private final double INTAKE_IN = 0, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.12, V4B_OUT = 0.85, TRANSFER_CLOSED = 0.52, TRANSFER_OPEN = 0.17, EXTENDO_RETRACTED = 0.05, EXTENDO_EXTENDED = 0.7, WRIST_UP = 0.4, WRIST_INTAKING = 1, DOOR_OPEN = 0, DOOR_CLOSED = 0.5;
//    private final int LIFT_RETRACTED = 0, LIFT_HIGH_BASKET = -2800;
//
//    private int liftTarget = LIFT_RETRACTED;
//    private double leftV4BTarget = V4B_IN;
//    private double transferTarget = TRANSFER_CLOSED;
//    private double lExtTarget = EXTENDO_RETRACTED;
//    private double wristTarget = WRIST_UP;
//    private double doorTarget = DOOR_CLOSED;
//
//    private Follower follower;
//    private Pose startPose = new Pose(0,0, Math.toRadians(0));
//    private Pose sample1Pos = new Pose(12.5,18.5, Math.toRadians(0));
//    private Pose basketPos = new Pose(8.3,17, Math.toRadians(-45));
//    private Pose avoidPos = new Pose(60, 10, Math.toRadians(120));
//    private Pose parkPos = new Pose(53, -15, Math.toRadians(90));
//
//    private Path toBasket, toSample1, score1, toSample2, score2,toSample3, score3, toPark;
//    private Timer pathTimer;
////    private int pathState;
//
//    public void buildPaths() {
//        toBasket = new Path(new BezierLine(new Point(startPose), new Point(basketPos)));
//        toBasket.setLinearHeadingInterpolation(startPose.getHeading(), basketPos.getHeading());
//        toBasket.setPathEndTimeoutConstraint(2.5);
//
//        toSample1 = new Path(new BezierLine(new Point(basketPos), new Point(sample1Pos)));
//        toSample1.setLinearHeadingInterpolation(basketPos.getHeading(), sample1Pos.getHeading());
//        toSample1.setPathEndTimeoutConstraint(2.5);
//
//        toPark = new Path(new BezierCurve(new Point(basketPos), new Point(avoidPos), new Point(parkPos)));
//        toPark.setLinearHeadingInterpolation(basketPos.getHeading(), parkPos.getHeading());
//        toPark.setPathEndTimeoutConstraint(2.5);
//    }
//
//    @Override
//    public void init() {
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//        pathTimer = new Timer();
//
//        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
//        leftExtendo = hardwareMap.get(Servo.class, "leftExtendo");
//        rightExtendo = hardwareMap.get(Servo.class, "rightExtendo");
//        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        leftV4B = hardwareMap.get(Servo.class, "leftV4B");
//        door = hardwareMap.get(Servo.class, "door");
//        transfer = hardwareMap.get(Servo.class, "trans");
//        pitch = hardwareMap.get(Servo.class, "pitch");
//
//        door.setPosition(doorTarget);
//        leftV4B.setPosition(leftV4BTarget);
//        transfer.setPosition(transferTarget);
//        wrist.setPosition(wristTarget);
//        leftExtendo.setPosition(lExtTarget);
//        rightExtendo.setPosition(1-lExtTarget);
//        leftLift.setTargetPosition(liftTarget);
//        pitch.setPosition(0.88);
//
//    }
//
//    @Override
//    public void init_loop(){
//
//        if (!gamepad2.left_bumper) {
//            transfer.setPosition(0.52);
//        } else {
//            transfer.setPosition(0.17);
//        }
//
//        if (!liftLimit.getState()){
//            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
//        telemetry.addData("Lift Limit", liftLimit.getState());
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//
//        autonomousPathUpdate();
//        follower.update();
//        leftV4B.setPosition(leftV4BTarget);
//        pitch.setPosition(0.88);
//        door.setPosition(doorTarget);
//        wrist.setPosition(wristTarget);
//        leftLift.setPower(1);
//        leftLift.setTargetPosition(liftTarget);
//        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftExtendo.setPosition(lExtTarget);
//        rightExtendo.setPosition(1-lExtTarget);
//        transfer.setPosition(transferTarget);
//
//        telemetry.addData("transfer Pos", transfer.getPosition());
//        telemetry.addData("4Bar Pos", leftV4B.getPosition());
//        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
//        telemetry.addData("Lift Limit", liftLimit.getState());
//        telemetry.update();
//
//    }
//
//    public void setPathState(int state) {
////        pathState = state;
//        pathTimer.resetTimer();
//        autonomousPathUpdate();
//    }
//
//    public void autonomousPathUpdate(){
//
//        if (pathTimer.getElapsedTime() > 1550)
//            leftV4BTarget = 0.85;
//
//        if(pathTimer.getElapsedTime() > 2400)
//            transferTarget = 0.17;
//
//        if (pathTimer.getElapsedTime() > 2700)
//            leftV4BTarget = 0.12;
//
//        if (pathTimer.getElapsedTime() > 3200)
//            liftTarget = 0;
//
//        if(pathTimer.getElapsedTime() > 25000)
//            follower.followPath(toPark);
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        setPathState(0);
//        follower.followPath(toBasket);
//        liftTarget = -2800;
//    }
//
//    @Override
//    public void stop() { super.stop(); }
//
//}
