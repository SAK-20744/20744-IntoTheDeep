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
//@Autonomous(name = "0+4new")
//public class Sample4Auto extends OpMode{
//
//    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
//    private DcMotorEx leftLift, intake;
//    private DigitalChannel liftLimit;
//
//    //fix wristintaking, and dooropen
//    private final double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.12, V4B_OUT = 0.85, TRANSFER_CLOSED = 0.52, TRANSFER_OPEN = 0.17, EXTENDO_RETRACTED = 0.05, EXTENDO_EXTENDED = 0.7, WRIST_UP = 0.4, WRIST_INTAKING = 1, DOOR_OPEN = 0.5, DOOR_CLOSED = 1;
//    private final int LIFT_RETRACTED = 0, LIFT_HIGH_BASKET = -2800;
//
//    private int liftTarget = LIFT_RETRACTED;
//    private double leftV4BTarget = V4B_IN;
//    private double transferTarget = TRANSFER_CLOSED;
//    private double lExtTarget = EXTENDO_RETRACTED;
//    private double wristTarget = WRIST_UP;
//    private double doorTarget = DOOR_OPEN;
//    private double intakePower = INTAKE_OFF;
//
//    private Follower follower;
//    private Pose startPos = new Pose(8.500, 112.000, Math.toRadians(0));
//    private Pose basketPos = new Pose(16.000, 128.000, Math.toRadians(-45));
//    private Pose sample1Pos = new Pose(30.000, 131.000, Math.toRadians(0));
//    private Pose sample2Pos = new Pose(30.00, 121.000, Math.toRadians(0));
//    private Pose sample3Pos = new Pose(26.000, 133.000, Math.toRadians(20));
//
//    private Pose avoidPos = new Pose(56.000, 124.000, Point.CARTESIAN);
//    private Pose parkPos = new Pose(60.000, 110.000, Math.toRadians(-90));
//
//    private Path toBasket, toSample1, score1, toSample2, score2,toSample3, score3, toAvoid, toPark;
//    private Timer pathTimer;
////    private int pathState;
//
//    public void buildPaths() {
//
//        follower.setHeadingOffset(-90);
//        toBasket = new Path(new BezierLine(new Point(startPos), new Point(basketPos)));
//        toBasket.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
//
//        toSample1 = new Path(new BezierLine(new Point(basketPos), new Point(sample1Pos)));
//        toSample1.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0), 0.3);
//
//        score1 = new Path(new BezierLine(new Point(sample1Pos), new Point(basketPos)));
//        score1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
//
//        toSample2 = new Path(new BezierLine(new Point(basketPos), new Point(sample2Pos)));
//        toSample2.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0), 0.3);
//
//        score2 = new Path(new BezierLine(new Point(sample2Pos), new Point(basketPos)));
//        score2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));
//
//        toSample3 = new Path(new BezierLine(new Point(basketPos), new Point(sample3Pos)));
//        toSample3.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(20), 0.3);
//
//        score3 = new Path(new BezierLine(new Point(sample3Pos), new Point(basketPos)));
//        score3.setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-45));
//
//        toPark = new Path(new BezierCurve(new Point(basketPos), new Point(avoidPos), new Point(parkPos)));
//        toPark.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90), 0.5);
//
//    }
//
//    @Override
//    public void init() {
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPos);
//        buildPaths();
//
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
//        intake.setPower(intakePower);
//
//        telemetry.addData("transfer Pos", transfer.getPosition());
//        telemetry.addData("4Bar Pos", leftV4B.getPosition());
//        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
//        telemetry.addData("Lift Limit", liftLimit.getState());
//        telemetry.addData("Intake", intake.getPower());
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
//        follower.followPath(toBasket);
//        liftTarget = -2800;
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
//        if(pathTimer.getElapsedTime() > 3200) {
//            follower.followPath(toSample1);
//            liftTarget = 0;
//        }
//
//        if(pathTimer.getElapsedTime() > 3750){
//            intakePower = INTAKE_IN;
//            doorTarget = DOOR_CLOSED;
//            lExtTarget = EXTENDO_EXTENDED;
//            wristTarget = WRIST_INTAKING;
//        }
//
//        if(pathTimer.getElapsedTime() > 5250) {
//            wristTarget = WRIST_UP;
//            intakePower = INTAKE_OFF;
//            lExtTarget = EXTENDO_RETRACTED;
//            doorTarget = DOOR_OPEN;
//        }
//
//        if(pathTimer.getElapsedTime() > 6000)
//            transferTarget = 0.52;
//
//        if(pathTimer.getElapsedTime() > 6300) {
//            follower.followPath(score1);
//            liftTarget = LIFT_HIGH_BASKET;
//        }
//
//        if (pathTimer.getElapsedTime() > 7500) {
////            follower.breakFollowing();
//            leftV4BTarget = 0.85;
//        }
//
//        if(pathTimer.getElapsedTime() > 8600)
//            transferTarget = 0.17;
//
//        if (pathTimer.getElapsedTime() > 9000)
//            leftV4BTarget = 0.12;
//
//        if(pathTimer.getElapsedTime() > 9700) {
//            follower.followPath(toSample2);
//            liftTarget = 0;
//        }
//
//        if(pathTimer.getElapsedTime() > 11250){
//            intakePower = INTAKE_IN;
//            doorTarget = DOOR_CLOSED;
//            lExtTarget = EXTENDO_EXTENDED;
//            wristTarget = WRIST_INTAKING;
//        }
//
//        if(pathTimer.getElapsedTime() > 12050) {
//            wristTarget = WRIST_UP;
//            intakePower = INTAKE_OFF;
//            lExtTarget = EXTENDO_RETRACTED;
//            doorTarget = DOOR_OPEN;
//        }
//
//        if(pathTimer.getElapsedTime() > 12800)
//            transferTarget = 0.52;
//
//        if(pathTimer.getElapsedTime() > 13200) {
//            follower.followPath(score2);
//            liftTarget = LIFT_HIGH_BASKET;
//        }
//
//        if (pathTimer.getElapsedTime() > 14650) {
////            follower.breakFollowing();
//            leftV4BTarget = 0.85;
//        }
//
//        if(pathTimer.getElapsedTime() > 16600)
//            transferTarget = 0.17;
//
//        if (pathTimer.getElapsedTime() > 17000)
//            leftV4BTarget = 0.12;
//
//        if(pathTimer.getElapsedTime() > 18000) {
//            follower.followPath(toSample3);
//            liftTarget = 0;
//        }
////
////        if (pathTimer.getElapsedTime() > 16500) {
////            liftTarget = 0;
////            follower.breakFollowing();
//////            follower.setPose(basketPos2);
//////            follower.followPath(toPark);
////        }
//
//        if(pathTimer.getElapsedTime() > 19000){
//            intakePower = INTAKE_IN;
//            lExtTarget = EXTENDO_EXTENDED;
//            wristTarget = WRIST_INTAKING;
//        }
//
//        if(pathTimer.getElapsedTime() > 20000) {
//            wristTarget = WRIST_UP;
//            intakePower = INTAKE_OFF;
//            lExtTarget = EXTENDO_RETRACTED;
//            doorTarget = DOOR_OPEN;
//        }
//
//        if(pathTimer.getElapsedTime() > 20500)
//            transferTarget = 0.52;
//
//        if(pathTimer.getElapsedTime() > 21000) {
//            follower.followPath(score3);
//            liftTarget = LIFT_HIGH_BASKET;
//        }
//
//        if (pathTimer.getElapsedTime() > 23000) {
////            follower.breakFollowing();
//            leftV4BTarget = 0.85;
//        }
//
//        if(pathTimer.getElapsedTime() > 24200)
//            transferTarget = 0.17;
//
//        if (pathTimer.getElapsedTime() > 25000)
//            leftV4BTarget = 0.12;
//
//        if (pathTimer.getElapsedTime() > 26500) {
//            liftTarget = 0;
//            follower.followPath(toPark);
//        }
//    }
//
//    @Override
//    public void start() {
//        super.start();
//        setPathState(0);
//
//    }
//
//    @Override
//    public void stop() { super.stop(); }
//
//}