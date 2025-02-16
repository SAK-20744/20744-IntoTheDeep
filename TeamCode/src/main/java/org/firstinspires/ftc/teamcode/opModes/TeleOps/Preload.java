package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.util.Timer;

@Config
@Autonomous(name = "Preload", group = "Competition")
public class Preload extends OpMode {

    private Follower follower;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private boolean specimenmode = false;

    private double looptime = 0;
    private Timer pathTimer;
    private Servo wrist, door, roll, claw, rail, lDiffy, rDiffy;
    private DcMotorEx lLift, rLift, intake, extendo;
    private DigitalChannel liftLimit, extendoLimit;

//    RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "Color");

    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0.3,
            LDIFFY_TRANSFERING = 0.21, LDIFFY_CLIPPING = 0.3, LDIFFY_SCORING = 0.64, LDIFFY_WALL = 1,
            RDIFFY_TRANSFERING = 0.38, RDIFFY_CLIPPING = 0.3, RDIFFY_SCORING = 0.7, RDIFFY_WALL = 0.8,
            CLAW_CLOSED = 0.55, CLAW_OPEN = 0.0,
            WRIST_TRANSFERING = 0.82, WRIST_UP = 0.4, WRIST_INTAKING = 0.13,
            DOOR_OPEN = 0.6, DOOR_CLOSED = 0.2,
            ROLL_DEPO = 0.7, ROLL_TRANSFERING = 0,
            RAIL_TRANSFERING = .5, RAIL_WALL= 1, RAIL_SCORING = 1, RAIL_CLIPPING = 0.6;

    public static int
            LIFT_RETRACTED = -25, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1250, LIFT_HIGH_RUNG = 700, LIFT_MID_RUNG = 300, clipRange = 200,
            EXTENDO_RETRACTED = 5, EXTENDO_EXTENDED = 450;

    private int liftTarget = LIFT_RETRACTED;
    private int liftLiftedTarget = LIFT_HIGH_BASKET;
    private int extendoTarget = EXTENDO_RETRACTED;

    private double lDiffyTarget = LDIFFY_TRANSFERING;
    private double rDiffyTarget = RDIFFY_TRANSFERING;

    private double clawTarget = CLAW_OPEN;
    private double wristTarget = WRIST_TRANSFERING;
    private double doorTarget = DOOR_OPEN;
    private double intakePower = INTAKE_OFF;
    private double rollTarget = ROLL_TRANSFERING;

    private double railTarget = RAIL_TRANSFERING;

    private PIDController liftPID;
    public static double lp = -0.0073, li = 0, ld = 0.0000028;

    private PIDController extendoPID;
    public static double ep = 0.038, ei = 0, ed = 0.000005;

    @Override
    public void init() {

        pathTimer = new Timer();

        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftPID = new PIDController(lp, li, ld);
        extendoPID = new PIDController(ep, ei, ed);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        extendoLimit = hardwareMap.get(DigitalChannel.class, "extendoLimit");

        lLift = hardwareMap.get(DcMotorEx.class, "lLift");
        rLift = hardwareMap.get(DcMotorEx.class, "rLift");
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        roll = hardwareMap.get(Servo.class, "roll");
        door = hardwareMap.get(Servo.class, "door");
        claw = hardwareMap.get(Servo.class, "claw");
        lDiffy = hardwareMap.get(Servo.class, "lDiffy");
        rDiffy = hardwareMap.get(Servo.class, "rDiffy");
        rail = hardwareMap.get(Servo.class, "rail");

        lLift.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePower = INTAKE_OFF;

        door.setPosition(doorTarget);
        roll.setPosition(rollTarget);
        rail.setPosition(railTarget);
        wrist.setPosition(wristTarget);
//        extendo.setTargetPosition(extendoTarget);
//        lLift.setTargetPosition(liftTarget);
//        rLift.setTargetPosition(liftTarget);
        claw.setPosition(clawTarget);
        lDiffy.setPosition(lDiffyTarget);
        rDiffy.setPosition(rDiffyTarget);
        rail.setPosition(railTarget);
        intake.setPower(intakePower);

    }

    public void init_loop(){

        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intake.setPower(0);

        if (!gamepad2.right_bumper) {
            claw.setPosition(CLAW_CLOSED);
            door.setPosition(DOOR_CLOSED);
        } else {
            claw.setPosition(CLAW_OPEN);
            door.setPosition(DOOR_OPEN);
        }

        extendoTarget = EXTENDO_RETRACTED;
        intakePower = INTAKE_OFF;
        wristTarget = WRIST_UP;
        doorTarget = DOOR_CLOSED;
        clawTarget = CLAW_CLOSED;
        lDiffyTarget = LDIFFY_TRANSFERING;
        rDiffyTarget = RDIFFY_TRANSFERING;
        rollTarget = ROLL_TRANSFERING;
        railTarget = RAIL_TRANSFERING;

        telemetry.addData("lLift Current", lLift.getCurrentPosition());
        telemetry.addData("rLift Current", rLift.getCurrentPosition());
        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("Extendo Limit", extendoLimit.getState());
        telemetry.update();
    }

    public void autonomousPathUpdate() {

        extendoTarget = EXTENDO_RETRACTED;
        intakePower = INTAKE_OFF;
        wristTarget = WRIST_UP;
        doorTarget = DOOR_OPEN;
        clawTarget = CLAW_CLOSED;
        lDiffyTarget = LDIFFY_TRANSFERING;
        rDiffyTarget = RDIFFY_TRANSFERING;
        rollTarget = ROLL_TRANSFERING;
        railTarget = RAIL_TRANSFERING;

        leftFront.setPower(0.35);
        leftRear.setPower(0.35);
        rightFront.setPower(0.35);
        rightRear.setPower(0.35);

        if(pathTimer.getElapsedTime() > 10){
            leftFront.setPower(-0.6);
            leftRear.setPower(0.6);
            rightFront.setPower(0.6);
            rightRear.setPower(-0.6);
        }
        if(pathTimer.getElapsedTime() > 800){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        if(pathTimer.getElapsedTime() > 1400){
            leftFront.setPower(0.5);
            leftRear.setPower(0.5);
            rightFront.setPower(-0.5);
            rightRear.setPower(-0.5);
        }
        if(pathTimer.getElapsedTime() > 1800){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(-0);
            rightRear.setPower(-0);
        }
        if (pathTimer.getElapsedTime() > 2200)
        liftTarget = LIFT_HIGH_BASKET;
        if(pathTimer.getElapsedTime() > 2800){
            lDiffyTarget = LDIFFY_SCORING;
            rDiffyTarget = RDIFFY_SCORING;
            railTarget = RAIL_SCORING;
        }
        if(pathTimer.getElapsedTime() > 3200){
            leftFront.setPower(-0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightRear.setPower(-0.5);
        }
        if(pathTimer.getElapsedTime() > 3400){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
        if(pathTimer.getElapsedTime() > 3500){
            clawTarget = CLAW_OPEN;
        }
        if(pathTimer.getElapsedTime() > 3800)
            liftTarget = LIFT_RETRACTED;
        if(pathTimer.getElapsedTime() > 4300) {
            lDiffyTarget = LDIFFY_TRANSFERING;
            rDiffyTarget = RDIFFY_TRANSFERING;
            railTarget = RAIL_TRANSFERING;
            clawTarget = CLAW_OPEN;
            doorTarget = DOOR_CLOSED;
        }
        if(pathTimer.getElapsedTime() > 4700){
            leftFront.setPower(-0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);
        }
        if(pathTimer.getElapsedTime() > 5200){
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(-0);
            rightRear.setPower(-0);
        }
        if(pathTimer.getElapsedTime()>5500){
            extendoTarget = EXTENDO_EXTENDED;
            wristTarget = WRIST_INTAKING;
            intakePower = INTAKE_IN;
        }
        if(pathTimer.getElapsedTime()>6500){
            extendoTarget = EXTENDO_RETRACTED;
            wristTarget = WRIST_TRANSFERING;
            intakePower = INTAKE_OFF;
        }
    }

    @Override
    public void loop() {

        autonomousPathUpdate();

        door.setPosition(doorTarget);
        roll.setPosition(rollTarget);
        rail.setPosition(railTarget);
        wrist.setPosition(wristTarget);
        extendo.setTargetPosition(extendoTarget);
        lLift.setTargetPosition(liftTarget);
        rLift.setTargetPosition(liftTarget);
        claw.setPosition(clawTarget);
        lDiffy.setPosition(lDiffyTarget);
        rDiffy.setPosition(rDiffyTarget);
        rail.setPosition(railTarget);
        intake.setPower(intakePower);

        liftPID.setPID(lp,li,ld);
        int pos = rLift.getCurrentPosition();
        double power = liftPID.calculate(pos, liftTarget);

        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lLift.setPower(power);
        rLift.setPower(power);

        extendoPID.setPID(ep,ei,ed);
        int expos = extendo.getCurrentPosition();
        double expower = extendoPID.calculate(expos, extendoTarget);
        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        extendo.setPower(expower);

        telemetry.addData("Claw Pos", claw.getPosition());
        telemetry.addData("Wrist Pos", wrist.getPosition());
        telemetry.addData("lLift Current", lLift.getCurrentPosition());
        telemetry.addData("rLift Current", rLift.getCurrentPosition());
        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("Extendo Limit", extendoLimit.getState());
        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Specimen?", specimenmode);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - looptime));
        looptime = loop;
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
    }

    @Override
    public void stop() { super.stop(); }

}
