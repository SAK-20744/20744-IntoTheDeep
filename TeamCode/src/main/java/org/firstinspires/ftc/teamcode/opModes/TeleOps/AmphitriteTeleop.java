package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;

@Config
@TeleOp(name = "Ampy", group = "Competition")
public class AmphitriteTeleop extends OpMode {

    private Follower follower;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private double looptime = 0;
    private Servo wrist, door, roll, claw, rail, lDiffy, rDiffy;
    private DcMotorEx lLift, rLift, intake, extendo;
    private DigitalChannel liftLimit, extendoLimit;

    public static double
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0,
            LDIFFY_TRANSFERING = 0.21, LDIFFY_CLIPPING = 0.45, LDIFFY_SCORING = 0.73, LDIFFY_WALL = 1,
            RDIFFY_TRANSFERING = 0.38, RDIFFY_CLIPPING = 0.15, RDIFFY_SCORING = 0.73, RDIFFY_WALL = 1,
            CLAW_CLOSED = 0.55, CLAW_OPEN = 0.3,
            WRIST_TRANSFERING = 1, WRIST_UP = 0.5, WRIST_INTAKING = 0.35,
            DOOR_OPEN = 0.47, DOOR_CLOSED = 0.05,
            ROLL_DEPO = 0.7, ROLL_TRANSFERING = 0,
            RAIL_TRANSFERING = .65, RAIL_WALL= .8, RAIL_SCORING = 1;

    public static int
            LIFT_RETRACTED = 0, LIFT_MID_BASKET = 500, LIFT_HIGH_BASKET = 1150,
            EXTENDO_RETRACTED = 0, EXTENDO_EXTENDED = 500;

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
    public static double lp = -0.015, li = 0, ld = 0;

    private PIDController extendoPID;
    public static double ep = 0.05, ei = 0, ed = 0;

    @Override
    public void init() {

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

    }

    public void init_loop(){

        if (liftLimit.getState()){
            lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (!extendoLimit.getState()){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("lLift Current", lLift.getCurrentPosition());
        telemetry.addData("rLift Current", rLift.getCurrentPosition());
        telemetry.addData("Extendo Current", extendo.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("Extendo Limit", extendoLimit.getState());
        telemetry.update();
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * 0.7;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

        if (gamepad1.left_bumper) {
            extendoTarget = EXTENDO_EXTENDED;

            if (gamepad1.right_bumper) {
                intakePower = INTAKE_IN;
                wristTarget = WRIST_INTAKING;
            } else if (gamepad1.y) {
                intakePower = INTAKE_OUT;
                wristTarget = WRIST_UP;
            } else {
                intakePower = INTAKE_OFF;
                wristTarget = WRIST_UP;
            }
        } else {
            extendoTarget = EXTENDO_RETRACTED;
            if (gamepad1.right_bumper) {
                intakePower = INTAKE_IN;
                wristTarget = WRIST_INTAKING;
            } else if (gamepad1.y) {
                intakePower = INTAKE_OUT;
                wristTarget = WRIST_UP;
            } else {
                intakePower = INTAKE_OFF;
                wristTarget = WRIST_TRANSFERING;
            }
        }
        if(gamepad2.right_bumper && !gamepad1.left_bumper) {
            clawTarget = CLAW_CLOSED;
            doorTarget = DOOR_OPEN;
        } else {
            clawTarget = CLAW_OPEN;
            doorTarget = DOOR_CLOSED;
        }

        if(gamepad2.dpad_down || gamepad1.dpad_down)
            liftLiftedTarget = LIFT_MID_BASKET;
        if(gamepad2.dpad_up || gamepad1.dpad_up)
            liftLiftedTarget = LIFT_HIGH_BASKET;

        if(gamepad1.a) {
            liftTarget = LIFT_RETRACTED;
            lDiffyTarget = LDIFFY_TRANSFERING;
            rDiffyTarget = RDIFFY_TRANSFERING;
            rollTarget = ROLL_TRANSFERING;
            railTarget = RAIL_TRANSFERING;
        }

        if(gamepad1.b) {
            liftTarget = liftLiftedTarget;
            lDiffyTarget = LDIFFY_SCORING;
            rDiffyTarget = RDIFFY_SCORING;
            rollTarget = ROLL_DEPO;
            railTarget = RAIL_SCORING;
        }

        if(gamepad1.right_trigger > 0.5)
            liftTarget -= 75;

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
        telemetry.update();

        telemetry.addData("Intake", intake.getPower());
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - looptime));
        looptime = loop;
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() { super.stop(); }

}
