package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;

@Config
@TeleOp(name = "Adidas Teleop V2", group = "Competition")
public class AdidasTeleopV2 extends OpMode {

    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, topLift, intake;
    private DigitalChannel liftLimit;
    private AnalogInput clawInput;

    private boolean aPressed = true, bPressed = false;

    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.365, V4B_OUT = 0.24, TRANSFER_CLOSED = 0.35, TRANSFER_OPEN = 0, EXTENDO_RETRACTED = 0, EXTENDO_EXTENDED = 0.65, WRIST_TRANSFERING = 0, WRIST_UP = 0.7, WRIST_INTAKING = 0.882, DOOR_OPEN = 0.5, DOOR_CLOSED = 0.93, PITCH_DEPO = 0.5, PITCH_TRANSFERING = 0.73;
    public static int LIFT_RETRACTED = 105, LIFT_MID_BASKET = 500 ,LIFT_HIGH_BASKET = 1100;

    public static double SpecV4B_IN = 0.365, SpecV4B_OUT = 0.6, SpecPITCH_DEPO = 0.5, SpecPITCH_TRANSFERING = 0.89;
    public static int SpecLIFT_RETRACTED = 0, SpecMid = -200 , SpecHigh = 450;

    private int liftTarget = LIFT_RETRACTED;
    private int liftLiftedTarget = LIFT_HIGH_BASKET;
    private double leftV4BTarget = V4B_IN;
    private double transferTarget = TRANSFER_OPEN;
    private double lExtTarget = EXTENDO_RETRACTED;
    private double wristTarget = WRIST_TRANSFERING;
    private double doorTarget = DOOR_OPEN;
    private double intakePower = INTAKE_OFF;
    private double pitchTarget = PITCH_TRANSFERING;

    private boolean locSet = false;
    private PIDController liftPID;
    public static double p = 0.015, i = 0, d = 0.0005;

    @Override
    public void init() {

        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        liftPID = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
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
        clawInput = hardwareMap.get(AnalogInput.class, "clawPos");

        topLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void init_loop(){

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Top Lift Current", topLift.getCurrentPosition());
        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.update();
    }

    @Override
    public void loop() {

//        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        follower.update();

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
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
            lExtTarget = EXTENDO_EXTENDED;
//            doorTarget = DOOR_CLOSED;
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
            lExtTarget = EXTENDO_RETRACTED;
//            doorTarget = DOOR_OPEN;
//            wristTarget = WRIST_TRANSFERING;

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
            transferTarget = TRANSFER_CLOSED;
            doorTarget = DOOR_OPEN;
        } else {
            transferTarget = TRANSFER_OPEN;
            doorTarget = DOOR_CLOSED;
        }

        if(gamepad2.dpad_down || gamepad1.dpad_down)
            liftLiftedTarget = LIFT_MID_BASKET;
        if(gamepad2.dpad_up || gamepad1.dpad_up)
            liftLiftedTarget = LIFT_HIGH_BASKET;

        if(gamepad1.a) {
            liftTarget = LIFT_RETRACTED;
            leftV4BTarget = V4B_IN;
            pitchTarget = PITCH_TRANSFERING;
        }

        if(gamepad1.b) {
            liftTarget = liftLiftedTarget;
            leftV4BTarget = V4B_OUT;
            pitchTarget = PITCH_DEPO;
        }

        if(gamepad1.right_trigger > 0.5)
            liftTarget -= 75;

        pitch.setPosition(pitchTarget);
        leftV4B.setPosition(leftV4BTarget);
        door.setPosition(doorTarget);
        wrist.setPosition(wristTarget);

        leftExtendo.setPosition(lExtTarget);
        rightExtendo.setPosition(1-lExtTarget);
        transfer.setPosition(transferTarget);
        intake.setPower(intakePower);

        liftPID.setPID(p,i,d);
        int pos = leftLift.getCurrentPosition();
        double power = liftPID.calculate(pos, liftTarget);

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            topLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftLift.setPower(power);
        topLift.setPower(power);

        telemetry.addData("transfer Pos", transfer.getPosition());
        telemetry.addData("4Bar Pos", leftV4B.getPosition());
        telemetry.addData("lift pos", pos);
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.addData("lift target", liftTarget);

        telemetry.addData("Intake", intake.getPower());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() { super.stop(); }

}
