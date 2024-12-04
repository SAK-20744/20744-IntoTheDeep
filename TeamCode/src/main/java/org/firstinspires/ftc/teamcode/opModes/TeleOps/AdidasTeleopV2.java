package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private Pose basketLoc;

    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.285, V4B_OUT = 0.5, TRANSFER_CLOSED = 0.52, TRANSFER_OPEN = 0.17, EXTENDO_RETRACTED = 0.11, EXTENDO_EXTENDED = 0.5, WRIST_TRANSFERING = 0.12, WRIST_UP = 0.78, WRIST_INTAKING = 0.882, DOOR_OPEN = 0.5, DOOR_CLOSED = 0.87, PITCH_DEPO = 0.3, PITCH_TRANSFERING = 0.765;
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

    private boolean locSet = false;

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
            wristTarget = WRIST_TRANSFERING;

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

//        if(gamepad2.a) {
//            basketLoc = follower.getPose();
//            locSet = true;
//        }e
//
//        if(gamepad2.b && locSet)
//            follower.holdPoint(basketLoc);
//        else

        pitch.setPosition(pitchTarget);
        leftV4B.setPosition(leftV4BTarget);
        door.setPosition(doorTarget);
        wrist.setPosition(wristTarget);

        if(liftTarget == LIFT_RETRACTED) {
            leftLift.setPower(0.7);
            topLift.setPower(0.7);
        }
        else{
            leftLift.setPower(1);
            topLift.setPower(1);
        }

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
        telemetry.addData("Lift Limit", liftLimit.getState());
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
