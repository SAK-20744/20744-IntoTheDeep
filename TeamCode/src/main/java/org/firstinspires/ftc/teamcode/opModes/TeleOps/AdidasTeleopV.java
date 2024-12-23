package org.firstinspires.ftc.teamcode.opModes.TeleOps;

import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.subsystems.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;

@Disabled
@Config
@TeleOp(name = "Adidas Teleop new", group = "Competition")
public class AdidasTeleopV extends OpMode {

    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, intake;
    private DigitalChannel liftLimit;

    private Pose basketLoc;

    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.12, V4B_OUT = 0.775, TRANSFER_CLOSED = 0.52, TRANSFER_OPEN = 0.17, EXTENDO_RETRACTED = 0.05, EXTENDO_EXTENDED = 0.7, WRIST_UP = 0.4, WRIST_INTAKING = 1, DOOR_OPEN = 0.5, DOOR_CLOSED = 1;
    public static int LIFT_RETRACTED = 0,LIFT_MID_BASKET = -1450 ,LIFT_HIGH_BASKET = -2850;

    private int liftTarget = LIFT_RETRACTED;
    private int liftLiftedTarget = LIFT_HIGH_BASKET;
    private double leftV4BTarget = V4B_IN;
    private double transferTarget = TRANSFER_OPEN;
    private double lExtTarget = EXTENDO_RETRACTED;
    private double wristTarget = WRIST_UP;
    private double doorTarget = DOOR_OPEN;
    private double intakePower = INTAKE_OFF;

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
    public void loop() {

        if (gamepad1.left_bumper) {
            lExtTarget = EXTENDO_EXTENDED;
            doorTarget = DOOR_CLOSED;
            if (gamepad1.right_bumper) {
                intakePower = INTAKE_IN;
                wristTarget = WRIST_INTAKING;
            } else if (gamepad1.y) {
                intakePower = INTAKE_OUT;
                wristTarget = WRIST_INTAKING;
            } else {
                intakePower = INTAKE_OFF;
                wristTarget = WRIST_UP;
            }
        } else {
            lExtTarget = EXTENDO_RETRACTED;
            doorTarget = DOOR_OPEN;
        }

//        if(gamepad2.right_bumper)
//            transferTarget = TRANSFER_CLOSED;
//        else
//            transferTarget = TRANSFER_OPEN;

        if(gamepad2.dpad_down || gamepad1.dpad_down)
            liftLiftedTarget = LIFT_MID_BASKET;
        if(gamepad2.dpad_up || gamepad1.dpad_up)
            liftLiftedTarget = LIFT_HIGH_BASKET;

        if(gamepad1.a) {
            transferTarget = TRANSFER_OPEN;
            liftTarget = LIFT_RETRACTED;
            leftV4BTarget = V4B_IN;
        }
        if(gamepad1.b) {
            transferTarget = TRANSFER_CLOSED;
            liftTarget = liftLiftedTarget;
            leftV4BTarget = V4B_OUT;
        }

//        if(gamepad2.a) {
//            basketLoc = follower.getPose();
//            locSet = true;
//        }
//
//        if(gamepad2.b && locSet)
//            follower.holdPoint(basketLoc);
//        else

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        pitch.setPosition(0.88);
        leftV4B.setPosition(leftV4BTarget);
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

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() { super.stop(); }

}
