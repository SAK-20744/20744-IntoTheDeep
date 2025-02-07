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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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

    public static double pX = 0.05, iX = 0, dX = 0.05;
    public static double pY = 0.055, iY = 0, dY = 0.35;
    public static double pTurn = 0.045, iTurn = 0, dTurn = 0.05;

    PIDFController speedController;
    PIDFController strafeController;
    PIDFController turnController;

    double forwardsback = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;

    public static double targetStrafe = 0;
    public static double targetDrive = 12;

    private double looptime = 0;
    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift, topLift, intake;
    private DigitalChannel liftLimit;
    private AnalogInput clawInput;

    public Limelight3A LL;

    private boolean targetFound = false;

    private boolean aPressed = true, bPressed = false;

    public static double INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_OFF = 0, V4B_IN = 0.25, V4B_OUT = 0.6, TRANSFER_CLOSED = 0.35, TRANSFER_OPEN = 0, EXTENDO_RETRACTED = 0, EXTENDO_EXTENDED = 0.65, WRIST_TRANSFERING = 0, WRIST_UP = 0.7, WRIST_INTAKING = 0.882, DOOR_OPEN = 0.6, DOOR_CLOSED = 0.93, PITCH_DEPO = 0.5, PITCH_TRANSFERING = 0.75;
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

        speedController = new PIDFController(pX, iX, dX, 0);
        strafeController = new PIDFController(pY, iY, dY, 0);
        turnController = new PIDFController(pTurn, iTurn, dTurn, 0);

        LL = hardwareMap.get(Limelight3A.class, "ll3");
        LL.pipelineSwitch(0);
        LL.start();
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

        targetFound = false;
        LL.pipelineSwitch(0);
        LLResult yel = LL.getLatestResult();
//        LL.pipelineSwitch(3);
//        LLResult blu = LL.getLatestResult();
        if (yel != null) {
            if (yel.isValid()) {
                double detectedX = yel.getTx();
                double detectedY = yel.getTy();

                telemetry.addData("yel x", detectedX);
                telemetry.addData("yel y", detectedY);
                targetFound = true;
            }
        }
//        if (blu != null) {
//            if (blu.isValid()) {
//                telemetry.addData("blu x", blu.getTx());
//                telemetry.addData("blu y", blu.getTy());
//                targetFound = true;
//            }
//        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad2.left_bumper && targetFound) {
//                turn = turnController.calculate(targetTurn, desiredTag.ftcPose.pitch);
                strafe = (strafeController.calculate(targetStrafe, -1*yel.getTx()));
                forwardsback = speedController.calculate(targetDrive, yel.getTy());

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", forwardsback, strafe, gamepad1.right_stick_x);
                moveRobot(forwardsback, strafe, 0);
            }
            else{
        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
        }

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

    public void moveRobot(double x, double y, double pitch) {
        // Calculate wheel powers.
        double leftFrontPower    =  x - y - pitch;
        double rightFrontPower   =  x + y + pitch;
        double leftBackPower     =  x + y - pitch;
        double rightBackPower    =  x - y + pitch;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftBackPower);
        rightRear.setPower(rightBackPower);
    }

}
