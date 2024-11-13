package org.firstinspires.ftc.teamcode.opModes.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Deposit.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Preload")
public class Auto_0_0 extends OpMode{

    private Servo wrist, door, pitch, transfer, leftV4B, leftExtendo, rightExtendo;
    private DcMotorEx leftLift;
    private DigitalChannel liftLimit;

    private int liftTarget = 0;

    private Follower follower;
    private Pose startPose = new Pose(0,0, Math.toRadians(0));
    private Pose basketPos = new Pose(20,15, Math.toRadians(0));
    private Path toBasket, toSample1, score1, toSample2, score2,toSample3, score3, toPark;

    public void buildPaths() {
        toBasket = new Path(new BezierLine(new Point(startPose), new Point(basketPos)));
        toBasket.setConstantHeadingInterpolation(0);
    }

    @Override
    public void init() {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        liftLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        leftExtendo = hardwareMap.get(Servo.class, "leftExtendo");
        rightExtendo = hardwareMap.get(Servo.class, "rightExtendo");
        leftLift = hardwareMap.get(DcMotorEx.class, "lift");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftV4B = hardwareMap.get(Servo.class, "leftV4B");
        door = hardwareMap.get(Servo.class, "door");
        transfer = hardwareMap.get(Servo.class, "trans");

        door.setPosition(0.5);
        leftV4B.setPosition(0.13);
        wrist.setPosition(0.4);
        leftExtendo.setPosition(0.05);
        rightExtendo.setPosition(0.95);

    }

    @Override
    public void loop() {
        follower.update();
        leftV4B.setPosition(0.13);
        wrist.setPosition(0.4);
        door.setPosition(0.5);
        leftLift.setTargetPosition(liftTarget);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtendo.setPosition(0.05);
        rightExtendo.setPosition(0.95);

        if (gamepad2.left_bumper) {
            transfer.setPosition(0.52);
        } else {
            transfer.setPosition(0.17);
        }

        leftLift.setTargetPosition(liftTarget);

        if (!liftLimit.getState()){
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Lift Current", leftLift.getCurrentPosition());
        telemetry.addData("Lift Limit", liftLimit.getState());
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        follower.followPath(toBasket);
        liftTarget = 500;
    }

    @Override
    public void stop() {
        super.stop();
    }

}
