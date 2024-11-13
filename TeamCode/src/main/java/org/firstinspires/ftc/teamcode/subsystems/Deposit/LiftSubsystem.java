package org.firstinspires.ftc.teamcode.subsystems.Deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LiftSubsystem {

    public DcMotorEx leftLift, rightLift;

    public final PIDFCoefficients
            LIFT_UP_VELOCITY_PIDF_COEFFICIENTS = new PIDFCoefficients(9,6,0,4),
            LIFT_UP_POSITION_PIDF_COEFFICIENTS = new PIDFCoefficients(10,0,0,0),
            LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS = new PIDFCoefficients(10,2,0,-4.5),
            LIFT_DOWN_POSITION_PIDF_COEFFICIENTS = new PIDFCoefficients(5,0,0,0);

    public static int
            LIFT_TOLERANCE = 4,
            LIFT_MAX = 520,
            LIFT_VELOCITY_TOLERANCE = 28*4/6,
            LIFT_VELOCITY = 1200;

    public int leftLiftTargetPosition, rightLiftTargetPosition, leftLiftCurrentAdjust = 0, rightLiftCurrentAdjust = 0;

    public void initialize() {

//        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
//        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftTargetPosition = 0;
//        rightLiftTargetPosition = 0;
        leftLift.setTargetPosition(leftLiftTargetPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLift.setTargetPosition(rightLiftTargetPosition);
//        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setLiftTarget(int pos){
        leftLiftTargetPosition = pos;
    }

    public void updateLiftMotors() {
//        if (leftLiftTargetPosition < 0 && !presetInMotion && !liftGoing) leftLiftTargetPosition = 0;
        if (leftLiftTargetPosition > LIFT_MAX) leftLiftTargetPosition = LIFT_MAX;
        correctLiftError();
        if (leftLiftTargetPosition >= leftLift.getCurrentPosition()-5) {
            leftLift.setVelocityPIDFCoefficients(LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.f);
            rightLift.setVelocityPIDFCoefficients(LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_UP_VELOCITY_PIDF_COEFFICIENTS.f);
            leftLift.setPositionPIDFCoefficients(LIFT_UP_POSITION_PIDF_COEFFICIENTS.p);
            rightLift.setPositionPIDFCoefficients(LIFT_UP_POSITION_PIDF_COEFFICIENTS.p);
        } else {
            leftLift.setVelocityPIDFCoefficients(LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.f);
            rightLift.setVelocityPIDFCoefficients(LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.p,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.i,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.d,LIFT_DOWN_VELOCITY_PIDF_COEFFICIENTS.f);
            leftLift.setPositionPIDFCoefficients(LIFT_DOWN_POSITION_PIDF_COEFFICIENTS.p);
            rightLift.setPositionPIDFCoefficients(LIFT_DOWN_POSITION_PIDF_COEFFICIENTS.p);
        }
        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setTargetPosition(leftLiftTargetPosition + leftLiftCurrentAdjust);
        rightLift.setTargetPosition(rightLiftTargetPosition + rightLiftCurrentAdjust);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setTargetPosition(leftLiftTargetPosition + leftLiftCurrentAdjust);
        rightLift.setTargetPosition(rightLiftTargetPosition + rightLiftCurrentAdjust);
        leftLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        rightLift.setTargetPositionTolerance(LIFT_TOLERANCE);
        leftLift.setVelocity(LIFT_VELOCITY);
        rightLift.setVelocity(LIFT_VELOCITY);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void correctLiftError() {
        rightLiftTargetPosition = rightLift.getCurrentPosition()+ leftLiftTargetPosition -leftLift.getCurrentPosition();
    }

}

/**
 * 8=D
 */